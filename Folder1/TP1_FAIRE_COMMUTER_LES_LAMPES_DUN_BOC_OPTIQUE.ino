/**************************************************************************
 * TP1 - Chenillard VMD (Arduino Uno R3 + MCP2515)
 * ------------------------------------------------------------------------
 * Objectif : Commander cycliquement les feux via le bus CAN.
 * Séquence : Veilleuse → Code → Phare → Clignotant → Éteint → ... (avant)
 *            Veilleuse → Clignotant → Stop → Klaxon/Libre → Éteint → ... (arrière)
 **************************************************************************/

#include <SPI.h>
#include "mcp2515.h"

// ----------------- CONFIGURATION MATÉRIELLE -----------

const int SPI_CS_PIN = 9;              // Broche CS du MCP2515
MCP2515 mcp2515(SPI_CS_PIN);           // Contrôleur CAN

// ------------- CONFIGURATION DES MODULES --------------

// Décommenter UN SEUL module à contrôler :
#define MODULE_AVANT_DROIT
//#define MODULE_AVANT_GAUCHE
//#define MODULE_ARRIERE_DROIT
//#define MODULE_ARRIERE_GAUCHE

// Structure de configuration CAN
struct ModuleConfig {
  const char* nom;    // Nom descriptif
  uint32_t id_im;    // ID pour Input Message (RXF1)
  uint32_t id_aim;  // ID pour Acquittement (TXD1)
  bool is_avant;   // True si module avant, false si arrière
};

// Tableau de configurations pour tous les modules
const ModuleConfig modules[] = {
  {"Feux avant droit",   0x0E880000, 0x0EA00000, true},
  {"Feux avant gauche",  0x0E080000, 0x0E200000, true},
  {"Feux arrière droit", 0x0F880000, 0x0FA00000, false},
  {"Feux arrière gauche",0x0F080000, 0x0F200000, false}
};

// Sélection du module actif
#if defined(MODULE_AVANT_DROIT)
  #define MODULE_INDEX 0
#elif defined(MODULE_AVANT_GAUCHE)
  #define MODULE_INDEX 1
#elif defined(MODULE_ARRIERE_DROIT)
  #define MODULE_INDEX 2
#elif defined(MODULE_ARRIERE_GAUCHE)
  #define MODULE_INDEX 3
#else
  #error "Aucun module sélectionné!"
#endif

const ModuleConfig& module = modules[MODULE_INDEX]; // Référence au module actif

// Registres MCP25050
#define REG_GPDDR 0x1F        // Configuration des ports (GP0-GP7)
#define REG_GPLAT 0x1E        // État des sorties
#define MASQUE_SORTIES 0x0F   // Masque pour les 4 bits de poids faible (GP0-GP3)

// ------------ DÉFINITION DES SÉQUENCES DE FEUX --------------

struct EtatFeu {
  uint8_t valeur;           // Valeur à écrire dans GPLAT
  const char* description;  // Description lisible
};

// Séquences prédéfinies pour les modules avant
const EtatFeu sequenceAvant[] = {
  {0x01, "Veilleuse (GP0)"},
  {0x02, "Code (GP1)"}, 
  {0x04, "Phare (GP2)"},
  {0x08, "Clignotant (GP3)"},
  {0x00, "Tous éteints"}
};

// Séquence pour le module arrière gauche
const EtatFeu chenillardArriereGauche[] = {
  {0x08, "Veilleuse (GP0)"},        // GP0 (0x08)
  {0x04, "Clignotant (GP1)"},      // GP1 (0x04)
  {0x02, "Stop (GP2)"},           // GP2 (0x02)
  {0x01, "Klaxon (GP3)"},        // GP3 (0x01)
  {0x00, "Tous éteints"}
};

// Séquence pour le module arrière droit
const EtatFeu chenillardArriereDroit[] = {
  {0x08, "Veilleuse (GP0)"},       // GP0 (0x08)
  {0x04, "Clignotant (GP1)"},     // GP1 (0x04)
  {0x02, "Stop (GP2)"},          // GP2 (0x02)
  {0x01, "Libre (GP3)"},        // GP3 (0x01)
  {0x00, "Tous éteints"}
};

// Pointeurs vers la séquence active et gestion d'état
const EtatFeu* sequenceActive;
uint8_t nbEtats;
uint8_t etatActuel = 0;

// ----------- GESTION DU TEMPS -------------

const unsigned long INTERVALLE_CHANGEMENT = 1000; // Intervalle entre états (ms)
unsigned long dernierChangement = 0;             // Dernier changement d'état

// ------------ FONCTIONS CAN --------------

/**
 * Initialise le contrôleur CAN
 * @return true si succès, false sinon
 */
bool initialiserCAN() {
  SPI.begin();
  
  if (mcp2515.reset() != MCP2515::ERROR_OK) {
    Serial.println("[ERREUR] Échec reset MCP2515");
    return false;
  }

  if (mcp2515.setBitrate(CAN_100KBPS, MCP_16MHZ) != MCP2515::ERROR_OK) {
    Serial.println("[ERREUR] Configuration bitrate CAN");
    return false;
  }

  mcp2515.setNormalMode();
  Serial.print("[CAN] Prêt (100 kbps) - Module: ");
  Serial.println(module.nom);
  return true;
}

/**
 * Envoie une commande au module VMD et vérifie l'acquittement
 * @param registre Registre cible (GPDDR/GPLAT)
 * @param masque Bits à modifier
 * @param valeur Valeur à écrire
 * @return true si succès, false sinon
 */
bool envoyerCommande(uint8_t registre, uint8_t masque, uint8_t valeur) {
  struct can_frame trame;
  
  // Configuration de la trame CAN
  trame.can_id = module.id_im | CAN_EFF_FLAG; // Mode étendu
  trame.can_dlc = 3;                         // 3 octets de données
  trame.data[0] = registre;
  trame.data[1] = masque;
  trame.data[2] = valeur;

  // Debug: affichage des détails de la trame
  Serial.print("[ENVOI] [RXF1]: 0x");
  Serial.print(module.id_im, HEX);
  Serial.print(" | Registre: 0x");
  Serial.print(registre, HEX);
  Serial.print(" | Masque: 0x");
  Serial.print(masque, HEX);
  Serial.print(" | Valeur: 0x");
  Serial.println(valeur, HEX);

  // Purge du buffer CAN avant envoi
  struct can_frame trameRecue;
  while (mcp2515.readMessage(&trameRecue) == MCP2515::ERROR_OK);

  // Envoi de la trame
  if (mcp2515.sendMessage(&trame) != MCP2515::ERROR_OK) {
    Serial.println("[ERREUR] Échec envoi CAN");
    return false;
  }

  // Attente acquittement (200ms timeout)
  unsigned long debutAttente = millis();
  while (millis() - debutAttente < 200) {
    if (mcp2515.readMessage(&trameRecue) == MCP2515::ERROR_OK) {
      if ((trameRecue.can_id & 0x1FFFFFFF) == module.id_aim) {
        Serial.print("[RECEPTION] [TXD1] Acquittement reçu - ID: 0x");
        Serial.println(module.id_aim, HEX);
        return true;
      }
    }
  }

  Serial.println("[ERREUR] Timeout acquittement");
  return false;
}

/**
 * Configure les ports du module VMD
 * @return true si succès, false sinon
 */
bool configurerModule() {
  Serial.println("\n[CONFIG] Initialisation module...");
  
  // Configuration des ports (GP0-3 en sortie)
  if (!envoyerCommande(REG_GPDDR, 0x7F, 0xF0)) {
    Serial.println("[ERREUR] Configuration GPDDR");
    return false;
  }

  // Initialisation: toutes les sorties éteintes
  if (!envoyerCommande(REG_GPLAT, MASQUE_SORTIES, 0x00)) {
    Serial.println("[ERREUR] Initialisation GPLAT");
    return false;
  }

  Serial.println("[CONFIG] Module prêt");
  return true;
}

// ------------ SETUP ---------------

void setup() {
  Serial.begin(115200);
  while (!Serial); // Pour les cartes avec USB natif

  // Sélection de la séquence appropriée
  if (module.is_avant) {
    sequenceActive = sequenceAvant;
    nbEtats = sizeof(sequenceAvant) / sizeof(EtatFeu);
  } else {
    #if defined(MODULE_ARRIERE_GAUCHE)
      sequenceActive = chenillardArriereGauche;
      nbEtats = sizeof(chenillardArriereGauche) / sizeof(EtatFeu);
    #elif defined(MODULE_ARRIERE_DROIT)
      sequenceActive = chenillardArriereDroit;
      nbEtats = sizeof(chenillardArriereDroit) / sizeof(EtatFeu);
    #endif
  }

  // Initialisation CAN
  if (!initialiserCAN() || !configurerModule()) {
    Serial.println("\n[ARRÊT] Erreur critique - Vérifiez le câblage");
    while (1); // Blocage
  }

  Serial.println("\n[SYSTÈME] Prêt\n");
}

// -------- BOUCLE PRINCIPALE ----------

void loop() {
  // Gestion du changement d'état périodique
  if (millis() - dernierChangement >= INTERVALLE_CHANGEMENT) {
    dernierChangement = millis();

    // Passage à l'état suivant
    etatActuel = (etatActuel + 1) % nbEtats;
    const EtatFeu& etat = sequenceActive[etatActuel];

    // Affichage et envoi
    Serial.print("\n Changement état⬇️:\n ");
    Serial.println(etat.description);;
    
    
    if (!envoyerCommande(REG_GPLAT, MASQUE_SORTIES, etat.valeur)) {
      Serial.println("[ERREUR] Échec commande feux");
    }
  }
}
