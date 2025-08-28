/**************************************************************************
 * TP1 - Chenillard VMD 
 * ------------------------------------------------------------------------
 * Objectif :
 *   - Commander cycliquement les feux via le bus CAN.
 *   - Vérifier la bonne réception avec l’acquittement (AIM).
 *   - Utiliser une interruption matérielle.
 *
 * Matériel utilisé :
 *   - Arduino Uno R3
 *   - CAN-BUS Shield SeeedStudio 
 *   - Bloc optique du VMD (module CAN01A)
 **************************************************************************/

#include <SPI.h>        // Bibliothèque SPI (nécessaire pour dialoguer avec MCP2515)
#include "mcp2515.h"    // Bibliothèque spécifique pour contrôler le MCP2515

// ------------------------- CONFIGURATION MATÉRIELLE -------------------------

const int SPI_CS_PIN = 9;      // Broche CS (Chip Select) pour sélectionner le MCP2515
const int INT_PIN     = 2;     // Broche d'interruption (INT) reliée au MCP2515

MCP2515 mcp2515(SPI_CS_PIN);   // Création d’une instance pour contrôler le MCP2515
volatile bool messageRecu = false;  // Flag (variable modifiée par interruption)

// ------------------------ SÉLECTION DU MODULE À CONTRÔLER -------------------
// Ici on choisit quel bloc optique on veut commander.
// -> On décommente un seul define à la fois.

#define MODULE_AVANT_DROIT
//#define MODULE_AVANT_GAUCHE
//#define MODULE_ARRIERE_DROIT
//#define MODULE_ARRIERE_GAUCHE

// Structure regroupant les informations d’un module VMD
struct ModuleConfig {
  const char* nom;        // Nom lisible du module (affichage console)
  uint32_t id_im;         // Identifiant CAN pour envoyer une commande (IM = Input Message)
  uint32_t id_aim;        // Identifiant CAN pour l’acquittement (AIM = Acknowledge Input Message)
  bool is_avant;          // true = module avant / false = module arrière
};

// Liste de configuration pour chaque module (IDs CAN spécifiques au VMD)
const ModuleConfig modules[] = {
  {"Feux avant droit",   0x0E880000, 0x0EA00000, true},
  {"Feux avant gauche",  0x0E080000, 0x0E200000, true},
  {"Feux arrière droit", 0x0F880000, 0x0FA00000, false},
  {"Feux arrière gauche",0x0F080000, 0x0F200000, false}
};

// Sélection de l’index en fonction du module activé
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

// Référence vers le module choisi
const ModuleConfig& module = modules[MODULE_INDEX];

// ------------------------- REGISTRES & CONSTANTES CAN -----------------------
// Le MCP25050 (dans le bloc optique) a des registres internes pour piloter les sorties.

#define REG_GPDDR 0x1F        // Registre de direction (0 = entrée, 1 = sortie)
#define REG_GPLAT 0x1E        // Registre qui commande l’état des sorties
#define MASQUE_SORTIES 0x0F   // Masque utilisé : GP0 à GP3 (4 bits = 4 feux)

// ------------------------- SÉQUENCES DES ÉTATS DES FEUX ---------------------
// Chaque état correspond à un allumage particulier d’un feu.

struct EtatFeu {
  uint8_t valeur;        // Valeur binaire envoyée au registre GPLAT
  const char* description; // Texte affiché pour savoir quel feu est actif
};

// Séquence chenillard avant (4 feux : veilleuse, code, phare, clignotant)
const EtatFeu sequenceAvant[] = {
  {0x01, "Veilleuse (GP0)"},
  {0x02, "Code (GP1)"},
  {0x04, "Phare (GP2)"},
  {0x08, "Clignotant (GP3)"},
  {0x00, "Tous éteints"}
};

// Séquence chenillard arrière gauche
const EtatFeu chenillardArriereGauche[] = {
  {0x08, "Veilleuse (GP0)"},
  {0x04, "Clignotant (GP1)"},
  {0x02, "Stop (GP2)"},
  {0x01, "Klaxon (GP3)"},
  {0x00, "Tous éteints"}
};

// Séquence chenillard arrière droit
const EtatFeu chenillardArriereDroit[] = {
  {0x08, "Veilleuse (GP0)"},
  {0x04, "Clignotant (GP1)"},
  {0x02, "Stop (GP2)"},
  {0x01, "Libre (GP3)"},
  {0x00, "Tous éteints"}
};

// Pointeur vers la séquence active (avant / arrière)
const EtatFeu* sequenceActive;
uint8_t nbEtats;        // Nombre d’états dans la séquence
uint8_t etatActuel = 0; // Index de l’état courant

// ----------------------------- PARAMÈTRES TEMPORELS -------------------------

const unsigned long INTERVALLE_CHANGEMENT = 1000; // Durée (ms) entre chaque feu
unsigned long dernierChangement = 0;              // Mémorise le temps du dernier changement

// ======================== INITIALISATION CAN ================================
// Fonction pour initialiser le bus CAN via le MCP2515

bool initialiserCAN() {
  SPI.begin();  // Démarrage de la communication SPI

  // Reset du MCP2515
  if (mcp2515.reset() != MCP2515::ERROR_OK) {
    Serial.println("[ERREUR] Échec reset MCP2515");
    return false;
  }

  // Configuration du débit (ici 100 kbps, quartz 16 MHz)
  if (mcp2515.setBitrate(CAN_100KBPS, MCP_16MHZ) != MCP2515::ERROR_OK) {
    Serial.println("[ERREUR] Configuration bitrate");
    return false;
  }

  // Passage en mode normal (permet émission + réception)
  mcp2515.setNormalMode();

  Serial.print("[CAN] Prêt (100 kbps) - Module : ");
  Serial.println(module.nom);
  return true;
}

// ======================== ENVOI COMMANDE CAN (IM) ===========================
// Permet d’envoyer une commande au module VMD (trame CAN étendue)

bool envoyerCommande(uint8_t registre, uint8_t masque, uint8_t valeur) {
  struct can_frame trame;

  // Identifiant CAN = ID du module (IM) + drapeau EFF (Extended Frame Format)
  trame.can_id = module.id_im | CAN_EFF_FLAG;
  trame.can_dlc = 3;        // 3 octets de données (registre, masque, valeur)
  trame.data[0] = registre; // Quel registre écrire
  trame.data[1] = masque;   // Quels bits affecter
  trame.data[2] = valeur;   // Nouvelle valeur

  // Affichage console pour suivi
  Serial.print("[ENVOI] [RXF1] ID: 0x");
  Serial.print(module.id_im, HEX);
  Serial.print(" | Registre: 0x");
  Serial.print(registre, HEX);
  Serial.print(" | Masque: 0x");
  Serial.print(masque, HEX);
  Serial.print(" | Valeur: 0x");
  Serial.println(valeur, HEX);

  // Envoi sur le bus CAN
  return (mcp2515.sendMessage(&trame) == MCP2515::ERROR_OK);
}

// ======================== CONFIGURATION DU MODULE VMD =======================
// Met en place le module cible (direction des ports, initialisation sorties)

bool configurerModule() {
  Serial.println("\n[CONFIG] Initialisation module VMD...");

  // GPDDR = définit GP0-GP3 en sortie (0xF0 = GP4-7 entrée, GP0-3 sortie)
  if (!envoyerCommande(REG_GPDDR, 0x7F, 0xF0)) {
    Serial.println("[ERREUR] Configuration direction GPDDR");
    return false;
  }

  // GPLAT = met toutes les sorties à 0 (tous feux éteints)
  if (!envoyerCommande(REG_GPLAT, MASQUE_SORTIES, 0x00)) {
    Serial.println("[ERREUR] Initialisation GPLAT");
    return false;
  }

  Serial.println("[CONFIG] Module prêt\n");
  return true;
}

// ======================== INTERRUPTION SUR TRAME CAN =======================
// Fonction appelée automatiquement quand le MCP2515 génère une interruption

void interruptionCAN() {
  messageRecu = true; // On met le flag à vrai (traité dans loop)
}

// =============================== SETUP =====================================

void setup() {
  Serial.begin(115200);   // Initialisation console série
  while (!Serial);        // Attente de la connexion série

  // Configuration de la broche INT
  pinMode(INT_PIN, INPUT);
  attachInterrupt(digitalPinToInterrupt(INT_PIN), interruptionCAN, FALLING);

  // Sélection de la séquence de feux en fonction du module choisi
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

  // Initialisation du bus CAN et du module VMD
  if (!initialiserCAN() || !configurerModule()) {
    Serial.println("[ARRÊT] Erreur critique - Vérifiez câblage !");
    while (1); // Bloque le programme en cas d’échec
  }

  Serial.println("[SYSTÈME] Prêt\n");
}

// ============================== LOOP PRINCIPALE ============================

void loop() {
  // --- Partie 1 : Chenillard cyclique ---
  if (millis() - dernierChangement >= INTERVALLE_CHANGEMENT) {
    dernierChangement = millis();

    // Passage à l’état suivant
    etatActuel = (etatActuel + 1) % nbEtats;
    const EtatFeu& etat = sequenceActive[etatActuel];

    Serial.print("\n ⬇️ Changement d’état : ");
    Serial.println(etat.description);

    // Envoi de la commande CAN au module
    if (!envoyerCommande(REG_GPLAT, MASQUE_SORTIES, etat.valeur)) {
      Serial.println("[ERREUR] Envoi de commande CAN");
    }
  }

  // --- Partie 2 : Traitement des trames reçues ---
  if (messageRecu) { // Vérifie si une interruption a signalé un message
    messageRecu = false;

    struct can_frame trame;
    // Lecture des trames reçues tant qu’il y en a
    while (mcp2515.readMessage(&trame) == MCP2515::ERROR_OK) {
      uint32_t id_recue = trame.can_id & 0x1FFFFFFF; // ID reçu (maské)

      if (id_recue == module.id_aim) {
        // Acquittement attendu (AIM)
        Serial.print("[RECEPTION] ✅ AIM reçu - ID: 0x");
        Serial.println(module.id_aim, HEX);
      } else {
        // Trame qui ne correspond pas à notre module
        Serial.print("[RECEPTION] 📨 Trame inconnue - ID: 0x");
        Serial.println(id_recue, HEX);
      }
    }
  }
}
