/**************************************************************************
 * TP1 - Chenillard VMD (Arduino Uno R3 + MCP2515 + Interruption)
 * ------------------------------------------------------------------------
 * Objectif : Commander cycliquement les feux via le bus CAN avec détection
 *            d’acquittement (AIM) par interruption matérielle.
 * Module cible : Feux avant droit, gauche, arrière droit ou gauche.
 * Matériel :
 *   - Arduino Uno R3
 *   - CAN-BUS Shield SeeedStudio (MCP2515 + TJA1050)
 *   - Bloc optique VMD (répond via trame AIM)
 **************************************************************************/

#include <SPI.h>
#include "mcp2515.h"

// ------------------------- CONFIGURATION MATÉRIELLE -------------------------

const int SPI_CS_PIN = 9;      // Broche CS pour MCP2515
const int INT_PIN     = 2;     // Broche d'interruption (INT) du MCP2515

MCP2515 mcp2515(SPI_CS_PIN);   // Instance du contrôleur CAN
volatile bool messageRecu = false;  // Flag mis à jour par interruption

// ------------------------ SÉLECTION DU MODULE À CONTRÔLER -------------------

// Décommenter UN SEUL module :
#define MODULE_AVANT_DROIT
//#define MODULE_AVANT_GAUCHE
//#define MODULE_ARRIERE_DROIT
//#define MODULE_ARRIERE_GAUCHE

struct ModuleConfig {
  const char* nom;        // Nom lisible du module
  uint32_t id_im;         // ID Input Message (commande)
  uint32_t id_aim;        // ID Acquittement du module
  bool is_avant;          // true = avant / false = arrière
};

const ModuleConfig modules[] = {
  {"Feux avant droit",   0x0E880000, 0x0EA00000, true},
  {"Feux avant gauche",  0x0E080000, 0x0E200000, true},
  {"Feux arrière droit", 0x0F880000, 0x0FA00000, false},
  {"Feux arrière gauche",0x0F080000, 0x0F200000, false}
};

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

const ModuleConfig& module = modules[MODULE_INDEX];

// ------------------------- REGISTRES & CONSTANTES CAN -----------------------

#define REG_GPDDR 0x1F        // Configuration direction des ports (GP0-7)
#define REG_GPLAT 0x1E        // État des sorties (LEDs)
#define MASQUE_SORTIES 0x0F   // GP0 à GP3 (feux)

// ------------------------- SÉQUENCES DES ÉTATS DES FEUX ---------------------

struct EtatFeu {
  uint8_t valeur;
  const char* description;
};

// Feux avant
const EtatFeu sequenceAvant[] = {
  {0x01, "Veilleuse (GP0)"},
  {0x02, "Code (GP1)"},
  {0x04, "Phare (GP2)"},
  {0x08, "Clignotant (GP3)"},
  {0x00, "Tous éteints"}
};

// Feux arrière gauche
const EtatFeu chenillardArriereGauche[] = {
  {0x08, "Veilleuse (GP0)"},
  {0x04, "Clignotant (GP1)"},
  {0x02, "Stop (GP2)"},
  {0x01, "Klaxon (GP3)"},
  {0x00, "Tous éteints"}
};

// Feux arrière droit
const EtatFeu chenillardArriereDroit[] = {
  {0x08, "Veilleuse (GP0)"},
  {0x04, "Clignotant (GP1)"},
  {0x02, "Stop (GP2)"},
  {0x01, "Libre (GP3)"},
  {0x00, "Tous éteints"}
};

const EtatFeu* sequenceActive;
uint8_t nbEtats;
uint8_t etatActuel = 0;

// ----------------------------- PARAMÈTRES TEMPORELS -------------------------

const unsigned long INTERVALLE_CHANGEMENT = 1000; // Durée entre chaque feu
unsigned long dernierChangement = 0;

// ======================== INITIALISATION CAN ================================

bool initialiserCAN() {
  SPI.begin();

  if (mcp2515.reset() != MCP2515::ERROR_OK) {
    Serial.println("[ERREUR] Échec reset MCP2515");
    return false;
  }

  if (mcp2515.setBitrate(CAN_100KBPS, MCP_16MHZ) != MCP2515::ERROR_OK) {
    Serial.println("[ERREUR] Configuration bitrate");
    return false;
  }

  mcp2515.setNormalMode();
  Serial.print("[CAN] Prêt (100 kbps) - Module : ");
  Serial.println(module.nom);
  return true;
}


// ======================== ENVOI COMMANDE CAN (IM) ===========================

bool envoyerCommande(uint8_t registre, uint8_t masque, uint8_t valeur) {
  struct can_frame trame;

  trame.can_id = module.id_im | CAN_EFF_FLAG;
  trame.can_dlc = 3;
  trame.data[0] = registre;
  trame.data[1] = masque;
  trame.data[2] = valeur;

  Serial.print("[ENVOI] [RXF1] ID: 0x");
  Serial.print(module.id_im, HEX);
  Serial.print(" | Registre: 0x");
  Serial.print(registre, HEX);
  Serial.print(" | Masque: 0x");
  Serial.print(masque, HEX);
  Serial.print(" | Valeur: 0x");
  Serial.println(valeur, HEX);

  return (mcp2515.sendMessage(&trame) == MCP2515::ERROR_OK);
}

// ======================== CONFIGURATION DU MODULE VMD =======================

bool configurerModule() {
  Serial.println("\n[CONFIG] Initialisation module VMD...");

  if (!envoyerCommande(REG_GPDDR, 0x7F, 0xF0)) {
    Serial.println("[ERREUR] Configuration direction GPDDR");
    return false;
  }

  if (!envoyerCommande(REG_GPLAT, MASQUE_SORTIES, 0x00)) {
    Serial.println("[ERREUR] Initialisation GPLAT");
    return false;
  }

  Serial.println("[CONFIG] Module prêt\n");
  return true;
}

// ======================== INTERRUPTION SUR TRAME CAN =======================

void interruptionCAN() {
  messageRecu = true;
}

// =============================== SETUP =====================================

void setup() {
  Serial.begin(115200);
  while (!Serial);

  pinMode(INT_PIN, INPUT);
  attachInterrupt(digitalPinToInterrupt(INT_PIN), interruptionCAN, FALLING);

  // Sélection de la séquence de feux
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

  if (!initialiserCAN() || !configurerModule()) {
    Serial.println("[ARRÊT] Erreur critique - Vérifiez câblage !");
    while (1);
  }

  Serial.println("[SYSTÈME] Prêt\n");
}

// ============================== LOOP PRINCIPALE ============================

void loop() {
  // Gestion cyclique des feux
  if (millis() - dernierChangement >= INTERVALLE_CHANGEMENT) {
    dernierChangement = millis();

    etatActuel = (etatActuel + 1) % nbEtats;
    const EtatFeu& etat = sequenceActive[etatActuel];

    Serial.print("\n ⬇️ Changement d’état : ");
    Serial.println(etat.description);

    if (!envoyerCommande(REG_GPLAT, MASQUE_SORTIES, etat.valeur)) {
      Serial.println("[ERREUR] Envoi de commande CAN");
    }
  }

  // Traitement des messages reçus (interruption)
  if (messageRecu) {
    messageRecu = false;

    struct can_frame trame;
    while (mcp2515.readMessage(&trame) == MCP2515::ERROR_OK) {
      uint32_t id_recue = trame.can_id & 0x1FFFFFFF;

      if (id_recue == module.id_aim) {
        Serial.print("[RECEPTION] ✅ AIM reçu - ID: 0x");
        Serial.println(module.id_aim, HEX);
      } else {
        Serial.print("[RECEPTION] 📨 Trame inconnue - ID: 0x");
        Serial.println(id_recue, HEX);
      }
    }
  }
}
