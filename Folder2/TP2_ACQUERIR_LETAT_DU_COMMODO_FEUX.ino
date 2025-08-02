// =============================================================
// TP2 - Acquisition de l’état des feux via le commodo CAN
// Objectif : Lire les entrées du module (GP0 à GP7), afficher les feux actifs et l'état d commodo,
// Envoyer une trame de commande combinée (logique TOR + addition de bits).
// Buzzer ajouté : activation via GP7 (klaxon)
// =============================================================

#include <SPI.h>
#include <mcp2515.h>

//========== CONFIGURATION MATÉRIELLE / CAN ==========

const int SPI_CS_PIN = 9;
const int INT_PIN    = 2;
const int BUZZER_PIN = 3; // Buzzer connecté sur D3

MCP2515 mcp2515(SPI_CS_PIN);

#define CAN_SPEED   CAN_100KBPS
#define CAN_CLOCK   MCP_16MHZ

//==========IDENTIFIANTS CAN (format étendu)==========

#define ID_IM_COMMODO   0x05081F00
#define ID_AIM_COMMODO  0x05200000
#define ID_OM_COMMODO   0x05400000

//========== REGISTRES MCP25050 ==========

#define REG_GPDDR   0x1F
#define REG_GPLAT   0x1E

//========== MASQUES GP0 à GP7 ==========

#define MASK_VEILLEUSE  (1 << 0)
#define MASK_WARNING    (1 << 1)
#define MASK_PHARE      (1 << 2)
#define MASK_CODE       (1 << 3)
#define MASK_CLIGN_G    (1 << 4)
#define MASK_CLIGN_D    (1 << 5)
#define MASK_STOP       (1 << 6)
#define MASK_KLAXON     (1 << 7)

//========== VARIABLES GLOBALES ==========

volatile bool messageRecu = false;

uint8_t dernierEtatBoutons = 0;
bool ackPending  = false;
bool ackPrinted  = false;

// =================== INITIALISATION CAN =====================

void initialiserCAN() {
  SPI.begin();
  while (mcp2515.reset() != MCP2515::ERROR_OK);
  while (mcp2515.setBitrate(CAN_SPEED, CAN_CLOCK) != MCP2515::ERROR_OK);
  while (mcp2515.setNormalMode() != MCP2515::ERROR_OK);
}

void configurerModuleCommodoEnEntree() {
  struct can_frame trame;
  trame.can_id  = ID_IM_COMMODO | CAN_EFF_FLAG;
  trame.can_dlc = 3;
  trame.data[0] = REG_GPDDR;
  trame.data[1] = 0xFF;
  trame.data[2] = 0xFF;
  mcp2515.sendMessage(&trame);
}

// =================== TRAITEMENT CAN =========================

void interruptionCAN() {
  messageRecu = true;
}

void traiterTrameCAN(struct can_frame trame) {
  uint32_t id = trame.can_id & 0x1FFFFFFF;

  // ---- Trame OM
  if (id == ID_OM_COMMODO && trame.can_dlc >= 2) {
    uint8_t etatBrut = trame.data[1];
    uint8_t boutons = ~etatBrut;

    if (boutons != dernierEtatBoutons) {
      dernierEtatBoutons = boutons;
      envoyerCommandeFeux(boutons);
      afficherEtatFeux(boutons);

      // Gestion du buzzer selon GP7
      if (boutons & MASK_KLAXON) {
        digitalWrite(BUZZER_PIN, HIGH); // Active le buzzer
      } else {
        digitalWrite(BUZZER_PIN, LOW);  // Désactive le buzzer
      }
    }
  }

  // ---- Trame AIM (ACK)
  else if (id == ID_AIM_COMMODO && ackPending && !ackPrinted) {
    Serial.print("[RX] AIM reçu : 0x");
    Serial.print(trame.data[2], HEX);
    Serial.print(" (0b"); Serial.print(trame.data[2], BIN); Serial.println(")\n");
    Serial.println("--------------------------------------------------");
    ackPending = false;
    ackPrinted = true;
  }
}

void envoyerCommandeFeux(uint8_t valeur) {
  struct can_frame trame;
  trame.can_id  = ID_IM_COMMODO | CAN_EFF_FLAG;
  trame.can_dlc = 3;
  trame.data[0] = REG_GPLAT;
  trame.data[1] = 0xFF;
  trame.data[2] = valeur;

  mcp2515.sendMessage(&trame);

  Serial.print("[TX] Feux activés : 0x"); Serial.print(valeur, HEX);
  Serial.print(" (0b"); Serial.print(valeur, BIN); Serial.println(")");
  ackPending = true;
  ackPrinted = false;
}

// ================= AFFICHAGE SERIAL =========================

void afficherEtatFeux(uint8_t boutons) {
  Serial.println("======= ÉTAT DES FEUX =======");
  Serial.print("Clignotant Gauche: "); Serial.println(boutons & MASK_CLIGN_G ? "ACTIF" : "inactif");
  Serial.print("Clignotant Droit : "); Serial.println(boutons & MASK_CLIGN_D ? "ACTIF" : "inactif");
  Serial.print("Stop             : "); Serial.println(boutons & MASK_STOP    ? "ACTIF" : "inactif");
  Serial.print("Klaxon           : "); Serial.println(boutons & MASK_KLAXON  ? "ACTIF" : "inactif");
  Serial.print("Veilleuse        : "); Serial.println(boutons & MASK_VEILLEUSE ? "ACTIF" : "inactif");
  Serial.print("Phare            : "); Serial.println(boutons & MASK_PHARE     ? "ACTIF" : "inactif");
  Serial.print("Code             : "); Serial.println(boutons & MASK_CODE      ? "ACTIF" : "inactif");
  Serial.print("Warning          : "); Serial.println(boutons & MASK_WARNING   ? "ACTIF" : "inactif");
  Serial.println("=============================\n");
}

// ===================== SETUP & LOOP =========================

void setup() {
  Serial.begin(115200);
  while (!Serial);

  pinMode(INT_PIN, INPUT);
  attachInterrupt(digitalPinToInterrupt(INT_PIN), interruptionCAN, FALLING);

  pinMode(BUZZER_PIN, OUTPUT);      // Configure la broche du buzzer
  digitalWrite(BUZZER_PIN, LOW);    // Désactive au démarrage

  initialiserCAN();
  configurerModuleCommodoEnEntree();

  Serial.println("\n[SYSTÈME PRÊT - Interruption active]");
}

void loop() {
  if (messageRecu) {
    messageRecu = false;

    struct can_frame trame;
    while (mcp2515.readMessage(&trame) == MCP2515::ERROR_OK) {
      traiterTrameCAN(trame);
    }
  }
}
