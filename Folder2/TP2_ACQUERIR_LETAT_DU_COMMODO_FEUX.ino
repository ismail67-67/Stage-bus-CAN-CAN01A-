// ============================================================
// TP2 - ACQUISITION ETAT COMMODO FEUX VIA CAN BUS
// Version finale optimisée avec logs clairs et détection réactive
// ============================================================

#include <SPI.h>
#include <mcp2515.h>

// -------------------- CONFIGURATION ------------------------
const int SPI_CS_PIN = 9;
MCP2515 mcp2515(SPI_CS_PIN);

#define CAN_SPEED CAN_100KBPS
#define CAN_CLOCK MCP_16MHZ

#define ID_IM_COMMODO   0x05081F00
#define ID_AIM_COMMODO  0x05200000
#define ID_IRM_COMMODO  0x05041E07
#define ID_OM_COMMODO   0x05400000

#define REG_GPDDR  0x1F
#define REG_GPLAT  0x1E
#define REG_IOTEN  0x1C

#define MASK_CLIGN_G  (1 << 4)
#define MASK_CLIGN_D  (1 << 5)
#define MASK_STOP     (1 << 6)
#define MASK_KLAXON   (1 << 7)
#define MASK_D8       (1 << 1)

#define LED_CLIGN_G 11
#define LED_CLIGN_D 12
#define LED_STOP    13
#define LED_KLAXON  14

#define POLLING_INTERVAL  50
#define BLINK_INTERVAL    500

// -------------------- STRUCTURE ET ETAT --------------------
typedef struct {
  bool clignGauche;
  bool clignDroit;
  bool stop;
  bool klaxon;
  bool warning;
  bool ledD8;
  unsigned long dernierPoll;
  unsigned long dernierClignotement;

  bool prevClignGauche;
  bool prevClignDroit;
  bool prevStop;
  bool prevKlaxon;

  bool ackPrinted;
} CommodoState;

CommodoState etat = {false, false, false, false, false, false, 0, 0, false, false, false, false, false};
bool etatClignotement = false;
bool changementEtat = false;

// -------------------- AFFICHAGE TRAMES ---------------------
void afficherTrameTX(const char* label, uint8_t reg, uint8_t mask, uint8_t val) {
  Serial.print("[TX] "); Serial.print(label);
  Serial.print(" | Registre: 0x"); Serial.print(reg, HEX);
  Serial.print(" | Masque: 0x"); Serial.print(mask, HEX);
  Serial.print(" | Valeur: 0x"); Serial.print(val, HEX);
  Serial.print(" (0b"); Serial.print(val, BIN); Serial.println(")");
}

void afficherTrameRX_ACK(uint32_t id, uint8_t reg, uint8_t mask, uint8_t val) {
  if (!etat.ackPrinted) {
    Serial.print("[RX] ACK reçu depuis module 0x"); Serial.print(id, HEX);
    Serial.print(" | Registre: 0x"); Serial.print(reg, HEX);
    Serial.print(" | Masque: 0x"); Serial.print(mask, HEX);
    Serial.print(" | Valeur: 0x"); Serial.print(val, HEX);
    Serial.print(" (0b"); Serial.print(val, BIN); Serial.println(")");
    etat.ackPrinted = true;
  }
}

// ---------------------- CAN SETUP --------------------------
void initCAN() {
  SPI.begin();
  if (mcp2515.reset() != MCP2515::ERROR_OK) while (1);
  if (mcp2515.setBitrate(CAN_SPEED, CAN_CLOCK) != MCP2515::ERROR_OK) while (1);
  if (mcp2515.setNormalMode() != MCP2515::ERROR_OK) while (1);
}

void configurerModule() {
  struct can_frame trame;

  trame.can_id = ID_IM_COMMODO | CAN_EFF_FLAG;
  trame.can_dlc = 3;
  trame.data[0] = REG_GPDDR;
  trame.data[1] = 0xFF;
  trame.data[2] = 0xFF & ~MASK_D8;
  mcp2515.sendMessage(&trame);

  trame.data[0] = REG_GPLAT;
  trame.data[1] = MASK_D8;
  trame.data[2] = MASK_D8;
  mcp2515.sendMessage(&trame);
}

// --------------------- LOGIQUE CAN -------------------------
void envoyerCommande(uint8_t mask, bool actif, const char* nom) {
  struct can_frame trame;
  trame.can_id = ID_IM_COMMODO | CAN_EFF_FLAG;
  trame.can_dlc = 3;
  trame.data[0] = REG_GPLAT;
  trame.data[1] = mask;
  trame.data[2] = actif ? mask : 0x00;
  mcp2515.sendMessage(&trame);
  afficherTrameTX(nom, REG_GPLAT, mask, trame.data[2]);
  etat.ackPrinted = false;
}

void demanderEtat() {
  struct can_frame trame;
  trame.can_id = ID_IRM_COMMODO | CAN_EFF_FLAG;
  trame.can_dlc = 1;
  trame.data[0] = REG_GPLAT;
  mcp2515.sendMessage(&trame);
  etat.dernierPoll = millis();
}

bool traiterMessageCAN(can_frame trame) {
  if ((trame.can_id & 0x1FFFFFFF) == ID_OM_COMMODO && trame.can_dlc >= 2) {
    bool b1 = !(trame.data[1] & MASK_CLIGN_G);
    bool b2 = !(trame.data[1] & MASK_CLIGN_D);
    bool b3 = !(trame.data[1] & MASK_STOP);
    bool b4 = !(trame.data[1] & MASK_KLAXON);

    etat.warning = b1 && b2;

    if (b1 != etat.prevClignGauche) {
      envoyerCommande(MASK_CLIGN_G, b1, "CLIGNOTANT G");
      etat.prevClignGauche = b1;
      changementEtat = true;
    }
    if (b2 != etat.prevClignDroit) {
      envoyerCommande(MASK_CLIGN_D, b2, "CLIGNOTANT D");
      etat.prevClignDroit = b2;
      changementEtat = true;
    }
    if (b3 != etat.prevStop) {
      envoyerCommande(MASK_STOP, b3, "STOP");
      etat.prevStop = b3;
      changementEtat = true;
    }
    if (b4 != etat.prevKlaxon) {
      envoyerCommande(MASK_KLAXON, b4, "KLAXON");
      etat.prevKlaxon = b4;
      changementEtat = true;
    }

    etat.clignGauche = b1;
    etat.clignDroit  = b2;
    etat.stop        = b3;
    etat.klaxon      = b4;

    return true;
  }
  else if ((trame.can_id & 0x1FFFFFFF) == ID_AIM_COMMODO) {
    afficherTrameRX_ACK(ID_AIM_COMMODO, REG_GPLAT, 0xFF, trame.data[2]);
    return true;
  }
  return false;
}

// ------------------- LOGIQUE LED ET DEBUG ------------------
void actualiserLEDs() {
  if (millis() - etat.dernierClignotement > BLINK_INTERVAL) {
    etatClignotement = !etatClignotement;
    etat.dernierClignotement = millis();
  }

  digitalWrite(LED_CLIGN_G, etat.clignGauche ? etatClignotement : LOW);
  digitalWrite(LED_CLIGN_D, etat.clignDroit  ? etatClignotement : LOW);
  digitalWrite(LED_STOP,    etat.stop        ? HIGH : LOW);
  digitalWrite(LED_KLAXON,  etat.klaxon      ? HIGH : LOW);
}

void afficherEtat() {
  Serial.println("\n==== ETAT FEUX COMMODO ====");
  Serial.print("Clignotant Gauche : "); Serial.println(etat.clignGauche ? "ACTIF" : "inactif");
  Serial.print("Clignotant Droit   : "); Serial.println(etat.clignDroit ? "ACTIF" : "inactif");
  Serial.print("Feu Stop           : "); Serial.println(etat.stop ? "ACTIF" : "inactif");
  Serial.print("Klaxon             : "); Serial.println(etat.klaxon ? "ACTIF" : "inactif");
  Serial.print("Warning (Détresse) : "); Serial.println(etat.warning ? "ACTIF" : "inactif");
  Serial.println("===========================");
}

// ------------------------ SETUP / LOOP ----------------------
void setup() {
  pinMode(LED_CLIGN_G, OUTPUT);
  pinMode(LED_CLIGN_D, OUTPUT);
  pinMode(LED_STOP, OUTPUT);
  pinMode(LED_KLAXON, OUTPUT);

  Serial.begin(115200);
  while (!Serial);

  initCAN();
  configurerModule();
  demanderEtat();
  Serial.println("\n[SYSTEME PRÊT]");
}

void loop() {
  struct can_frame trame;

  if (mcp2515.readMessage(&trame) == MCP2515::ERROR_OK) {
    if (traiterMessageCAN(trame) && changementEtat) {
      afficherEtat();
      changementEtat = false;
    }
  }

  if (millis() - etat.dernierPoll >= POLLING_INTERVAL) {
    demanderEtat();
  }

  actualiserLEDs();
}
