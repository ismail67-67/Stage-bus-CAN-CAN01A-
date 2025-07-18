// ============================================================================
// TP3 - Contrôle des feux avant droit (FVD) via CAN
// Arduino Uno + MCP2515 - Véhicule Multiplexé Didactique
// ============================================================================
#include <SPI.h>
#include <mcp2515.h>

const int SPI_CS_PIN = 9;
MCP2515 mcp2515(SPI_CS_PIN);

#define CAN_SPEED CAN_100KBPS
#define CAN_CLOCK MCP_16MHZ

// Identifiants CAN
#define ID_IM_FVD   0x0E880000  // Input Message Feux Avant Droit
#define ID_AIM_FVD  0x0EA00000  // Acquittement IM Feux Avant Droit

#define REG_GPDDR  0x1F         // Registre direction GPIO
#define REG_GPLAT  0x1E         // Registre état GPIO
#define MASK_FVD   0x0F         // Masque pour les 4 bits LSB

#define TEMPO_FEUX 3500         // 3.5s pour cycle feux
#define TEMPO_CLIGNOT 1600      // 1.6s pour clignotant

// Structure pour l'état des feux FVD
struct {
  uint8_t veilleuse : 1;
  uint8_t code      : 1;
  uint8_t phare     : 1;
  uint8_t clign_d   : 1;
} commandes;

unsigned long dernier_changement = 0;
unsigned long dernier_clignotement = 0;
bool clignotant_actif = false;

// ==============================================
// FONCTION : Afficher l'état des feux
// ==============================================
void printFeuxStatus() {
  Serial.println("\n[ETAT DES FEUX - FVD]");
  Serial.print("  Veilleuse : "); Serial.println(commandes.veilleuse ? "ON" : "OFF");
  Serial.print("  Code      : "); Serial.println(commandes.code ? "ON" : "OFF");
  Serial.print("  Phare     : "); Serial.println(commandes.phare ? "ON" : "OFF");
  Serial.print("  Cligno D  : "); Serial.println(commandes.clign_d ? "ON" : "OFF");
  
  uint8_t val = (commandes.clign_d << 3) | (commandes.phare << 2) |
                (commandes.code << 1) | commandes.veilleuse;
  Serial.print("  Valeur binaire : 0b");
  for (int8_t i = 3; i >= 0; i--) Serial.print((val >> i) & 1);
  Serial.print(" (0x"); Serial.print(val, HEX); Serial.println(")");
  Serial.println("----------------------------------");
}

// ==============================================
// FONCTION : Initialisation CAN
// ==============================================
bool initCAN() {
  if (mcp2515.reset() != MCP2515::ERROR_OK) {
    Serial.println("[ERREUR] Reset CAN failed");
    return false;
  }
  if (mcp2515.setBitrate(CAN_SPEED, CAN_CLOCK) != MCP2515::ERROR_OK) {
    Serial.println("[ERREUR] Configuration bitrate");
    return false;
  }
  if (mcp2515.setNormalMode() != MCP2515::ERROR_OK) {
    Serial.println("[ERREUR] Mode normal");
    return false;
  }
  return true;
}

// ==============================================
// FONCTION : Envoi avec acquittement
// ==============================================
bool sendWithAck(uint32_t id, uint32_t ack_id, uint8_t reg, uint8_t mask, uint8_t value) {
  struct can_frame frame;
  frame.can_id  = id | CAN_EFF_FLAG;  // Mode étendu
  frame.can_dlc = 3;                  // 3 octets de données
  frame.data[0] = reg;                // Registre cible
  frame.data[1] = mask;               // Masque
  frame.data[2] = value;              // Valeur

  // Affichage détaillé de la trame envoyée
  Serial.println("\n[ENVOI TRAME]");
  Serial.print("  [RX]ID: 0x"); Serial.println(id, HEX);
  Serial.print("  Registre: 0x"); Serial.println(reg, HEX);
  Serial.print("  Masque: 0b"); 
  for (int8_t i = 7; i >= 0; i--) Serial.print((mask >> i) & 1);
  Serial.print(" (0x"); Serial.print(mask, HEX); Serial.println(")");
  Serial.print("  Valeur: 0b"); 
  for (int8_t i = 7; i >= 0; i--) Serial.print((value >> i) & 1);
  Serial.print(" (0x"); Serial.print(value, HEX); Serial.println(")");

  if (mcp2515.sendMessage(&frame) != MCP2515::ERROR_OK) {
    Serial.println("[ERREUR] Envoi trame");
    return false;
  }

  // Attente acquittement
  struct can_frame ack_frame;
  unsigned long t0 = millis();
  while (millis() - t0 < 500) {  // Timeout 500ms
    if (mcp2515.readMessage(&ack_frame) == MCP2515::ERROR_OK) {
      if ((ack_frame.can_id & 0x1FFFFFFF) == ack_id) {
        Serial.println("\n[ACQUITTEMENT RECU]");
        Serial.print(" [TX]ID: 0x"); Serial.println(ack_id, HEX);
        Serial.print("  Module: Feux Avant Droit");
        Serial.println("\n----------------------------------");
        return true;
      }
    }
  }
  Serial.println("[ERREUR] Timeout acquittement");
  return false;
}

// ==============================================
// FONCTION : Mise à jour cycle feux
// ==============================================
void updateFeux() {
  static uint8_t cycle = 0;

  Serial.print("\n[CYCLE FEUX] Phase "); Serial.println(cycle);
  
  // Séquence: OFF -> Veilleuse -> Code -> Phare
  switch (cycle) {
    case 0:  // Tout éteint
      commandes.veilleuse = 0; 
      commandes.code = 0; 
      commandes.phare = 0; 
      break;
    case 1:  // Veilleuse
      commandes.veilleuse = 1; 
      break;
    case 2:  // Code
      commandes.code = 1; 
      break;
    case 3:  // Phare
      commandes.code = 0; 
      commandes.phare = 1; 
      break;
  }
  cycle = (cycle + 1) % 4;

  // Construction valeur et envoi
  uint8_t val_fvd = (commandes.clign_d << 3) | (commandes.phare << 2) |
                    (commandes.code << 1) | commandes.veilleuse;
  sendWithAck(ID_IM_FVD, ID_AIM_FVD, REG_GPLAT, MASK_FVD, val_fvd);
  printFeuxStatus();
}

// ==============================================
// SETUP
// ==============================================
void setup() {
  Serial.begin(115200);
  while (!Serial); // Attente connexion série
  SPI.begin();
  
  Serial.println("\nInitialisation CAN...");
  if (!initCAN()) {
    Serial.println("[ERREUR] Initialisation CAN echouee");
    while (1); // Boucle infinie
  }
  
  // Initialisation état feux
  commandes = {0, 0, 0, 0};
  
  // Configuration initiale des GPIO en sortie
  if (!sendWithAck(ID_IM_FVD, ID_AIM_FVD, REG_GPDDR, 0x0F, 0x00)) {
    Serial.println("[ERREUR] Config GPIO");
    while (1);
  }
  
  Serial.println("\n[SYSTEME PRET] Controle Feux Avant Droit");
  Serial.println("==================================");
}

// ==============================================
// LOOP
// ==============================================
void loop() {
  unsigned long now = millis();

  // Gestion cycle feux principal
  if (now - dernier_changement >= TEMPO_FEUX) {
    dernier_changement = now;
    updateFeux();
  }

  // Gestion clignotant indépendant
  if (now - dernier_clignotement >= TEMPO_CLIGNOT) {
    dernier_clignotement = now;
    clignotant_actif = !clignotant_actif;
    commandes.clign_d = clignotant_actif ? 1 : 0;

    uint8_t val_fvd = (commandes.clign_d << 3) | (commandes.phare << 2) |
                      (commandes.code << 1) | commandes.veilleuse;
    sendWithAck(ID_IM_FVD, ID_AIM_FVD, REG_GPLAT, MASK_FVD, val_fvd);
    printFeuxStatus();
  }
  
  delay(100); // Petite pause
}
