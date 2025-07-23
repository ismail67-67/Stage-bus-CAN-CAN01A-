/*******************************************************************************
* TP3 – Contrôle des feux d’un bloc optique via CAN (VMD - MCP2515)
* Séquence : Clignotant droit independent des feux chaque 1.6s → Veilleuse → Veilleuse+Code → Veilleuse+Phare → Eteint → ...
* Arduino Uno R3 + MCP2515 – Feux Avant Droit (modifiable)
 ******************************************************************************/

#include <SPI.h>
#include <mcp2515.h>

// ============================================================================
    // CONFIGURATION PRINCIPALE
// ============================================================================

// === Bloc cible à modifier facilement ici ===
#define NOM_BLOC      "FVG"           // Nom pour affichage (ex: "FVD", "FVG", "ARD")
#define ID_IM_BLOC    0x0E080000      // ID de trame IM (Input Message) {Rx}
#define ID_AIM_BLOC   0x0E200000      // ID de trame AIM (Ack Input Message) {TX}

// Pour utiliser d'autres modules VMD (changer ID_IM et ID_AIM si besoin) :
//  Avant gauche  : RX=0x0E080000 TX=0x0E200000
//  Avant droite  : RX=0x0E880000 TX=0x0EA00000

// === CAN ===
#define SPI_CS_PIN    9
#define CAN_SPEED     CAN_100KBPS
#define CAN_CLOCK     MCP_16MHZ

// === Registres GPIO du MCP25050 ===
#define REG_GPDDR     0x1F            // Direction des GPIO
#define REG_GPLAT     0x1E            // État des GPIO
#define MASK_BLOC     0x0F            // Masque pour bits 0–3

// === Temps (en ms) ===
#define TEMPO_FEUX     3500           // 3.5 s entre chaque étape des feux
#define TEMPO_CLIGNOT  1600           // 1.6 s pour le clignotement

// ============================================================================
    // VARIABLES GLOBALES
// ============================================================================

MCP2515 mcp2515(SPI_CS_PIN);

struct {
  uint8_t veilleuse : 1;
  uint8_t code      : 1;
  uint8_t phare     : 1;
  uint8_t clign_d   : 1;
} commandes;

unsigned long t_prev_feux = 0;
unsigned long t_prev_cligno = 0;
bool clignotant_on = false;

// ============================================================================
    // AFFICHAGE DE L’ÉTAT DES FEUX
// ============================================================================
void printFeuxStatus() {
  Serial.println("\n[ÉTAT DES FEUX - " NOM_BLOC "]");
  Serial.print("  Veilleuse : "); Serial.println(commandes.veilleuse ? "ON" : "OFF");
  Serial.print("  Code      : "); Serial.println(commandes.code ? "ON" : "OFF");
  Serial.print("  Phare     : "); Serial.println(commandes.phare ? "ON" : "OFF");
  Serial.print("  Clignotant: "); Serial.println(commandes.clign_d ? "ON" : "OFF");

  uint8_t val = (commandes.clign_d << 3) | (commandes.phare << 2) |
                (commandes.code << 1) | commandes.veilleuse;

  Serial.print("  Valeur binaire : 0b");
  for (int8_t i = 3; i >= 0; i--) Serial.print((val >> i) & 1);
  Serial.print(" (0x"); Serial.print(val, HEX); Serial.println(")");
  Serial.println("----------------------------------------");
}

// ============================================================================
    // INITIALISATION CAN
// ============================================================================
bool initCAN() {
  if (mcp2515.reset() != MCP2515::ERROR_OK) return false;
  if (mcp2515.setBitrate(CAN_SPEED, CAN_CLOCK) != MCP2515::ERROR_OK) return false;
  if (mcp2515.setNormalMode() != MCP2515::ERROR_OK) return false;
  return true;
}

// ============================================================================
   //  ENVOI DE TRAME AVEC ACQUITTEMENT
// ============================================================================
bool sendWithAck(uint32_t id, uint32_t ack_id, uint8_t reg, uint8_t mask, uint8_t value) {
  struct can_frame frame;
  frame.can_id  = id | CAN_EFF_FLAG;  // Mode étendu
  frame.can_dlc = 3;
  frame.data[0] = reg;
  frame.data[1] = mask;
  frame.data[2] = value;

  if (mcp2515.sendMessage(&frame) != MCP2515::ERROR_OK) {
    Serial.println("[ERREUR] Envoi CAN");
    return false;
  }

  struct can_frame ack;
  unsigned long t0 = millis();
  while (millis() - t0 < 500) {
    if (mcp2515.readMessage(&ack) == MCP2515::ERROR_OK) {
      if ((ack.can_id & 0x1FFFFFFF) == ack_id) {
        Serial.println("[ACQUITTEMENT OK]");
        return true;
      }
    }
  }
  Serial.println("[TIMEOUT] Aucun acquittement");
  return false;
}

// ============================================================================
    //  MISE À JOUR DU CYCLE DES FEUX PRINCIPAUX
// ============================================================================
void updateFeux() {
  static uint8_t etape = 0;

  // Réinitialise tout
  commandes.veilleuse = 0;
  commandes.code      = 0;
  commandes.phare     = 0;

  // Applique la logique du cycle "cumulatif"
  switch (etape) {
    case 1:
      commandes.veilleuse = 1;
      break;
    case 2:
      commandes.veilleuse = 1;
      commandes.code = 1;
      break;
    case 3:
      commandes.veilleuse = 1;
      commandes.phare = 1;
      break;
    default:
      // Étape 0 : tout éteint
      break;
  }

  // Passage à l’étape suivante
  etape = (etape + 1) % 4;

  // Envoie la trame mise à jour
  uint8_t val = (commandes.clign_d << 3) | (commandes.phare << 2) |
                (commandes.code << 1) | commandes.veilleuse;

  sendWithAck(ID_IM_BLOC, ID_AIM_BLOC, REG_GPLAT, MASK_BLOC, val);
  printFeuxStatus();
}

// ============================================================================
   //  SETUP PRINCIPAL
// ============================================================================
void setup() {
  Serial.begin(115200);
  while (!Serial);
  SPI.begin();

  Serial.println("[INIT] Initialisation CAN...");
  if (!initCAN()) {
    Serial.println("[ERREUR] CAN non initialisé");
    while (1);
  }

  // Initialisation GPIO en sortie
  if (!sendWithAck(ID_IM_BLOC, ID_AIM_BLOC, REG_GPDDR, MASK_BLOC, 0x00)) {
    Serial.println("[ERREUR] Configuration GPIO");
    while (1);
  }

  commandes = {0, 0, 0, 0};
  Serial.println("[SYSTEME PRET] Contrôle des feux " NOM_BLOC);
}

// ============================================================================
     //  LOOP
// ============================================================================
void loop() {
  unsigned long now = millis();

  // Séquence cyclique des feux (veilleuse → code → phare)
  if (now - t_prev_feux >= TEMPO_FEUX) {
    t_prev_feux = now;
    updateFeux();
  }

  // Clignotement (bit 3)
  if (now - t_prev_cligno >= TEMPO_CLIGNOT) {
    t_prev_cligno = now;
    clignotant_on = !clignotant_on;
    commandes.clign_d = clignotant_on;

    uint8_t val = (commandes.clign_d << 3) | (commandes.phare << 2) |
                  (commandes.code << 1) | commandes.veilleuse;

    sendWithAck(ID_IM_BLOC, ID_AIM_BLOC, REG_GPLAT, MASK_BLOC, val);
    printFeuxStatus();
  }

  delay(50);  // Légère pause
}
