/******************************************************************************
 * TP3 – Contrôle cyclique d’un bloc optique via CAN (MCP2515 - VMD)
 * --------------------------------------------------------------------------
 * Séquence : Clignotant droit independent des feux chaque 1.6s → Veilleuse → Veilleuse+Code → Veilleuse+Phare → Eteint → ...
 * - Utilisation de l’interruption matérielle (INT) pour réception d’ACK.
 * - Pas de polling actif
 * - Feux Avant Droit (modifiable)
 * Matériel : Arduino Uno R3 + Shield CAN MCP2515 (INT sur D2 recommandé)
 ******************************************************************************/

#include <SPI.h>
#include <mcp2515.h>

//========== CONFIGURATION GÉNÉRALE ==========

#define SPI_CS_PIN      9           // Broche CS du MCP2515
#define INT_PIN         2           // Broche d'interruption (INT MCP2515 → D2)
#define CAN_SPEED       CAN_100KBPS
#define CAN_CLOCK       MCP_16MHZ

#define NOM_BLOC        "FAD"       // Nom du bloc (ex : FVD = Feux Avant Droit)
#define ID_IM_BLOC      0x0E880000  // ID trame IM  (commande vers bloc)
#define ID_AIM_BLOC     0x0EA00000  // ID trame AIM (acquittement depuis bloc)

#define REG_GPDDR       0x1F        // Registre de direction (entrée/sortie)
#define REG_GPLAT       0x1E        // Registre de latence (valeurs des GPIO)
#define MASK_BLOC       0x0F        // GP0–GP3 utilisés (veilleuse à clignotant)

#define TEMPO_FEUX      3500        // Durée entre états de feux (ms)
#define TEMPO_CLIGNOT   1600        // Fréquence clignotant droit (ms)

MCP2515 mcp2515(SPI_CS_PIN);       // Instance du contrôleur CAN

//========== STRUCTURE ÉTAT DES FEUX ==========

struct {
  uint8_t veilleuse : 1;
  uint8_t code      : 1;
  uint8_t phare     : 1;
  uint8_t clign_d   : 1;
} commandes;

unsigned long t_prev_feux    = 0;
unsigned long t_prev_cligno  = 0;
bool clignotant_on = false;

volatile bool ackRecu       = false;   // Flag déclenché par l'interruption
volatile bool attenteAck    = false;   // Signal qu'on attend un ACK

//========== INITIALISATION DU BUS CAN ==========

bool initialiserCAN() {
  if (mcp2515.reset() != MCP2515::ERROR_OK) return false;
  if (mcp2515.setBitrate(CAN_SPEED, CAN_CLOCK) != MCP2515::ERROR_OK) return false;
  if (mcp2515.setNormalMode() != MCP2515::ERROR_OK) return false;
  return true;
}

bool configurerSortieGPIO() {
  struct can_frame frame;
  frame.can_id  = ID_IM_BLOC | CAN_EFF_FLAG;
  frame.can_dlc = 3;
  frame.data[0] = REG_GPDDR;     // Registre direction
  frame.data[1] = MASK_BLOC;     // Cible : GP0 à GP3
  frame.data[2] = 0x00;          // 0 = Sortie
  return (mcp2515.sendMessage(&frame) == MCP2515::ERROR_OK);
}

//========== AFFICHAGE DÉTAILLÉ SUR MONITEUR ==========

void afficherEtatFeux() {
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

//========== ENVOI DE COMMANDE AVEC GESTION D’ACK ==========

bool envoyerCommande(uint8_t val) {
  struct can_frame frame;
  frame.can_id  = ID_IM_BLOC | CAN_EFF_FLAG;
  frame.can_dlc = 3;
  frame.data[0] = REG_GPLAT;
  frame.data[1] = MASK_BLOC;
  frame.data[2] = val;

  ackRecu = false;
  attenteAck = true;

  if (mcp2515.sendMessage(&frame) != MCP2515::ERROR_OK) {
    Serial.println("[ERREUR] Échec d’envoi CAN");
    return false;
  }

  // Attend l’interruption ou timeout
  unsigned long t0 = millis();
  while (attenteAck && (millis() - t0 < 500));

  if (ackRecu) return true;

  Serial.println("[TIMEOUT] Aucun ACK reçu");
  return false;
}

//========== ROUTINE D’INTERRUPTION CAN ==========

void interruptionCAN() {
  struct can_frame frame;
  while (mcp2515.readMessage(&frame) == MCP2515::ERROR_OK) {
    uint32_t id = frame.can_id & 0x1FFFFFFF;
    if (id == ID_AIM_BLOC) {
      ackRecu = true;
      attenteAck = false;
      Serial.print("[ACK] Reçu de 0x"); Serial.print(ID_AIM_BLOC, HEX);
      Serial.print(" → Valeur : 0x"); Serial.println(frame.data[2], HEX);
    }
  }
}

//========== MISE À JOUR DE LA SÉQUENCE DE FEUX ==========

void miseAJourFeux() {
  static uint8_t etape = 0;
  commandes = {0, 0, 0, commandes.clign_d};  // Reset sauf clignotant

  switch (etape) {
    case 1: commandes.veilleuse = 1; break;
    case 2: commandes.veilleuse = commandes.code = 1; break;
    case 3: commandes.veilleuse = commandes.phare = 1; break;
    default: break;  // Étape 0 : tout éteint
  }

  etape = (etape + 1) % 4;

  uint8_t val = (commandes.clign_d << 3) | (commandes.phare << 2) |
                (commandes.code << 1) | commandes.veilleuse;

  envoyerCommande(val);
  afficherEtatFeux();
}

//========== CLIGNOTANT DROIT (AUTONOME) ==========

void miseAJourClignotant() {
  clignotant_on = !clignotant_on;
  commandes.clign_d = clignotant_on;

  uint8_t val = (commandes.clign_d << 3) | (commandes.phare << 2) |
                (commandes.code << 1) | commandes.veilleuse;

  envoyerCommande(val);
  afficherEtatFeux();
}

//========== SETUP PRINCIPAL ==========

void setup() {
  Serial.begin(115200);
  while (!Serial);
  SPI.begin();

  pinMode(INT_PIN, INPUT);
  attachInterrupt(digitalPinToInterrupt(INT_PIN), interruptionCAN, FALLING);

  Serial.println("[INIT] Initialisation du bus CAN...");
  if (!initialiserCAN()) {
    Serial.println("[ERREUR FATALE] Bus CAN non opérationnel !");
    while (1);
  }

  if (!configurerSortieGPIO()) {
    Serial.println("[ERREUR] Configuration GPIO échouée !");
    while (1);
  }

  commandes = {0, 0, 0, 0};
  Serial.println("[PRÊT] Système opérationnel pour le bloc " NOM_BLOC);
}

//========== LOOP PRINCIPALE ==========

void loop() {
  unsigned long now = millis();

  if (now - t_prev_feux >= TEMPO_FEUX) {
    t_prev_feux = now;
    miseAJourFeux();
  }

  if (now - t_prev_cligno >= TEMPO_CLIGNOT) {
    t_prev_cligno = now;
    miseAJourClignotant();
  }
}
