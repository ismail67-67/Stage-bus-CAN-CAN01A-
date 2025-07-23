// =============================================================
// TP2 - Acquisition de l’état des feux via le commodo CAN
// Objectif : Lire les entrées du module (GP0 à GP7), afficher les feux actifs et l'état d commodo,
// Envoyer une trame de commande combinée (logique TOR + addition de bits).
// =============================================================

#include <SPI.h>
#include <mcp2515.h>

// ======================= PARAMÈTRES CAN ==========================
const int SPI_CS_PIN = 9;                  // CS du module MCP2515
MCP2515 mcp2515(SPI_CS_PIN);

#define CAN_SPEED  CAN_100KBPS
#define CAN_CLOCK  MCP_16MHZ

// ID étendus pour la communication avec le module CAN01A (MCP25050)
#define ID_IM_COMMODO   0x05081F00  // Instruction maître
#define ID_AIM_COMMODO  0x05200000  // Accusé de réception (ACK)
#define ID_IRM_COMMODO  0x05041E07  // Instruction de lecture
#define ID_OM_COMMODO   0x05400000  // Observation maître (retour d’état)

#define REG_GPDDR  0x1F  // Direction des broches
#define REG_GPLAT  0x1E  // Valeur des sorties
#define REG_IOTEN  0x1C  // Activation I/O (inutile ici)

// Masques associés aux broches GP0 à GP7 du module CAN
#define MASK_VEILLEUSE  (1 << 0)  // GP0 = 0x01
#define MASK_WARNING    (1 << 1)  // GP1 = 0x02
#define MASK_PHARE      (1 << 2)  // GP2 = 0x04
#define MASK_CODE       (1 << 3)  // GP3 = 0x08
#define MASK_CLIGN_G    (1 << 4)  // GP4 = 0x10
#define MASK_CLIGN_D    (1 << 5)  // GP5 = 0x20
#define MASK_STOP       (1 << 6)  // GP6 = 0x40
#define MASK_KLAXON     (1 << 7)  // GP7 = 0x80

#define POLLING_INTERVAL 50       // Délai entre chaque requête IRM (ms)

// ======================= VARIABLES GLOBALES =======================
uint8_t dernierEtatBoutons = 0;    // Dernier état connu (pour éviter les doublons)
unsigned long dernierPoll = 0;     // Timestamp de la dernière requête IRM
bool ackPrinted = false;           // Évite de réafficher le même ACK
bool ackPending = false;           // Attente d’un ACK après envoi d’une commande

// ======================== INITIALISATION CAN ========================
void initCAN() {
  SPI.begin();
  while (mcp2515.reset()           != MCP2515::ERROR_OK);
  while (mcp2515.setBitrate(CAN_SPEED, CAN_CLOCK) != MCP2515::ERROR_OK);
  while (mcp2515.setNormalMode()   != MCP2515::ERROR_OK);
}

// Configuration du module en entrée (tous les GP en entrée)
void configurerModule() {
  struct can_frame trame;
  trame.can_id  = ID_IM_COMMODO | CAN_EFF_FLAG;
  trame.can_dlc = 3;
  trame.data[0] = REG_GPDDR;   // Direction
  trame.data[1] = 0xFF;        // Masque complet
  trame.data[2] = 0xFF;        // Tous en entrée
  mcp2515.sendMessage(&trame);
}

// ======================= AFFICHAGE ET ENVOI =========================

// Affiche l’état de chaque feu dans un format lisible
void afficherEtatFeux(uint8_t boutons) {
  Serial.println("\n==== ETAT FEUX COMMODO ====");
  Serial.print("Clignotant Gauche: "); Serial.println(boutons & MASK_CLIGN_G    ? "ACTIF" : "inactif");
  Serial.print("Clignotant Droit : "); Serial.println(boutons & MASK_CLIGN_D    ? "ACTIF" : "inactif");
  Serial.print("Stop             : "); Serial.println(boutons & MASK_STOP       ? "ACTIF" : "inactif");
  Serial.print("Klaxon           : "); Serial.println(boutons & MASK_KLAXON     ? "ACTIF" : "inactif");
  Serial.print("Veilleuse        : "); Serial.println(boutons & MASK_VEILLEUSE  ? "ACTIF" : "inactif");
  Serial.print("Phare            : "); Serial.println(boutons & MASK_PHARE      ? "ACTIF" : "inactif");
  Serial.print("Code             : "); Serial.println(boutons & MASK_CODE       ? "ACTIF" : "inactif");
  Serial.print("Warning          : "); Serial.println(boutons & MASK_WARNING    ? "ACTIF" : "inactif");
  Serial.println("===========================\n");
}

// Envoie la commande combinée (logique TOR) vers le module
void envoyerCommande(uint8_t valeur) {
  struct can_frame trame;
  trame.can_id  = ID_IM_COMMODO | CAN_EFF_FLAG;
  trame.can_dlc = 3;
  trame.data[0] = REG_GPLAT;    // Écriture de l’état
  trame.data[1] = 0xFF;         // Masque complet
  trame.data[2] = valeur;       // Valeur à activer (ex: 0xC0 pour stop+klaxon)

  mcp2515.sendMessage(&trame);

  Serial.print("[TX] Feux actifs : 0x"); Serial.print(valeur, HEX);
  Serial.print(" (0b"); Serial.print(valeur, BIN); Serial.println(")");

  ackPrinted = false;
  ackPending = true;
}

// Demande l’état actuel des entrées du module (via trame IRM)
void demanderEtat() {
  struct can_frame trame;
  trame.can_id  = ID_IRM_COMMODO | CAN_EFF_FLAG;
  trame.can_dlc = 1;
  trame.data[0] = REG_GPLAT;  // Lecture de l’état des sorties
  mcp2515.sendMessage(&trame);
  dernierPoll = millis();
}

// ====================== TRAITEMENT DES TRAMES ========================

void traiterMessageCAN(can_frame trame) {
  uint32_t id = trame.can_id & 0x1FFFFFFF;

  // Trame de retour d’état (OM)
  if (id == ID_OM_COMMODO && trame.can_dlc >= 2) {
    uint8_t boutons = ~trame.data[1]; // Inversion logique : appui = 1

    // Changement d’état détecté
    if (boutons != dernierEtatBoutons) {
      dernierEtatBoutons = boutons;
      envoyerCommande(boutons);    // Envoi des feux actifs combinés
      afficherEtatFeux(boutons);   // Affichage détaillé
    }
  }

  // Trame d’ACK (AIM)
  else if (id == ID_AIM_COMMODO && ackPending && !ackPrinted) {
    Serial.print("[RX] ACK depuis 0x"); Serial.print(ID_AIM_COMMODO, HEX);
    Serial.print(" : 0x"); Serial.print(trame.data[2], HEX);
    Serial.print(" (0b"); Serial.print(trame.data[2], BIN); Serial.println(")");

    ackPrinted  = true;
    ackPending  = false;
  }
}

// Boucle de gestion du CAN (lecture et polling)
void loopCAN() {
  struct can_frame trame;

  // Lecture d’un message reçu
  if (mcp2515.readMessage(&trame) == MCP2515::ERROR_OK) {
    traiterMessageCAN(trame);
  }

  // Requête périodique d’état toutes les 50 ms
  if (millis() - dernierPoll >= POLLING_INTERVAL) {
    demanderEtat();
  }
}

// ============================== SETUP / LOOP ===============================

void setup() {
  Serial.begin(115200);
  while (!Serial);              // Attente ouverture moniteur série

  initCAN();                    // Initialisation du bus CAN
  configurerModule();           // Tous les GP en entrée
  demanderEtat();               // Première requête d’état

  Serial.println("\n[SYSTEME PRÊT - MODE TOR + COMBINAISON]");
}

void loop() {
  loopCAN();                    // Gestion continue du bus CAN
}
