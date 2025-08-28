// ============================================================================
//  TP4 VMD - Commande des feux via CAN
//  ---------------------------------------------------------------------------
//  Objectif : Lire l'état du commodo (via messages CAN) et commander en temps
//  réel les différents blocs optiques du véhicule multiplexé didactique (VMD).
//
//  Fonctionnalités :
//   - Lecture du commodo via trames OM (Output Message) envoyées automatiquement
//   - Requête périodique (IRM) pour rafraîchir l'état si besoin
//   - Gestion du clignotement gauche/droit et warning
//   - Commande des feux de stop et du klaxon
//   - Gestion simultanée des veilleuses, code et phares
//
//  Matériel :
//   - Arduino UNO R3
//   - Shield MCP2515 CAN
//   - Modules VMD (commodo + blocs optiques)
// ============================================================================

#include <SPI.h>
#include <mcp2515.h>

//========== CONFIGURATION CAN ET MCP2515 ==========

const int SPI_CS_PIN = 9;          // Broche CS (Chip Select) reliée au MCP2515
MCP2515 mcp2515(SPI_CS_PIN);       // Instance du contrôleur CAN

#define CAN_SPEED   CAN_100KBPS    // Débit du bus CAN (100 kbps sur VMD)
#define CAN_CLOCK   MCP_16MHZ      // Quartz du MCP2515 = 16 MHz

//========== IDENTIFIANTS CAN DES MODULES VMD ==========
// Ces ID sont définis par le constructeur du VMD :
//  - IM : Input Message (commande envoyée vers un module)
//  - IRM : Info Request Message (requête d'état)
//  - OM : Output Message (réponse automatique envoyée par le module)

#define ID_IM_COMMODO   0x05081F00 // IM : configurer le commodo
#define ID_IRM_COMMODO  0x05041E07 // IRM : demander état du commodo
#define ID_OM_COMMODO   0x05400000 // OM : état envoyé automatiquement par le commodo

#define ID_IM_FAD       0x0E880000 // Bloc feux avant droit
#define ID_IM_FAG       0x0E080000 // Bloc feux avant gauche
#define ID_IM_ARD       0x0F880000 // Bloc feux arrière droit
#define ID_IM_ARG       0x0F080000 // Bloc feux arrière gauche

//========== COMMANDES DES LEDS SUR MODULES DE FEUX ==========
// Bits utilisés dans le registre GPLAT (GPIO latch) du MCP25050

#define LED_CLIGN_AV     0x08 // Clignotant avant (bit 3)
#define LED_CLIGN_AR     0x04 // Clignotant arrière (bit 2)
#define LED_STOP_CMD     0x02 // Feu stop (bit 1)
#define LED_KLAXON_CMD   0x01 // Klaxon (bit 0)
#define LED_PHARE        0x04 // Phare (bit 2)
#define LED_CODE         0x02 // Code (bit 1)

//========== REGISTRES INTERNES MCP25050 ==========
// Le MCP25050 est un expander CAN→GPIO. On configure ses broches via trames IM.

#define REG_GPDDR   0x1F // GPDDR = direction (1 = entrée, 0 = sortie)
#define REG_GPLAT   0x1E // GPLAT = état logique des broches (LEDs ON/OFF)
#define REG_IOTEN   0x1C // IOTEN = activation des interruptions

//========== MASQUES DES BOUTONS DU COMMODO ==========
// Les bits lus dans l’OM du commodo correspondent à l’état des boutons.

#define MASK_VEILLEUSE  (1 << 0)
#define MASK_WARNING    (1 << 1)
#define MASK_PHARE      (1 << 2)
#define MASK_CODE       (1 << 3)
#define MASK_CLIGN_G    (1 << 4)
#define MASK_CLIGN_D    (1 << 5)
#define MASK_STOP       (1 << 6)
#define MASK_KLAXON     (1 << 7)

//========== PARAMÈTRES DE TEMPORISATION ==========

#define POLLING_INTERVAL     200   // Période entre deux IRM (ms)
#define BLINK_INTERVAL       800   // Durée ON/OFF clignotant (800 ms → 1 Hz)
#define DISPLAY_INTERVAL     1000  // Fréquence d'affichage série (ms)
#define FEUX_UPDATE_INTERVAL 200   // Intervalle minimal entre mises à jour feux (ms)
#define CAN_SEND_DELAY       5     // Délai minimal entre deux envois CAN (ms)

//========== STRUCTURE DE DONNÉES ==========
// Représente l’état logique du commodo et des modules de feux.

typedef struct {
  bool veilleuse, warning, phare, code;
  bool clignG, clignD, stop, klaxon;
  uint8_t fad, fag, ard, arg; // Valeurs envoyées aux blocs optiques
} EtatCommodo;

EtatCommodo etat = {false};          // État courant
EtatCommodo dernierEtat = {false};   // Dernier état affiché (pour éviter répétitions)

// Variables de timing
unsigned long tLastPoll = 0, tLastBlink = 0, tLastDisplay = 0, tLastFeuxUpdate = 0, tLastCanSend = 0;
bool blinkState = false; // État ON/OFF clignotant (géré en toggle)

//========== INITIALISATION CAN ET MODULES ==========

// Configure le MCP2515 en mode normal
void initCAN() {
  SPI.begin();
  mcp2515.reset();
  mcp2515.setBitrate(CAN_SPEED, CAN_CLOCK);
  mcp2515.setNormalMode();
}

// Configure le module commodo (entrées uniquement)
void configCommodo() {
  struct can_frame frame = { .can_id = ID_IM_COMMODO | CAN_EFF_FLAG, .can_dlc = 3 };
  frame.data[0] = REG_GPDDR; frame.data[1] = 0xFF; frame.data[2] = 0xFF; // Toutes les broches en entrée
  mcp2515.sendMessage(&frame);
  delay(CAN_SEND_DELAY);

  frame.data[0] = REG_IOTEN; frame.data[2] = 0xFF; // Active interruptions
  mcp2515.sendMessage(&frame);
  delay(CAN_SEND_DELAY);
}

// Configure un module de feux (sorties)
void configFeux(uint32_t id) {
  struct can_frame frame = { .can_id = id | CAN_EFF_FLAG, .can_dlc = 3 };
  frame.data[0] = REG_GPDDR; frame.data[1] = 0x0F; frame.data[2] = 0xF0; // GP0..GP3 = sorties
  mcp2515.sendMessage(&frame);
  delay(CAN_SEND_DELAY);

  frame.data[0] = REG_GPLAT; frame.data[2] = 0x00; // Tout éteint au départ
  mcp2515.sendMessage(&frame);
  delay(CAN_SEND_DELAY);
}

//========== ENVOI DES COMMANDES AUX MODULES DE FEUX ==========

void envoyerFeux(uint32_t id, uint8_t cmd) {
  // Contrôle du débit d'envoi (évite surcharge bus)
  if (millis() - tLastCanSend < CAN_SEND_DELAY) {
    delay(CAN_SEND_DELAY - (millis() - tLastCanSend));
  }

  struct can_frame frame = { .can_id = id | CAN_EFF_FLAG, .can_dlc = 3 };
  frame.data[0] = REG_GPLAT; frame.data[1] = 0x0F; frame.data[2] = cmd;
  mcp2515.sendMessage(&frame);
  tLastCanSend = millis();
}

//========== LOGIQUE DE TRAITEMENT DU COMMODO ==========
// Met à jour l’état des feux en fonction des boutons lus

void updateFeux() {
  // Gestion clignotement ON/OFF
  if (millis() - tLastBlink > BLINK_INTERVAL) {
    blinkState = !blinkState;
    tLastBlink = millis();
  }

  // Réinitialisation des blocs optiques (tout éteint)
  etat.fad = etat.fag = etat.ard = etat.arg = 0;

  // Veilleuses → avant gauche/droite + arrière gauche/droite
  if (etat.veilleuse) {
    etat.fag |= 0x01; etat.fad |= 0x01;   // Veilleuses avant
    etat.arg |= 0x08; etat.ard |= 0x08;   // Veilleuses arrière
  }

  // Warning → priorité sur clignotants
  if (etat.warning) {
    etat.clignG = etat.clignD = true;
  }

  // Code et Phare (avant seulement)
  if (etat.code)  { etat.fag |= LED_CODE;  etat.fad |= LED_CODE; }
  if (etat.phare) { etat.fag |= LED_PHARE; etat.fad |= LED_PHARE; }

  // Clignotants gauche (avant + arrière)
  if (etat.clignG && blinkState) {
    etat.fag |= LED_CLIGN_AV; etat.arg |= LED_CLIGN_AR;
  }
  // Clignotants droit (avant + arrière)
  if (etat.clignD && blinkState) {
    etat.fad |= LED_CLIGN_AV; etat.ard |= LED_CLIGN_AR;
  }

  // Stop → feux arrière (ARD + ARG)
  if (etat.stop) {
    etat.ard |= LED_STOP_CMD; etat.arg |= LED_STOP_CMD;
  }

  // Klaxon → AR gauche uniquement
  if (etat.klaxon) {
    etat.arg |= LED_KLAXON_CMD;
  }

  // Envoi des états finaux aux modules CAN
  envoyerFeux(ID_IM_FAD, etat.fad);
  envoyerFeux(ID_IM_FAG, etat.fag);
  envoyerFeux(ID_IM_ARD, etat.ard);
  envoyerFeux(ID_IM_ARG, etat.arg);
}

//========== AFFICHAGE SERIAL ==========

void afficherEtat() {
  // Affiche seulement si l'état a changé ou si délai écoulé
  if (memcmp(&etat, &dernierEtat, sizeof(EtatCommodo)) != 0 || millis() - tLastDisplay > DISPLAY_INTERVAL) {
    Serial.println("\n=== ETAT COMMODO ===");
    Serial.print("Clign. G  : "); Serial.println(etat.clignG ? "Actif" : "Inactif");
    Serial.print("Clign. D  : "); Serial.println(etat.clignD ? "Actif" : "Inactif");
    Serial.print("STOP      : "); Serial.println(etat.stop ? "Actif" : "Inactif");
    Serial.print("KLAXON    : "); Serial.println(etat.klaxon ? "Actif" : "Inactif");
    Serial.print("Veilleuse : "); Serial.println(etat.veilleuse ? "Actif" : "Inactif");
    Serial.print("Warning   : "); Serial.println(etat.warning ? "Actif" : "Inactif");
    Serial.print("Phare     : "); Serial.println(etat.phare ? "Actif" : "Inactif");
    Serial.print("Code      : "); Serial.println(etat.code ? "Actif" : "Inactif");

    Serial.println("=== ETAT FEUX ===");
    Serial.print("FAD : 0x"); Serial.println(etat.fad, HEX);
    Serial.print("FAG : 0x"); Serial.println(etat.fag, HEX);
    Serial.print("ARD : 0x"); Serial.println(etat.ard, HEX);
    Serial.print("ARG : 0x"); Serial.println(etat.arg, HEX);

    memcpy(&dernierEtat, &etat, sizeof(EtatCommodo)); // Sauvegarde pour comparer
    tLastDisplay = millis();
  }
}

//========== TRAITEMENT DES MESSAGES CAN ENTRANTS ==========

void recevoirEtatCommodo(struct can_frame f) {
  // Vérifie qu’il s’agit bien d’un OM du commodo
  if ((f.can_id & 0x1FFFFFFF) != ID_OM_COMMODO || f.can_dlc < 2) return;
  uint8_t p = f.data[1]; // Byte contenant les bits des boutons

  // Bits actifs à l'état bas → inversion avec "!" 
  etat.veilleuse = !(p & MASK_VEILLEUSE);
  etat.warning   = !(p & MASK_WARNING);
  etat.phare     = !(p & MASK_PHARE);
  etat.code      = !(p & MASK_CODE);
  etat.clignG    = !(p & MASK_CLIGN_G);
  etat.clignD    = !(p & MASK_CLIGN_D);
  etat.stop      = !(p & MASK_STOP);
  etat.klaxon    = !(p & MASK_KLAXON);
}

//========== ENVOI PÉRIODIQUE D’UNE IRM ==========

void demanderEtat() {
  // Respect délai minimal entre deux envois CAN
  if (millis() - tLastCanSend >= CAN_SEND_DELAY) {
    struct can_frame frame = { .can_id = ID_IRM_COMMODO | CAN_EFF_FLAG, .can_dlc = 1 };
    frame.data[0] = REG_GPLAT; // On demande la lecture de l’état
    mcp2515.sendMessage(&frame);
    tLastPoll = millis();
    tLastCanSend = millis();
  }
}

//========== SETUP ==========

void setup() {
  Serial.begin(115200);
  while (!Serial); // Attente console

  initCAN();
  configCommodo();        // Configure le commodo en entrée
  configFeux(ID_IM_FAD);  // Configure les 4 blocs optiques
  configFeux(ID_IM_FAG);
  configFeux(ID_IM_ARD);
  configFeux(ID_IM_ARG);

  Serial.println("\nTP4 VMD - Système prêt (version corrigée pour PicoScope)");
}

//========== LOOP PRINCIPALE ==========

void loop() {
  struct can_frame frame;

  // Lecture des messages CAN entrants
  if (mcp2515.readMessage(&frame) == MCP2515::ERROR_OK) {
    recevoirEtatCommodo(frame);
  }

  // Requête périodique IRM pour rafraîchir état du commodo
  if (millis() - tLastPoll >= POLLING_INTERVAL) {
    demanderEtat();
  }

  // Mise à jour régulière des feux + affichage console
  if (millis() - tLastFeuxUpdate >= FEUX_UPDATE_INTERVAL) {
    updateFeux();
    afficherEtat();
    tLastFeuxUpdate = millis();
  }
}
