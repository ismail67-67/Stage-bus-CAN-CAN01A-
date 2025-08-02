// ============================================================================
//  TP4 VMD - Commande des feux via CAN (Arduino Uno + MCP2515)
// ============================================================================

#include <SPI.h>
#include <mcp2515.h>

//========== CONFIGURATION CAN ET MCP2515 ==========

const int SPI_CS_PIN = 9; // Broche CS du MCP2515 connectée à D9 de l'Arduino
MCP2515 mcp2515(SPI_CS_PIN); // Création de l'objet MCP2515

#define CAN_SPEED   CAN_100KBPS           // Vitesse du bus CAN
#define CAN_CLOCK   MCP_16MHZ             // Horloge quartz du module CAN

//==========  IDENTIFIANTS CAN DES MODULES VMD ==========

#define ID_IM_COMMODO   0x05081F00 // Input Message pour configurer le commodo
#define ID_IRM_COMMODO  0x05041E07 // Requête d'état (Info Request Message)
#define ID_OM_COMMODO   0x05400000 // Message d'état automatique (Output Message)

#define ID_IM_FAD       0x0E880000 // Module feux avant droit
#define ID_IM_FAG       0x0E080000 // Module feux avant gauche
#define ID_IM_ARD       0x0F880000 // Module feux arrière droit
#define ID_IM_ARG       0x0F080000 // Module feux arrière gauche

//==========  COMMANDES DES LEDS SUR MODULES DE FEUX ==========

#define LED_CLIGN_AV     0x08 // Clignotant avant (bit 3)
#define LED_CLIGN_AR     0x04 // Clignotant arrière (bit 2)
#define LED_STOP_CMD     0x02 // Feux stop (bit 1)
#define LED_KLAXON_CMD   0x01 // Klaxon (bit 0)
#define LED_PHARE        0x04 // Phare (bit 2)
#define LED_CODE         0x02 // Code (bit 1)

//==========  REGISTRES INTERNES MCP25050 ==========

#define REG_GPDDR   0x1F // Direction des broches (entrée/sortie)
#define REG_GPLAT   0x1E // État logique des broches (LED ON/OFF)
#define REG_IOTEN   0x1C // Interrupt enable

//==========  MASQUES DES BP DU COMMODO ==========

#define MASK_VEILLEUSE  (1 << 0)
#define MASK_WARNING    (1 << 1)
#define MASK_PHARE      (1 << 2)
#define MASK_CODE       (1 << 3)
#define MASK_CLIGN_G    (1 << 4)
#define MASK_CLIGN_D    (1 << 5)
#define MASK_STOP       (1 << 6)
#define MASK_KLAXON     (1 << 7)

//==========  PARAMÈTRES TEMPORISATION ==========

#define POLLING_INTERVAL     200  // En ms : intervalle entre deux requêtes IRM
#define BLINK_INTERVAL       800   // Durée d'un état ON ou OFF du clignotant
#define DISPLAY_INTERVAL     1000   // Fréquence d'affichage de l'état (sérial)
#define FEUX_UPDATE_INTERVAL 200    // Intervalle minimal entre les mises à jour des feux (en ms)
#define CAN_SEND_DELAY       5     // Délai minimal entre chaque envoi CAN (en ms)

//==========  STRUCTURE DE DONNÉES ET VARIABLES GLOBALES ==========

typedef struct {
  bool veilleuse, warning, phare, code;
  bool clignG, clignD, stop, klaxon;
  uint8_t fad, fag, ard, arg;
} EtatCommodo;

EtatCommodo etat = {false}; // Structure principale
EtatCommodo dernierEtat = {false}; // Pour affichage conditionnel

unsigned long tLastPoll = 0, tLastBlink = 0, tLastDisplay = 0, tLastFeuxUpdate = 0, tLastCanSend = 0;
bool blinkState = false;

//==========  INITIALISATION CAN ET MODULES ==========

void initCAN() {
  SPI.begin();
  mcp2515.reset();
  mcp2515.setBitrate(CAN_SPEED, CAN_CLOCK);
  mcp2515.setNormalMode();
}

void configCommodo() {
  struct can_frame frame = { .can_id = ID_IM_COMMODO | CAN_EFF_FLAG, .can_dlc = 3 };
  frame.data[0] = REG_GPDDR; frame.data[1] = 0xFF; frame.data[2] = 0xFF; // Tout en entrée
  mcp2515.sendMessage(&frame);
  delay(CAN_SEND_DELAY);

  frame.data[0] = REG_IOTEN; frame.data[2] = 0xFF; // Active interruptions
  mcp2515.sendMessage(&frame);
  delay(CAN_SEND_DELAY);
}

void configFeux(uint32_t id) {
  struct can_frame frame = { .can_id = id | CAN_EFF_FLAG, .can_dlc = 3 };
  frame.data[0] = REG_GPDDR; frame.data[1] = 0x0F; frame.data[2] = 0xF0; // Sorties bits 3-0
  mcp2515.sendMessage(&frame);
  delay(CAN_SEND_DELAY);

  frame.data[0] = REG_GPLAT; frame.data[2] = 0x00; // Éteint les feux
  mcp2515.sendMessage(&frame);
  delay(CAN_SEND_DELAY);
}

//==========  ENVOI DES COMMANDES VERS MODULES DE FEUX (avec contrôle de débit) ==========

void envoyerFeux(uint32_t id, uint8_t cmd) {
  // Vérifie qu'on a attendu assez longtemps depuis le dernier envoi
  if (millis() - tLastCanSend < CAN_SEND_DELAY) {
    delay(CAN_SEND_DELAY - (millis() - tLastCanSend));
  }
  
  struct can_frame frame = { .can_id = id | CAN_EFF_FLAG, .can_dlc = 3 };
  frame.data[0] = REG_GPLAT; frame.data[1] = 0x0F; frame.data[2] = cmd;
  mcp2515.sendMessage(&frame);
  tLastCanSend = millis();
}

//==========  LOGIQUE DE TRAITEMENT DES BOUTONS DU COMMODO ==========

void updateFeux() {
  // Clignotement ON/OFF automatique
  if (millis() - tLastBlink > BLINK_INTERVAL) {
    blinkState = !blinkState;
    tLastBlink = millis();
  }

  // Réinitialisation
  etat.fad = etat.fag = etat.ard = etat.arg = 0;

  // Veilleuses avant (bit 0), arrière (bit 3)
  if (etat.veilleuse) {
    etat.fag |= 0x01; etat.fad |= 0x01;
    etat.arg |= 0x08; etat.ard |= 0x08;
  }

  // Warning prioritaire sur les clignotants
  if (etat.warning) {
    etat.clignG = etat.clignD = true;
  }

  // Code et Phare
  if (etat.code) { etat.fag |= LED_CODE; etat.fad |= LED_CODE; }
  if (etat.phare) { etat.fag |= LED_PHARE; etat.fad |= LED_PHARE; }

  // Clignotants clignent si état actif
  if (etat.clignG && blinkState) {
    etat.fag |= LED_CLIGN_AV; etat.arg |= LED_CLIGN_AR;
  }
  if (etat.clignD && blinkState) {
    etat.fad |= LED_CLIGN_AV; etat.ard |= LED_CLIGN_AR;
  }

  // Stop actif permanent
  if (etat.stop) {
    etat.ard |= LED_STOP_CMD; etat.arg |= LED_STOP_CMD;
  }

  // Klaxon arrière gauche
  if (etat.klaxon) {
    etat.arg |= LED_KLAXON_CMD;
  }

  // Envoi des états finaux aux modules (avec délai intégré dans envoyerFeux())
  envoyerFeux(ID_IM_FAD, etat.fad);
  envoyerFeux(ID_IM_FAG, etat.fag);
  envoyerFeux(ID_IM_ARD, etat.ard);
  envoyerFeux(ID_IM_ARG, etat.arg);
}

//==========  AFFICHAGE SERIAL DE L'ÉTAT DU COMMODO ==========

void afficherEtat() {
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

    memcpy(&dernierEtat, &etat, sizeof(EtatCommodo));
    tLastDisplay = millis();
  }
}

//==========  TRAITEMENT DES MESSAGES CAN ENTRANTS ==========

void recevoirEtatCommodo(struct can_frame f) {
  if ((f.can_id & 0x1FFFFFFF) != ID_OM_COMMODO || f.can_dlc < 2) return;
  uint8_t p = f.data[1];

  etat.veilleuse = !(p & MASK_VEILLEUSE);
  etat.warning   = !(p & MASK_WARNING);
  etat.phare     = !(p & MASK_PHARE);
  etat.code      = !(p & MASK_CODE);
  etat.clignG    = !(p & MASK_CLIGN_G);
  etat.clignD    = !(p & MASK_CLIGN_D);
  etat.stop      = !(p & MASK_STOP);
  etat.klaxon    = !(p & MASK_KLAXON);

  // On ne met pas à jour les feux immédiatement pour éviter les rafales
  // La mise à jour sera gérée par la boucle principale avec le bon timing
}

//==========  REQUÊTE PÉRIODIQUE D'ÉTAT (IRM) ==========

void demanderEtat() {
  // Respecte le délai minimum entre envois CAN
  if (millis() - tLastCanSend >= CAN_SEND_DELAY) {
    struct can_frame frame = { .can_id = ID_IRM_COMMODO | CAN_EFF_FLAG, .can_dlc = 1 };
    frame.data[0] = REG_GPLAT; // Lire l'état des entrées
    mcp2515.sendMessage(&frame);
    tLastPoll = millis();
    tLastCanSend = millis();
  }
}

//==========  SETUP ET BOUCLE PRINCIPALE ==========

void setup() {
  Serial.begin(115200);
  while (!Serial);

  initCAN();
  configCommodo();
  configFeux(ID_IM_FAD);
  configFeux(ID_IM_FAG);
  configFeux(ID_IM_ARD);
  configFeux(ID_IM_ARG);

  Serial.println("\nTP4 VMD - Système prêt (version corrigée pour PicoScope)");
}

void loop() {
  struct can_frame frame;

  // Lecture des messages CAN entrants
  if (mcp2515.readMessage(&frame) == MCP2515::ERROR_OK) {
    recevoirEtatCommodo(frame);
  }

  // Requête périodique d'état
  if (millis() - tLastPoll >= POLLING_INTERVAL) {
    demanderEtat();
  }

  // Mise à jour des feux avec contrôle de timing
  if (millis() - tLastFeuxUpdate >= FEUX_UPDATE_INTERVAL) {
    updateFeux();
    afficherEtat(); // On affiche l'état seulement quand on met à jour les feux
    tLastFeuxUpdate = millis();
  }
}
