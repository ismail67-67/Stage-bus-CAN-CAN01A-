/* 
 * Programme de contrôle des feux automobile via CAN
 * TP3 - Véhicule Multiplexé Didactique
 * 
 * Fonctionnalités :
 * - Cycle automatique des états des feux (3.5s)
 * - Gestion des clignotants (1.6s)
 * - Communication CAN avec les modules MCP25050
 * - Lecture des états du commodo
 * - Affichage debug des trames CAN
 */

// 1. INCLUSIONS ET CONFIGURATION
//--------------------------------------------------------------------------------
#include <SPI.h>
#include <mcp2515.h>

// Configuration matérielle
const int SPI_CS_PIN = 9;  // Broche CS pour le module CAN
MCP2515 mcp2515(SPI_CS_PIN);

// Configuration CAN
#define CAN_SPEED CAN_100KBPS  // Vitesse du bus CAN
#define CAN_CLOCK MCP_16MHZ    // Fréquence du quartz
#define DEBUG_MODE 1           // Activer les messages de debug

// 2. DÉFINITIONS DES IDENTIFIANTS CAN
//--------------------------------------------------------------------------------
// Identifiants pour le module COMMODO
#define ID_IM_COMMODO   0x05080000  // Input Message
#define ID_IRM_COMMODO  0x05041E07  // Information Request Message
#define ID_AIM_COMMODO  0x05200000  // Acknowledge Input Message

// Identifiants pour le module FVD (Feux Avant Droit)
#define ID_IM_FVD   0x0E880000  // Input Message
#define ID_AIM_FVD  0x0EA00000  // Acknowledge Input Message  
#define ID_IRM_FVD  0x0E841E07  // Information Request Message

// 3. DÉFINITIONS DES REGISTRES MCP25050
//--------------------------------------------------------------------------------
#define REG_GPDDR  0x1F  // Registre de direction des broches (I/O)
#define REG_GPLAT  0x1E  // Registre des états de sortie
#define REG_GPPIN  0x1E  // Registre de lecture des entrées (équivalent à GPLAT)

// 4. MASQUES DE CONFIGURATION
//--------------------------------------------------------------------------------
#define MASK_COMMODO 0x3F  // Masque pour le COMMODO (bits 0-5)
#define MASK_FVD     0x0F  // Masque pour le FVD (bits 0-3)

// 5. PARAMÈTRES DE TEMPORISATION (ms)
//--------------------------------------------------------------------------------
#define TEMPO_FEUX 3500     // 3.5s - Changement d'état des feux
#define TEMPO_CLIGNOT 1600  // 1.6s - Période de clignotement
#define TEMPO_LECTURE 1000  // 1.0s - Période de lecture des états

// 6. STRUCTURES DE DONNÉES
//--------------------------------------------------------------------------------
// Structure pour les commandes et états des feux
struct {
  // Commandes (sorties)
  uint8_t veilleuse : 1;    // GP0
  uint8_t warning : 1;      // GP1
  uint8_t phare : 1;        // GP2
  uint8_t code : 1;         // GP3
  uint8_t clign_g : 1;      // GP4
  uint8_t clign_d : 1;      // GP5
  
  // États lus (entrées)
  uint8_t status_veilleuse : 1;
  uint8_t status_warning : 1;
  uint8_t status_phare : 1;
  uint8_t status_code : 1;
  uint8_t status_clign_g : 1;
  uint8_t status_clign_d : 1;
} commandes, etat;

// Variables de temporisation
unsigned long dernier_changement = 0;
unsigned long dernier_clignotement = 0;
unsigned long dernier_lecture = 0;
bool clignotant_actif = false;

// 7. FONCTIONS D'AIDE ET DEBUG
//--------------------------------------------------------------------------------
/**
 * Affiche un message de debug sur le port série
 * @param message Le message à afficher
 */
void printDebug(String message) {
  #if DEBUG_MODE
    Serial.println("[DEBUG] " + message);
  #endif
}

/**
 * Affiche le contenu d'une trame CAN
 * @param frame La trame CAN à afficher
 * @param isReceived True si trame reçue, false si émise
 */
void printFrame(const can_frame &frame, bool isReceived) {
  Serial.print(isReceived ? "[RX] " : "[TX] ");
  Serial.print("ID: 0x");
  Serial.print(frame.can_id & 0x1FFFFFFF, HEX);
  Serial.print(" DLC: ");
  Serial.print(frame.can_dlc);
  Serial.print(" Data: ");
  for (int i = 0; i < frame.can_dlc; i++) {
    Serial.print("0x");
    Serial.print(frame.data[i], HEX);
    Serial.print(" ");
  }
  Serial.println();
}

/**
 * Affiche l'état courant des feux
 */
void printFeuxStatus() {
  Serial.println("\n=== ETAT DES FEUX ===");
  Serial.print("FVD - Veilleuse:");
  Serial.print(commandes.veilleuse);
  Serial.print(" Code:");
  Serial.print(commandes.code);
  Serial.print(" Phare:");
  Serial.print(commandes.phare);
  Serial.print(" Cligno D:");
  Serial.println(commandes.clign_d);
}

// 8. FONCTIONS DE COMMUNICATION CAN
//--------------------------------------------------------------------------------
/**
 * Initialise la communication CAN
 * @return True si succès, false sinon
 */
bool initCAN() {
  if(mcp2515.reset() != MCP2515::ERROR_OK) {
    printDebug("Erreur reset CAN");
    return false;
  }
  
  if(mcp2515.setBitrate(CAN_SPEED, CAN_CLOCK) != MCP2515::ERROR_OK) {
    printDebug("Erreur bitrate CAN");
    return false;
  }
  
  if(mcp2515.setNormalMode() != MCP2515::ERROR_OK) {
    printDebug("Erreur mode normal CAN");
    return false;
  }
  
  printDebug("CAN initialise");
  return true;
}

/**
 * Envoie un message CAN
 * @param id Identifiant CAN
 * @param reg Registre cible
 * @param mask Masque à appliquer
 * @param val Valeur à écrire
 * @return True si succès, false sinon
 */
bool sendMessage(uint32_t id, uint8_t reg, uint8_t mask, uint8_t val) {
  struct can_frame msg;
  msg.can_id = id | CAN_EFF_FLAG;  // Mode étendu
  msg.can_dlc = 3;                 // 3 octets de données
  msg.data[0] = reg;               // Adresse registre
  msg.data[1] = mask;              // Masque
  msg.data[2] = val;               // Valeur

  printFrame(msg, false);
  
  if(mcp2515.sendMessage(&msg) != MCP2515::ERROR_OK) {
    printDebug("Echec envoi message ID: 0x" + String(id, HEX));
    return false;
  }
  return true;
}

/**
 * Lit un message CAN avec timeout
 * @param expected_id Identifiant attendu
 * @param data Buffer pour les données
 * @param timeout Délai d'attente (ms)
 * @return True si message valide reçu, false sinon
 */
bool readMessage(uint32_t expected_id, uint8_t* data, uint16_t timeout = 500) {
  struct can_frame frame;
  unsigned long t0 = millis();
  
  while(millis() - t0 < timeout) {
    if(mcp2515.readMessage(&frame) == MCP2515::ERROR_OK) {
      printFrame(frame, true);
      if((frame.can_id & 0x1FFFFFFF) == expected_id) {
        memcpy(data, frame.data, frame.can_dlc);
        return true;
      }
    }
  }
  return false;
}

// 9. CONFIGURATION DES MODULES
//--------------------------------------------------------------------------------
/**
 * Configure un module CAN
 * @param im_id ID pour Input Message
 * @param aim_id ID pour Acknowledge
 * @param mask Masque à appliquer
 * @param init_val Valeur initiale
 * @param is_output True pour sortie, false pour entrée
 * @return True si succès, false sinon
 */
bool configModule(uint32_t im_id, uint32_t aim_id, uint8_t mask, uint8_t init_val, bool is_output) {
  // Configuration GPDDR (direction des broches)
  uint8_t gpddr_val = is_output ? 0x00 : 0xFF;
  
  if(!sendMessage(im_id, REG_GPDDR, mask, gpddr_val)) {
    printDebug("Erreur config GPDDR");
    return false;
  }
  
  // Attendre ACK
  uint8_t ack_data[8];
  if(!readMessage(aim_id, ack_data)) {
    printDebug("Timeout ACK config GPDDR");
    return false;
  }
  
  // Initialisation GPLAT (état des sorties)
  if(is_output && !sendMessage(im_id, REG_GPLAT, mask, init_val)) {
    printDebug("Erreur config GPLAT");
    return false;
  }
  
  return true;
}

// 10. GESTION DES ÉTATS DES FEUX
//--------------------------------------------------------------------------------
/**
 * Met à jour l'état des feux selon le cycle prédéfini
 */
void updateFeux() {
  static uint8_t cycle = 0;
  const char* etats[] = {"Tout eteint", "Veilleuse", "Veilleuse+Code", "Veilleuse+Phare"};
  
  Serial.print("Etat suivant des feux: ");
  Serial.println(etats[cycle]);
  
  // Cycle pour FVD (3.5s entre chaque changement)
  switch(cycle) {
    case 0: // Tout éteint
      commandes.veilleuse = 0;
      commandes.code = 0;
      commandes.phare = 0;
      cycle = 1;
      break;
    case 1: // Veilleuse
      commandes.veilleuse = 1;
      cycle = 2;
      break;
    case 2: // Veilleuse + Code
      commandes.code = 1;
      cycle = 3;
      break;
    case 3: // Veilleuse + Phare
      commandes.code = 0;
      commandes.phare = 1;
      cycle = 0;
      break;
  }
  
  // Envoi commande FVD (NON inversé)
  uint8_t val_fvd = (commandes.clign_d << 3) | 
                   (commandes.phare << 2) | 
                   (commandes.code << 1) | 
                   commandes.veilleuse;
  
  if(!sendMessage(ID_IM_FVD, REG_GPLAT, MASK_FVD, val_fvd)) {
    printDebug("Echec commande FVD");
  }
  
  // Envoi commande COMMODO (INVERSÉ)
  uint8_t val_commodo = ((!commandes.clign_d) << 5) | 
                       ((!commandes.clign_g) << 4) |
                       ((!commandes.code) << 3) | 
                       ((!commandes.phare) << 2) | 
                       ((!commandes.warning) << 1) | 
                       (!commandes.veilleuse);
  
  if(!sendMessage(ID_IM_COMMODO, REG_GPLAT, MASK_COMMODO, val_commodo)) {
    printDebug("Echec commande COMMODO");
  }
  
  // Lecture état du commodo après commande
  readCommodoStatus();
  printFeuxStatus();
}

/**
 * Lit l'état actuel du commodo
 * @return True si succès, false sinon
 */
bool readCommodoStatus() {
  // Envoi demande de lecture du registre GPPIN du COMMODO
  struct can_frame msg;
  msg.can_id = ID_IRM_COMMODO | CAN_EFF_FLAG;
  msg.can_dlc = 1;
  msg.data[0] = REG_GPPIN;
  
  if(mcp2515.sendMessage(&msg) != MCP2515::ERROR_OK) {
    printDebug("Echec demande lecture COMMODO");
    return false;
  }
  
  return true;
}

// 11. SETUP ET LOOP PRINCIPALE
//--------------------------------------------------------------------------------
void setup() {
  // Initialisation port série
  Serial.begin(115200);
  while(!Serial); // Attendre connexion série
  
  // Initialisation SPI
  SPI.begin();
  
  // Initialisation CAN
  if(!initCAN()) {
    Serial.println("ERREUR INIT CAN");
    while(1);
  }
  
  // Configuration COMMODO (sorties pour GP0-GP5)
  if(!configModule(ID_IM_COMMODO, ID_AIM_COMMODO, MASK_COMMODO, 0x3F, true)) {
    Serial.println("ERREUR CONFIG COMMODO");
  }
  
  // Configuration FVD (sorties)
  if(!configModule(ID_IM_FVD, ID_AIM_FVD, MASK_FVD, 0x00, true)) {
    Serial.println("ERREUR CONFIG FVD");
  }
  
  // Initialisation des commandes (tout éteint)
  commandes.veilleuse = 0;
  commandes.warning = 0;
  commandes.phare = 0;
  commandes.code = 0;
  commandes.clign_g = 0;
  commandes.clign_d = 0;
  
  Serial.println("\nSysteme pret");
  Serial.println("Configuration initiale:");
  printFeuxStatus();
}

void loop() {
  unsigned long maintenant = millis();
  
  // Changement d'état des feux (toutes les 3.5s)
  if(maintenant - dernier_changement >= TEMPO_FEUX) {
    dernier_changement = maintenant;
    updateFeux();
  }
  
  // Clignotant (toutes les 1.6s)
  if(maintenant - dernier_clignotement >= TEMPO_CLIGNOT) {
    dernier_clignotement = maintenant;
    clignotant_actif = !clignotant_actif;
    commandes.clign_d = clignotant_actif ? 1 : 0;
    
    // Envoi immédiat de la commande clignotant
    // FVD (NON inversé)
    uint8_t val_fvd = (commandes.clign_d << 3) | 
                     (commandes.phare << 2) | 
                     (commandes.code << 1) | 
                     commandes.veilleuse;
    
    if(!sendMessage(ID_IM_FVD, REG_GPLAT, MASK_FVD, val_fvd)) {
      printDebug("Echec commande clignotant FVD");
    }
    
    // COMMODO (INVERSÉ)
    uint8_t val_commodo = ((!commandes.clign_d) << 5) | 
                         ((!commandes.clign_g) << 4) |
                         ((!commandes.code) << 3) | 
                         ((!commandes.phare) << 2) | 
                         ((!commandes.warning) << 1) | 
                         (!commandes.veilleuse);
    
    if(!sendMessage(ID_IM_COMMODO, REG_GPLAT, MASK_COMMODO, val_commodo)) {
      printDebug("Echec commande clignotant COMMODO");
    }
    
    // Lecture état après commande
    readCommodoStatus();
    printFeuxStatus();
  }
  
  delay(100);  // Petite pause pour éviter la surcharge CPU
}