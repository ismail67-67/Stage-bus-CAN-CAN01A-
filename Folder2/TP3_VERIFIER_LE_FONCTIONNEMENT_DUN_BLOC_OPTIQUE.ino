/* ============================================================
 * TP2 - ACQUISITION ETAT COMMODO FEUX VIA CAN BUS
 * ------------------------------------------------------------
 * Programme optimisé pour lire l'état du commodo feux via CAN
 * et contrôler les LEDs en fonction des entrées
 * 
 * Structure:
 * 1. Configuration matérielle
 * 2. Définitions et constantes
 * 3. Structures de données
 * 4. Fonctions CAN Bus
 * 5. Gestion des LEDs
 * 6. Fonctions principales
 * ============================================================
 */

/**************************************************************
 * 1. CONFIGURATION MATERIELLE
 **************************************************************/
#include <SPI.h>
#include <mcp2515.h>

const int SPI_CS_PIN = 9;          // Broche CS du MCP2515
MCP2515 mcp2515(SPI_CS_PIN);       // Objet CAN

/**************************************************************
 * 2. DEFINITIONS ET CONSTANTES
 **************************************************************/

/* --------------------------
 * Paramètres CAN
 * ------------------------ */
#define CAN_SPEED CAN_100KBPS      // Vitesse bus CAN
#define CAN_CLOCK MCP_16MHZ        // Horloge MCP2515

/* --------------------------
 * Identifiants CAN (MCP25050)
 * ------------------------ */
#define ID_IM_COMMODO   0x05081F00 // Input Message (Configuration)
#define ID_AIM_COMMODO  0x05200000 // Acquittement IM
#define ID_IRM_COMMODO  0x05041E07 // Information Request Message
#define ID_OM_COMMODO   0x05400000 // Output Message (Automatique)

/* --------------------------
 * Broches LEDs
 * ------------------------ */
#define LED_CLIGN_G 11             // D11 - Clignotant gauche
#define LED_CLIGN_D 12             // D12 - Clignotant droit
#define LED_STOP    13             // D13 - Stop
#define LED_KLAXON  14             // D14 - Klaxon

/* --------------------------
 * Registres MCP25050
 * ------------------------ */
#define REG_GPDDR  0x1F            // Registre direction I/O
#define REG_GPLAT  0x1E            // Registre état des ports
#define REG_IOTEN  0x1C            // Registre interruptions

/* --------------------------
 * Masques des entrées/sorties
 * ------------------------ */
#define MASK_CLIGN_G  (1 << 4)     // GP4 - Clignotant gauche (B1)
#define MASK_CLIGN_D  (1 << 5)     // GP5 - Clignotant droit (B2)
#define MASK_STOP     (1 << 6)     // GP6 - Stop
#define MASK_KLAXON   (1 << 7)     // GP7 - Klaxon
#define MASK_D8       (1 << 1)     // GP1 - LED D8 (sortie)

/* --------------------------
 * Paramètres temporisation
 * ------------------------ */
#define POLLING_INTERVAL  200      // Intervalle interrogation (ms)
#define BLINK_INTERVAL    500      // Intervalle clignotement (ms)
#define DEBOUNCE_DELAY    50       // Anti-rebonds (ms)

/**************************************************************
 * 3. STRUCTURES DE DONNEES
 **************************************************************/

/**
 * @brief Structure pour stocker l'état du commodo
 */
typedef struct {
    bool clignGauche;              // Etat clignotant gauche (B1 pressé)
    bool clignDroit;               // Etat clignotant droit (B2 pressé)
    bool stop;                     // Etat feu stop
    bool klaxon;                   // Etat klaxon
    bool warning;                  // Mode warning (B1 ET B2 pressés)
    bool ledD8;                    // Etat LED D8
    unsigned long dernierPoll;     // Dernier polling
    unsigned long dernierClignotement; // Timer clignotement
} CommodoState;

/**************************************************************
 * 4. VARIABLES GLOBALES
 **************************************************************/
CommodoState etatCommando = {false, false, false, false, false, false, 0, 0};
bool etatClignotement = false;     // Etat courant clignotement

/**************************************************************
 * 5. FONCTIONS CAN BUS
 **************************************************************/

/**
 * @brief Initialise le module CAN
 */
void initCAN() {
    SPI.begin();
    
    if(mcp2515.reset() != MCP2515::ERROR_OK) {
        Serial.println("[ERREUR] Reset CAN échoué");
        while(1);
    }
    
    if(mcp2515.setBitrate(CAN_SPEED, CAN_CLOCK) != MCP2515::ERROR_OK) {
        Serial.println("[ERREUR] Configuration bitrate");
        while(1);
    }
    
    if(mcp2515.setNormalMode() != MCP2515::ERROR_OK) {
        Serial.println("[ERREUR] Activation mode normal");
        while(1);
    }
}

/**
 * @brief Configure le module MCP25050
 */
void configurerModule() {
    struct can_frame trame;
    
    // Configuration des directions I/O (GP1 en sortie, autres en entrée)
    trame.can_id = ID_IM_COMMODO | CAN_EFF_FLAG;
    trame.can_dlc = 3;
    trame.data[0] = REG_GPDDR;     // Registre direction
    trame.data[1] = 0xFF;          // Masque tous bits
    trame.data[2] = 0xFF & ~MASK_D8; // Tous en entrée sauf GP1
    
    if(mcp2515.sendMessage(&trame) != MCP2515::ERROR_OK) {
        Serial.println("[ERREUR] Envoi configuration I/O");
        while(1);
    }
    
    // Initialisation LED D8 éteinte (logique inversée)
    trame.data[0] = REG_GPLAT;     // Registre état
    trame.data[1] = MASK_D8;       // Masque pour GP1
    trame.data[2] = MASK_D8;       // GP1 à 1 (LED éteinte - logique inversée)
    
    if(mcp2515.sendMessage(&trame) != MCP2515::ERROR_OK) {
        Serial.println("[ERREUR] Init LED D8");
        while(1);
    }
}

/**
 * @brief Contrôle la LED D8 sur le module
 * @param etat true pour allumer, false pour éteindre
 */
void controlerLEDD8(bool etat) {
    struct can_frame trame;
    
    trame.can_id = ID_IM_COMMODO | CAN_EFF_FLAG;
    trame.can_dlc = 3;
    trame.data[0] = REG_GPLAT;     // Registre état
    trame.data[1] = MASK_D8;       // Masque pour GP1
    // Logique inversée: 0 pour allumer, 1 pour éteindre
    trame.data[2] = etat ? 0x00 : MASK_D8;
    
    if(mcp2515.sendMessage(&trame) != MCP2515::ERROR_OK) {
        Serial.println("[ERREUR] Contrôle LED D8");
    }
    
    etatCommando.ledD8 = etat;  // Mise à jour état interne
}

/**
 * @brief Demande l'état actuel du commodo
 */
void demanderEtat() {
    struct can_frame trame;
    trame.can_id = ID_IRM_COMMODO | CAN_EFF_FLAG;
    trame.can_dlc = 1;
    trame.data[0] = REG_GPLAT;
    
    if(mcp2515.sendMessage(&trame) == MCP2515::ERROR_OK) {
        etatCommando.dernierPoll = millis();
    } else {
        Serial.println("[ERREUR] Envoi demande etat");
    }
}

/**
 * @brief Traite un message CAN reçu
 * @param trame Message CAN à traiter
 * @return true si message valide traité
 */
bool traiterMessageCAN(can_frame trame) {
    if((trame.can_id & 0x1FFFFFFF) == ID_OM_COMMODO && trame.can_dlc >= 2) {
        // Lecture état des entrées (0 = bouton pressé, 1 = relâché)
        bool b1Presse = !(trame.data[1] & MASK_CLIGN_G);
        bool b2Presse = !(trame.data[1] & MASK_CLIGN_D);
        
        // Mise à jour de l'état
        etatCommando.clignGauche = b1Presse;
        etatCommando.clignDroit = b2Presse;
        etatCommando.stop = !(trame.data[1] & MASK_STOP);
        etatCommando.klaxon = !(trame.data[1] & MASK_KLAXON);
        
        // Détection mode warning (les DEUX boutons pressés)
        etatCommando.warning = b1Presse && b2Presse;
        
        return true;
    }
    return false;
}

/**************************************************************
 * 6. GESTION DES LEDS
 **************************************************************/

/**
 * @brief Met à jour l'état des LEDs en fonction de l'état du commodo
 */
void actualiserLEDs() {
    // Gestion timer clignotement
    if(millis() - etatCommando.dernierClignotement > BLINK_INTERVAL) {
        etatClignotement = !etatClignotement;
        etatCommando.dernierClignotement = millis();
    }
    
    // Mode warning (les deux boutons pressés)
    if(etatCommando.warning) {
        digitalWrite(LED_CLIGN_G, etatClignotement);
        digitalWrite(LED_CLIGN_D, etatClignotement);
        
        // Allumer LED D8 seulement en mode warning
        if(!etatCommando.ledD8) {
            controlerLEDD8(true);
        }
    } else {
        // Clignotants normaux
        digitalWrite(LED_CLIGN_G, etatCommando.clignGauche ? etatClignotement : LOW);
        digitalWrite(LED_CLIGN_D, etatCommando.clignDroit ? etatClignotement : LOW);
        
        // Eteindre LED D8 si pas en mode warning
        if(etatCommando.ledD8) {
            controlerLEDD8(false);
        }
    }
    
    // Feux stop et klaxon (pas de clignotement)
    digitalWrite(LED_STOP, etatCommando.stop ? HIGH : LOW);
    digitalWrite(LED_KLAXON, etatCommando.klaxon ? HIGH : LOW);
}

/**
 * @brief Affiche l'état courant du commodo sur le port série
 */
void afficherEtat() {
    Serial.println("\n==== ETAT FEUX COMMODO ====");
    Serial.print("Clignotant Gauche: "); Serial.println(etatCommando.clignGauche ? "ACTIF" : "inactif");
    Serial.print("Clignotant Droit:  "); Serial.println(etatCommando.clignDroit ? "ACTIF" : "inactif");
    Serial.print("Stop:              "); Serial.println(etatCommando.stop ? "ACTIF" : "inactif");
    Serial.print("Klaxon:            "); Serial.println(etatCommando.klaxon ? "ACTIF" : "inactif");
    Serial.print("Warning (Détresse):"); Serial.println(etatCommando.warning ? "ACTIF" : "inactif");
    Serial.println("===========================");
}

/**************************************************************
 * 7. FONCTIONS PRINCIPALES
 **************************************************************/

void setup() {
    // Configuration des broches LEDs
    pinMode(LED_CLIGN_G, OUTPUT);
    pinMode(LED_CLIGN_D, OUTPUT);
    pinMode(LED_STOP, OUTPUT);
    pinMode(LED_KLAXON, OUTPUT);
    
    // Initialisation port série
    Serial.begin(115200);
    while(!Serial); // Attente connexion pour les cartes avec USB
    
    Serial.println("\nInitialisation système...");
    
    // Initialisation CAN
    initCAN();
    configurerModule();
    
    // Première demande d'état
    demanderEtat();
    
    Serial.println("Système prêt");
}

void loop() {
    struct can_frame trame;
    
    // Lecture des messages CAN
    if(mcp2515.readMessage(&trame) == MCP2515::ERROR_OK) {
        if(traiterMessageCAN(trame)) {
            afficherEtat();
        }
    }
    
    // Polling périodique de l'état
    if(millis() - etatCommando.dernierPoll >= POLLING_INTERVAL) {
        demanderEtat();
    }
    
    // Mise à jour des LEDs
    actualiserLEDs();
}