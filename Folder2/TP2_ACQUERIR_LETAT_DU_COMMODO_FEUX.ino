// ============================================================= 
// TP2 - Acquisition de l’état des feux via le commodo CAN
// -----------------------------------------------------------------------------
// Objectif : 
//   - Lire les entrées du module commodo (GP0 à GP7).
//   - Afficher sur le moniteur série quels feux/boutons sont actifs.
//   - Envoyer une trame de commande CAN combinée (logique TOR + addition de bits).
//   - Activer un buzzer (GP7) si le klaxon est pressé.
//
// Matériel :
//   - Arduino Uno R3
//   - CAN-BUS Shield
//   - Module commodo VMD (CAN01A)
//   - Buzzer externe sur pin D3
// =============================================================

#include <SPI.h>        // Bibliothèque SPI pour communication avec MCP2515
#include <mcp2515.h>    // Bibliothèque du contrôleur CAN MCP2515

//========== CONFIGURATION MATÉRIELLE / CAN ==========

const int SPI_CS_PIN = 9;   // Broche CS du MCP2515
const int INT_PIN    = 2;   // Broche d’interruption reliée à INT du MCP2515
const int BUZZER_PIN = 3;   // Broche Arduino reliée à un buzzer

MCP2515 mcp2515(SPI_CS_PIN);   // Création de l’objet MCP2515

#define CAN_SPEED   CAN_100KBPS   // Débit CAN = 100 kbps
#define CAN_CLOCK   MCP_16MHZ     // Quartz MCP2515 = 16 MHz

//========== IDENTIFIANTS CAN (format étendu) ==========
// Chaque ID correspond à une trame spécifique du module commodo VMD.

#define ID_IM_COMMODO   0x05081F00  // IM = Input Message (pour envoyer des commandes)
#define ID_AIM_COMMODO  0x05200000  // AIM = Acknowledge Input Message (acquittement)
#define ID_OM_COMMODO   0x05400000  // OM = Output Message (état des entrées/boutons)

//========== REGISTRES MCP25050 (dans le module VMD) ==========

#define REG_GPDDR   0x1F   // GPDDR : registre direction (entrée/sortie)
#define REG_GPLAT   0x1E   // GPLAT : registre des sorties (état GP0-GP7)

//========== MASQUES GP0 à GP7 ==========
// Chaque bit correspond à un bouton ou une fonction du commodo.

#define MASK_VEILLEUSE  (1 << 0) // GP0
#define MASK_WARNING    (1 << 1) // GP1
#define MASK_PHARE      (1 << 2) // GP2
#define MASK_CODE       (1 << 3) // GP3
#define MASK_CLIGN_G    (1 << 4) // GP4
#define MASK_CLIGN_D    (1 << 5) // GP5
#define MASK_STOP       (1 << 6) // GP6
#define MASK_KLAXON     (1 << 7) // GP7

//========== VARIABLES GLOBALES ==========

volatile bool messageRecu = false;  // Flag déclenché par interruption

uint8_t dernierEtatBoutons = 0;     // Dernier état des boutons (pour éviter répétitions)
bool ackPending  = false;           // True si on attend un AIM après un envoi
bool ackPrinted  = false;           // True si AIM affiché une fois

// =================== INITIALISATION CAN =====================

// Initialise le bus CAN (reset, bitrate, mode normal)
void initialiserCAN() {
  SPI.begin();
  while (mcp2515.reset() != MCP2515::ERROR_OK);
  while (mcp2515.setBitrate(CAN_SPEED, CAN_CLOCK) != MCP2515::ERROR_OK);
  while (mcp2515.setNormalMode() != MCP2515::ERROR_OK);
}

// Configure le module commodo : tous les GP0-GP7 en entrée
void configurerModuleCommodoEnEntree() {
  struct can_frame trame;
  trame.can_id  = ID_IM_COMMODO | CAN_EFF_FLAG; // Trame IM (commande)
  trame.can_dlc = 3;                            // 3 octets de données
  trame.data[0] = REG_GPDDR;  // Registre cible = GPDDR (direction)
  trame.data[1] = 0xFF;       // Masque = tous les bits
  trame.data[2] = 0xFF;       // Valeur = 1 = entrée
  mcp2515.sendMessage(&trame); // Envoi de la configuration
}

// =================== TRAITEMENT CAN =========================

// Fonction appelée automatiquement lors d'une interruption MCP2515
void interruptionCAN() {
  messageRecu = true;  // Flag activé → messages à lire
}

// Traite chaque trame CAN reçue
void traiterTrameCAN(struct can_frame trame) {
  uint32_t id = trame.can_id & 0x1FFFFFFF; // Récupère ID en format 29 bits

  // ---- Si trame OM (Output Message) = état des boutons du commodo
  if (id == ID_OM_COMMODO && trame.can_dlc >= 2) {
    uint8_t etatBrut = trame.data[1];  // Donnée brute envoyée
    uint8_t boutons = ~etatBrut;       // Inversion car bits actifs à 0

    // Seulement si l'état a changé
    if (boutons != dernierEtatBoutons) {
      dernierEtatBoutons = boutons;

      // Envoie la commande correspondante aux feux
      envoyerCommandeFeux(boutons);

      // Affiche état détaillé sur le moniteur série
      afficherEtatFeux(boutons);

      // Gestion du buzzer : GP7 = klaxon
      if (boutons & MASK_KLAXON) {
        digitalWrite(BUZZER_PIN, HIGH); // Active buzzer
      } else {
        digitalWrite(BUZZER_PIN, LOW);  // Désactive buzzer
      }
    }
  }

  // ---- Si trame AIM (acquittement après une commande envoyée)
  else if (id == ID_AIM_COMMODO && ackPending && !ackPrinted) {
    Serial.print("[RX] AIM reçu : 0x");
    Serial.print(trame.data[2], HEX);
    Serial.print(" (0b"); Serial.print(trame.data[2], BIN); Serial.println(")\n");
    Serial.println("--------------------------------------------------");
    ackPending = false;  // Acquittement reçu
    ackPrinted = true;   // Marqué comme affiché
  }
}

// Envoie une commande CAN aux feux en fonction des boutons pressés
void envoyerCommandeFeux(uint8_t valeur) {
  struct can_frame trame;
  trame.can_id  = ID_IM_COMMODO | CAN_EFF_FLAG;
  trame.can_dlc = 3;
  trame.data[0] = REG_GPLAT; // On agit sur GPLAT (sorties)
  trame.data[1] = 0xFF;      // Tous les bits pris en compte
  trame.data[2] = valeur;    // Valeur à appliquer

  mcp2515.sendMessage(&trame); // Envoi de la trame

  Serial.print("[TX] Feux activés : 0x"); Serial.print(valeur, HEX);
  Serial.print(" (0b"); Serial.print(valeur, BIN); Serial.println(")");
  ackPending = true;   // On attend un AIM
  ackPrinted = false;  // Pas encore affiché
}

// ================= AFFICHAGE SERIAL =========================

// Affiche l’état détaillé des feux et boutons
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
  while (!Serial);  // Attente ouverture moniteur série

  // Active interruption matérielle sur INT du MCP2515
  pinMode(INT_PIN, INPUT);
  attachInterrupt(digitalPinToInterrupt(INT_PIN), interruptionCAN, FALLING);

  // Prépare la sortie buzzer
  pinMode(BUZZER_PIN, OUTPUT);
  digitalWrite(BUZZER_PIN, LOW); // Buzzer OFF au démarrage

  // Initialise le bus CAN et le commodo
  initialiserCAN();
  configurerModuleCommodoEnEntree();

  Serial.println("\n[SYSTÈME PRÊT - Interruption active]");
}

void loop() {
  // Si interruption signalée → lire les trames
  if (messageRecu) {
    messageRecu = false;

    struct can_frame trame;
    // Lecture en boucle de toutes les trames présentes
    while (mcp2515.readMessage(&trame) == MCP2515::ERROR_OK) {
      traiterTrameCAN(trame);
    }
  }
}
