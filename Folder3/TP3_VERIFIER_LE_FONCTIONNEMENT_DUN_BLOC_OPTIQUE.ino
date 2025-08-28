/******************************************************************************
 * TP3 – Contrôle cyclique d’un bloc optique via CAN
 * --------------------------------------------------------------------------
 * Séquence : 
 *   - Clignotant droit indépendant (toggle toutes les 1.6s)
 *   - Feux en séquence cyclique toutes les 3.5s :
 *       -> Étape 0 : Tous éteints
 *       -> Étape 1 : Veilleuse seule
 *       -> Étape 2 : Veilleuse + Code
 *       -> Étape 3 : Veilleuse + Phare
 *   - Le tout s’exécute en parallèle (clignotant + séquence feux)
 * 
 * Points clés :
 *   - Utilisation de l’interruption matérielle.
 *   - Attente d’ACK (trame AIM) envoyée par le bloc optique après réception
 *   - Pas de polling bloquant → utilisation de flags d’interruption
 * 
 * Matériel :
 *   - Arduino Uno R3
 *   - Shield CAN MCP2515 (CS=9, INT=2)
 *   - Bloc optique VMD (exemple : Feux Avant Droit)
 ******************************************************************************/

#include <SPI.h>
#include <mcp2515.h>

//========== CONFIGURATION GÉNÉRALE ==========

#define SPI_CS_PIN      9           // Broche CS pour MCP2515
#define INT_PIN         2           // Broche d'interruption reliée à INT du MCP2515
#define CAN_SPEED       CAN_100KBPS // Débit CAN
#define CAN_CLOCK       MCP_16MHZ   // Quartz MCP2515 = 16 MHz

// --- Identité du bloc contrôlé ---
#define NOM_BLOC        "FAD"       // Nom affiché (Feux Avant Droit)
#define ID_IM_BLOC      0x0E880000  // ID trame IM  : Input Message → envoi commande au bloc
#define ID_AIM_BLOC     0x0EA00000  // ID trame AIM : Acknowledge Input Message ← réponse du bloc

// --- Registres internes du MCP25050 (module du bloc optique) ---
#define REG_GPDDR       0x1F        // GPDDR : configuration direction (entrée/sortie)
#define REG_GPLAT       0x1E        // GPLAT : registre de latence → état logique des GPIO

#define MASK_BLOC       0x0F        // Masque de GP0 à GP3 (4 sorties actives : veilleuse, code, phare, clignotant)

// --- Temporisations ---
#define TEMPO_FEUX      3500        // Changement d’état des feux toutes les 3.5 s
#define TEMPO_CLIGNOT   1600        // Toggle du clignotant droit toutes les 1.6 s

MCP2515 mcp2515(SPI_CS_PIN);       // Instance contrôleur CAN

//========== STRUCTURE POUR LES COMMANDES FEUX ==========
// Utilisation de "bitfields" → 1 bit par variable
struct {
  uint8_t veilleuse : 1;  // GP0
  uint8_t code      : 1;  // GP1
  uint8_t phare     : 1;  // GP2
  uint8_t clign_d   : 1;  // GP3
} commandes;

unsigned long t_prev_feux    = 0;   // Horodatage précédent pour séquence feux
unsigned long t_prev_cligno  = 0;   // Horodatage précédent pour clignotant
bool clignotant_on = false;         // État courant du clignotant (ON/OFF)

// Flags d’interruption et d’ACK
volatile bool ackRecu       = false;   // Vrai si un AIM est reçu
volatile bool attenteAck    = false;   // Indique qu’on attend un ACK suite à un envoi

//========== INITIALISATION DU BUS CAN ==========

bool initialiserCAN() {
  // Reset du MCP2515
  if (mcp2515.reset() != MCP2515::ERROR_OK) return false;
  // Configuration vitesse CAN
  if (mcp2515.setBitrate(CAN_SPEED, CAN_CLOCK) != MCP2515::ERROR_OK) return false;
  // Passage en mode normal (communication active sur le bus)
  if (mcp2515.setNormalMode() != MCP2515::ERROR_OK) return false;
  return true;
}

// Configuration des GPIO du bloc optique en sortie (GP0–GP3)
bool configurerSortieGPIO() {
  struct can_frame frame;
  frame.can_id  = ID_IM_BLOC | CAN_EFF_FLAG; // ID trame IM + indicateur "extended frame"
  frame.can_dlc = 3;
  frame.data[0] = REG_GPDDR;     // GPDDR → registre de direction
  frame.data[1] = MASK_BLOC;     // On cible GP0 à GP3
  frame.data[2] = 0x00;          // 0 = sorties
  return (mcp2515.sendMessage(&frame) == MCP2515::ERROR_OK);
}

//========== AFFICHAGE DÉTAILLÉ SUR MONITEUR ==========

void afficherEtatFeux() {
  Serial.println("\n[ÉTAT DES FEUX - " NOM_BLOC "]");
  Serial.print("  Veilleuse : "); Serial.println(commandes.veilleuse ? "ON" : "OFF");
  Serial.print("  Code      : "); Serial.println(commandes.code ? "ON" : "OFF");
  Serial.print("  Phare     : "); Serial.println(commandes.phare ? "ON" : "OFF");
  Serial.print("  Clignotant: "); Serial.println(commandes.clign_d ? "ON" : "OFF");

  // Calcul de la valeur binaire (GP3..GP0)
  uint8_t val = (commandes.clign_d << 3) | (commandes.phare << 2) |
                (commandes.code << 1) | commandes.veilleuse;

  // Affichage en binaire et hex
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
  frame.data[0] = REG_GPLAT;  // Écriture dans GPLAT (sorties)
  frame.data[1] = MASK_BLOC;  // GP0–GP3
  frame.data[2] = val;        // Nouvelle valeur

  // Réinitialisation flags d’attente d’ACK
  ackRecu = false;
  attenteAck = true;

  // Envoi trame CAN
  if (mcp2515.sendMessage(&frame) != MCP2515::ERROR_OK) {
    Serial.println("[ERREUR] Échec d’envoi CAN");
    return false;
  }

  // Attente d’un ACK (max 500ms)
  unsigned long t0 = millis();
  while (attenteAck && (millis() - t0 < 500));

  if (ackRecu) return true;

  Serial.println("[TIMEOUT] Aucun ACK reçu");
  return false;
}

//========== ROUTINE D’INTERRUPTION CAN ==========

void interruptionCAN() {
  struct can_frame frame;
  // On lit toutes les trames disponibles
  while (mcp2515.readMessage(&frame) == MCP2515::ERROR_OK) {
    uint32_t id = frame.can_id & 0x1FFFFFFF; // Récupération ID sans drapeaux
    // Si c’est bien une trame AIM du bloc
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
  static uint8_t etape = 0; // Étape courante (0 → 3)

  // Réinitialisation des feux (clignotant conservé)
  commandes = {0, 0, 0, commandes.clign_d};

  switch (etape) {
    case 1: commandes.veilleuse = 1; break;                   // Étape 1 : veilleuse seule
    case 2: commandes.veilleuse = commandes.code = 1; break;  // Étape 2 : veilleuse + code
    case 3: commandes.veilleuse = commandes.phare = 1; break; // Étape 3 : veilleuse + phare
    default: break;                                           // Étape 0 : tout OFF
  }

  etape = (etape + 1) % 4; // Incrément cyclique

  // Conversion en valeur binaire et envoi
  uint8_t val = (commandes.clign_d << 3) | (commandes.phare << 2) |
                (commandes.code << 1) | commandes.veilleuse;

  envoyerCommande(val);
  afficherEtatFeux();
}

//========== CLIGNOTANT DROIT (AUTONOME) ==========

void miseAJourClignotant() {
  clignotant_on = !clignotant_on;     // Inversion d’état
  commandes.clign_d = clignotant_on;  // Mise à jour structure

  uint8_t val = (commandes.clign_d << 3) | (commandes.phare << 2) |
                (commandes.code << 1) | commandes.veilleuse;

  envoyerCommande(val);
  afficherEtatFeux();
}

//========== SETUP PRINCIPAL ==========

void setup() {
  Serial.begin(115200);
  while (!Serial);      // Attente ouverture terminal
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

  commandes = {0, 0, 0, 0}; // Tout éteint au départ
  Serial.println("[PRÊT] Système opérationnel pour le bloc " NOM_BLOC);
}

//========== LOOP PRINCIPALE ==========

void loop() {
  unsigned long now = millis();

  // Gestion de la séquence feux (toutes les 3.5s)
  if (now - t_prev_feux >= TEMPO_FEUX) {
    t_prev_feux = now;
    miseAJourFeux();
  }

  // Gestion du clignotant droit (toggle toutes les 1.6s)
  if (now - t_prev_cligno >= TEMPO_CLIGNOT) {
    t_prev_cligno = now;
    miseAJourClignotant();
  }
}
