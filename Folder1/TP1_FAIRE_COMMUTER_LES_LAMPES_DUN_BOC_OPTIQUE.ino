/*******************************************************************************
 *  TP1 - Chenillard VMD avec Arduino Uno R3 et MCP2515
 *  
 *  Description: 
 *    Ce programme implémente un chenillard sur les feux avant droit d'un véhicule
 *    didactique utilisant le bus CAN. Il suit les spécifications du TP1 du manuel
 *    VMD.
 *  
 *  Fonctionnement:
 *    - Séquence cyclique: Veilleuse -> Code -> Phare -> Clignotant droit -> Eteint
 *    - Communication CAN avec le module MCP25050
 *    - Affichage détaillé sur le moniteur série
 *  
 *  Modules compatibles : Feux avant droit (par défaut), modifiable pour
 *                       Feux avant gauche, arrière droit ou arrière gauche.

 *******************************************************************************/

#include <SPI.h>
#include "mcp2515.h"

// ============================================================================
// === CONFIGURATION MATÉRIELLE ==============================================
// ============================================================================

const int SPI_CS_PIN = 9;              // Broche CS reliée au MCP2515
MCP2515 mcp2515(SPI_CS_PIN);           // Objet pour contrôler le MCP2515

// ============================================================================
// === PARAMÈTRES CAN & REGISTRES MCP25050 ===================================
// ============================================================================

#define ID_IM   0x0E080000              // ID de trame IM pour le module (par défaut : feux avant droit) {RX}
#define ID_AIM  0x0E200000              // ID de trame AIM (acquittement) {TX}
#define REG_GPDDR 0x1F                  // Registre : direction des ports
#define REG_GPLAT 0x1E                  // Registre : sortie des ports
#define MASQUE_SORTIES 0x0F             // GP0 à GP3 en sortie (bits actifs)

// Pour utiliser d'autres modules VMD (changer ID_IM et ID_AIM si besoin) :
//  Avant droit   : RX=0x0E880000 TX=0x0EA00000
//  Avant gauche  : RX=0x0E080000 TX=0x0E200000
//  Arrière droit : RX=0x0F880000 TX=0x0FA00000
//  Arrière gauche: RX=0x0F080000 TX=0x0F200000

// ============================================================================
// === SÉQUENCE DES FEUX =====================================================
// ============================================================================

struct EtatFeu {
  uint8_t valeur;           // Valeur hexadécimale à écrire sur GPLAT
  const char* nom;          // Nom descriptif pour affichage
};

const EtatFeu chenillard[] = {
  {0x01, "Veilleuse"},
  {0x02, "Code"},
  {0x04, "Phare"},
  {0x08, "Clignotant"},
  {0x00, "Éteint"}
};

const int NB_ETATS = sizeof(chenillard) / sizeof(EtatFeu);
int indexEtat = 0;                     // Indice dans le tableau de séquence

// ============================================================================
// === GESTION TEMPS DE CYCLE ================================================
// ============================================================================

const unsigned long DELAI_CYCLE = 1200; // Délai entre deux états (en ms)
unsigned long tDernierCycle = 0;        // Dernier passage de cycle

// ============================================================================
// === INITIALISATION DU BUS CAN =============================================
// ============================================================================

bool erreurInit = false;               // Flag d'erreur de démarrage

void initialiserCAN() {
  SPI.begin();

  if (mcp2515.reset() != MCP2515::ERROR_OK) {
    Serial.println(" Erreur reset MCP2515");
    erreurInit = true;
    return;
  }

  if (mcp2515.setBitrate(CAN_100KBPS, MCP_16MHZ) != MCP2515::ERROR_OK) {
    Serial.println(" Erreur configuration CAN à 100 kbps");
    erreurInit = true;
    return;
  }

  mcp2515.setNormalMode();
  Serial.println("✅ Bus CAN initialisé en mode normal");
}

// ============================================================================
// === CONFIGURATION DU MODULE VMD (MCP25050) ================================
// ============================================================================

void configurerModuleVMD() {
  Serial.println("  Configuration du module VMD...");

  // Mettre GP0 à GP3 en sortie, GP4 à GP7 en entrée → valeur = 0xF0
  if (!envoyerTrameCAN(REG_GPDDR, 0x7F, 0xF0)) {
    Serial.println(" Échec configuration GPDDR");
    erreurInit = true;
    return;
  }

  // Forcer les sorties à 0 (tous les feux éteints)
  if (!envoyerTrameCAN(REG_GPLAT, MASQUE_SORTIES, 0x00)) {
    Serial.println(" Échec initialisation feux");
    erreurInit = true;
    return;
  }

  Serial.println(" Module VMD prêt à l'emploi");
}

// ============================================================================
// === ENVOI D’UNE TRAME CAN AVEC ATTENTE ACQUITTEMENT ========================
// ============================================================================

bool envoyerTrameCAN(uint8_t registre, uint8_t masque, uint8_t valeur) {
  struct can_frame trame;
  trame.can_id = ID_IM | CAN_EFF_FLAG;
  trame.can_dlc = 3;
  trame.data[0] = registre;   // Registre cible (ex: GPLAT)
  trame.data[1] = masque;     // Masque des bits modifiés
  trame.data[2] = valeur;     // Valeur à écrire

  // Vider le buffer CAN pour éviter de lire une ancienne trame AIM
  struct can_frame ack;
  while (mcp2515.readMessage(&ack) == MCP2515::ERROR_OK);

  // Envoi de la trame
  if (mcp2515.sendMessage(&trame) != MCP2515::ERROR_OK) {
    Serial.println(" Erreur d'envoi CAN");
    return false;
  }

  // Attente d'acquittement pendant 200 ms
  unsigned long t0 = millis();
  while (millis() - t0 < 200) {
    if (mcp2515.readMessage(&ack) == MCP2515::ERROR_OK) {
      if ((ack.can_id & 0x1FFFFFFF) == ID_AIM) {
        Serial.println("✅ Acquittement AIM reçu");
        return true;
      }
    }
  }

  Serial.println(" Timeout : pas d'acquittement reçu");
  return false;
}

// ============================================================================
// === SETUP INITIAL ==========================================================
// ============================================================================

void setup() {
  Serial.begin(115200);
  while (!Serial);  // Attente de l’ouverture du moniteur série

  Serial.println("\n========== TP1 - Chenillard VMD ==========");
  Serial.println("Module cible : Feux avant droit (ID: 0x0E880000)");
  Serial.println("===========================================\n");

  initialiserCAN();
  if (!erreurInit) configurerModuleVMD();

  if (erreurInit) Serial.println("\n Système bloqué suite à une erreur !");
  else Serial.println("\n Chenillard démarré !\n");
}

// ============================================================================
// === BOUCLE PRINCIPALE ======================================================
// ============================================================================

void loop() {
  if (erreurInit) return;

  // Avancer dans le chenillard après DELAI_CYCLE ms
  if (millis() - tDernierCycle >= DELAI_CYCLE) {
    tDernierCycle = millis();

    // Passer à l’état suivant
    indexEtat = (indexEtat + 1) % NB_ETATS;
    const EtatFeu& etat = chenillard[indexEtat];

    Serial.print("\n🔁 Nouvel état : ");
    Serial.print(etat.nom);
    Serial.print(" (0x");
    Serial.print(etat.valeur, HEX);
    Serial.println(")");

    // Envoi de la trame vers le VMD
    if (!envoyerTrameCAN(REG_GPLAT, MASQUE_SORTIES, etat.valeur)) {
      Serial.println(" Erreur d'envoi de la trame GPLAT");
    }
  }
}
