#include <LiquidCrystal_I2C.h>
#include <Wire.h>
LiquidCrystal_I2C lcd(0x27, 16, 2);
// Sélectionner le type de moteur (1, 2 ou 3)
#define MOTOR_TYPE 1  // Modifier cette valeur pour sélectionner un moteur

// Définir les paramètres du moteur en fonction du type sélectionné
#if MOTOR_TYPE == 1
const float Ra = 1.4f;      // Résistance de l'induit (Ohms)
const float La = 8.05e-3f;  // Inductance de l'induit (H)
const float Kt = 0.095f;    // Constante de couple (Nm/A)
const float J = 7.49e-4f;   // Inertie du rotor (kg.m²)
const float Bm = 4.32e-4f;  // Coefficient de frottement visqueux (Nm.s/rad)
#elif MOTOR_TYPE == 2
const float Ra = 3.2f;
const float La = 5.35e-3f;
const float Kt = 0.3f;
const float J = 5e-4f;
const float Bm = 1e-5f;
#elif MOTOR_TYPE == 3
const float Ra = 7.72f;
const float La = 162.73e-3f;
const float Kt = 1.25f;
const float J = 2.36e-2f;
const float Bm = 3e-3f;
#endif

const float Ts = 0.001f;  // Période d'échantillonnage (1ms)
float Tl = 0.0f;          // Couple de charge initial (Nm)

// Définir les broches de sortie PWM pour le contrôle du moteur
constexpr int VA_PWM = 6;  // Broche PWM pour la tension d'alimentation du moteur
constexpr int IA_PWM = 9;  // Broche PWM pour le courant du moteur
int MOTOR_RUN_LED = 8;
// Définir les broches des boutons pour les commandes de démarrage et d'arrêt
constexpr int START_BUTTON = 2;  // Broche du bouton de démarrage
constexpr int STOP_BUTTON = 3;   // Broche du bouton d'arrêt
constexpr int AUTOMATIC_BUTTON = 4;

volatile bool isRunning = false; // État du moteur (en marche ou arrêté)
volatile bool isAutomatic = false;
char mode = "M";
char state = "O";

// Définir les variables dynamiques pour le contrôle du moteur
float Ia = 0.0f;     // Courant du moteur (A)
float Va = 12.0f;    // Tension appliquée (V)
float omega = 0.0f;  // Vitesse angulaire (rad/s)
float theta = 0.0f;  // Position angulaire (rad)

// Fonctions d'interruption pour démarrer et arrêter le moteur
void startMotor() {
    isRunning = true; // Activer le moteur lorsque le bouton START est enfoncé

}

void stopMotor() {
    isRunning = false; // Arrêter le moteur lorsque le bouton STOP est enfoncé
}

void automaticMotor(){
  if (digitalRead(AUTOMATIC_BUTTON)==HIGH){
    isAutomatic = true;
  }
  else
  {
    isAutomatic = false;
  }
}

void(* resetFunc) (void) = 0; // declare reset function @ address 0 pour redémarrage automatique
// Variables globales pour minuterie
unsigned long previousMillis = 0;  // Pour stocker le temps précédent
const unsigned long restartInterval = 30000; // 30 secondes (en millisecondes)


void setup() {
    Serial.begin(115200); // Initialiser la communication série pour le débogage

    // Définir les broches PWM comme sorties
    pinMode(VA_PWM, OUTPUT);
    pinMode(IA_PWM, OUTPUT);
    pinMode(MOTOR_RUN_LED, OUTPUT);
   
    // Définir les broches des boutons comme entrées avec résistances pull-up internes
    pinMode(START_BUTTON, INPUT_PULLUP);
    pinMode(STOP_BUTTON, INPUT_PULLUP);
    pinMode(AUTOMATIC_BUTTON, INPUT_PULLUP);
    digitalWrite(MOTOR_RUN_LED, LOW);
      lcd.init();
  lcd.backlight();
  lcd.setCursor(0, 0);
  lcd.print("typ:");
  lcd.print(MOTOR_TYPE);
  lcd.setCursor(6, 0);
  lcd.print("mod:");
  lcd.print("M")
  lcd.setCursor(12, 0);
  lcd.print("s:");
  lcd.setCursor(0, 1);
  lcd.print("V:");
  lcd.print(0);
  lcd.setCursor(6, 1);
  lcd.print("I:");
  lcd.print(0);
  lcd.setCursor(11, 1);
  lcd.print("w:");
  lcd.print(0);
  
    // Attacher des interruptions pour détecter les pressions sur les boutons
    attachInterrupt(digitalPinToInterrupt(START_BUTTON), startMotor, FALLING);
    attachInterrupt(digitalPinToInterrupt(STOP_BUTTON), stopMotor, FALLING);
    attachInterrupt(digitalPinToInterrupt(AUTOMATIC_BUTTON), automaticMotor, CHANGE);
}

void loop() {
    if (isRunning==true && isAutomatic==false) {
  state = "I";
  mode = "M";
  lcd.setCursor(6, 0);
  lcd.print("mod:");
  lcd.print(mode);
  lcd.setCursor(12, 0);
  lcd.print("s:");
  lcd.print(state);
        // Mettre à jour les équations dynamiques du moteur
        Ia += ((Va - Ra * Ia - Kt * omega) / La) * Ts;  // Calculer le courant Ia
        omega += ((Kt * Ia - Bm * omega + Tl) / J) * Ts; // Calculer la vitesse angulaire
        theta += omega * Ts;                             // Mettre à jour la position angulaire

        // Convertir les valeurs dans la plage 0-4095 pour le DAC (DUE)

// Transformation de Va
uint32_t signal_Va = (uint32_t)map((int)(Va * 100), 0, 1500, 0, 4095);

// Transformation de Ia
uint32_t signal_Ia = (uint32_t)map((int)(Ia * 1000), 0, 100, 0, 4095);

// Contraindre les valeurs dans la plage 0-4095
signal_Va = constrain(signal_Va, 0, 4095);
signal_Ia = constrain(signal_Ia, 0, 4095);

// Envoyer les signaux DAC aux broches correspondantes
analogWrite(DAC0, signal_Va);  // A0 physique
analogWrite(DAC1, signal_Ia);  // A1 physique

// Allumer LED moteur
digitalWrite(MOTOR_RUN_LED, HIGH);

  lcd.setCursor(0, 1);
  lcd.print("V:");
  lcd.print(Va);
  lcd.setCursor(6, 1);
  lcd.print("I:");
  lcd.print(Ia);
  lcd.setCursor(11, 1);
  lcd.print("w:");
  lcd.print(omega);

    } else if(isRunning == false && isAutomatic == false) {
  state = "O";
  mode = "M";
  lcd.setCursor(6, 0);
  lcd.print("mod:");
  lcd.print(mode);
  lcd.setCursor(12, 0);
  lcd.print("s:");
  lcd.print(state);
        // Si le moteur est arrêté, mettre les sorties PWM à zéro
  digitalWrite(MOTOR_RUN_LED, LOW);
  lcd.setCursor(0, 1);
  lcd.print("V:");
  lcd.print(0);
  lcd.setCursor(6, 1);
  lcd.print("I:");
  lcd.print(0);
  lcd.setCursor(11, 1);
  lcd.print("w:");
  lcd.print(0);
    }
   else if (isRunning == false && isAutomatic == true) {
    // --- Mise à jour des équations dynamiques ---
  state = "I";
  mode = "A";
  lcd.setCursor(6, 0);
  lcd.print("mod:");
  lcd.print(mode);
  lcd.setCursor(12, 0);
  lcd.print("s:");
  lcd.print(state);
    Ia += ((Va - Ra * Ia - Kt * omega) / La) * Ts;   // Calculer le courant Ia
    omega += ((Kt * Ia - Bm * omega + Tl) / J) * Ts; // Calculer la vitesse angulaire
    theta += omega * Ts;                             // Mettre à jour la position angulaire

    // --- Conversion pour DAC ---
    uint32_t signal_Va = (uint32_t)(Va * 4095.0 / 15.0);
    uint32_t signal_Ia = (uint32_t)(Ia * 4095.0 / 0.1);

    // Contraindre les valeurs
    signal_Va = constrain(signal_Va, 0, 4095);
    signal_Ia = constrain(signal_Ia, 0, 4095);

    // --- Envoyer aux DACs ---
    analogWrite(DAC0, signal_Va);
    analogWrite(DAC1, signal_Ia);

    // --- LED indicateur moteur ---
    digitalWrite(MOTOR_RUN_LED, HIGH);

  lcd.setCursor(0, 1);
  lcd.print("V:");
  lcd.print(Va);
  lcd.setCursor(6, 1);
  lcd.print("I:");
  lcd.print(Ia);
  lcd.setCursor(11, 1);
  lcd.print("w:");
  lcd.print(omega);
    // --- Gestion du "reset logiciel" toutes les 30 secondes ---
    unsigned long currentMillis = millis();
    if (currentMillis - previousMillis >= restartInterval) {
        previousMillis = currentMillis;  // Réinitialiser la minuterie
        
        // Réinitialiser uniquement les variables
        Ia = 0.0;
        omega = 0.0;
        theta = 0.0;
  lcd.setCursor(0, 1);
  lcd.print("V:");
  lcd.print(Va);
  lcd.setCursor(6, 1);
  lcd.print("I:");
  lcd.print(Ia);
  lcd.setCursor(11, 1);
  lcd.print("w:");
  lcd.print(omega);
  digitalWrite(MOTOR_RUN_LED, LOW);
    }
 }
    
    delay(1); // Attendre 1 ms avant la prochaine itération
}
