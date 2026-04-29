#include <LiquidCrystal_I2C.h>
#include <Wire.h>

// Initialisation de l'écran LCD I2C
LiquidCrystal_I2C lcd(0x27, 16, 2);

// Sélection du type de moteur
#define MOTOR_TYPE 1

#if MOTOR_TYPE == 1
const float Ra = 1.4f;
const float La = 8.05e-3f;
const float Kt = 0.095f;
const float J = 7.49e-4f;
const float Bm = 4.32e-4f;
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

// Paramètres généraux
const float Ts = 0.001f; // période d’échantillonnage
float Tl = 0.0f;

// Broches de commande
constexpr int MOTOR_RUN_LED = 8;
constexpr int MOTOR_OFF_LED = 13;
constexpr int AUTO_LED = 9;
constexpr int START_BUTTON = 2;
constexpr int STOP_BUTTON = 3;
constexpr int AUTOMATIC_BUTTON = 4;

// État
volatile bool isRunning = false;
volatile bool isAutomatic = false;
char mode = 'M';
char state = 'O';

// Variables moteur
float Ia = 0.0f;
float Va = 12.0f;
float omega = 0.0f;
float theta = 0.0f;

// Minuterie
unsigned long previousMillis = 0;
const unsigned long restartInterval = 30000;

// Fonctions d’interruption
void startMotor() {
  isRunning = true;
}

void stopMotor() {
  isRunning = false;
}

void automaticMotor() {
  isAutomatic = digitalRead(AUTOMATIC_BUTTON) == HIGH;
}

void setup() {
  Serial.begin(115200);

  pinMode(MOTOR_RUN_LED, OUTPUT);
 pinMode(MOTOR_OFF_LED, OUTPUT);
  pinMode(AUTO_LED, OUTPUT);

  pinMode(START_BUTTON, INPUT_PULLUP);
  pinMode(STOP_BUTTON, INPUT_PULLUP);
  pinMode(AUTOMATIC_BUTTON, INPUT_PULLUP);

  digitalWrite(MOTOR_RUN_LED, LOW);
  digitalWrite(MOTOR_OFF_LED, LOW);
  digitalWrite(AUTO_LED, LOW);


  lcd.init();
  lcd.backlight();
  lcd.setCursor(0, 0); lcd.print("typ:"); lcd.print(MOTOR_TYPE);
  lcd.setCursor(6, 0); lcd.print("mod:"); lcd.print(mode);
  lcd.setCursor(12, 0); lcd.print("s:");
  lcd.setCursor(0, 1); lcd.print("V:"); lcd.print(0);
  lcd.setCursor(6, 1); lcd.print("I:"); lcd.print(0);
  lcd.setCursor(11, 1); lcd.print("w:"); lcd.print(0);

  attachInterrupt(digitalPinToInterrupt(START_BUTTON), startMotor, FALLING);
  attachInterrupt(digitalPinToInterrupt(STOP_BUTTON), stopMotor, FALLING);
  attachInterrupt(digitalPinToInterrupt(AUTOMATIC_BUTTON), automaticMotor, CHANGE);
}

void loop() {
  if (isRunning && !isAutomatic) {
    mode = 'M'; state = 'I';
    digitalWrite(MOTOR_RUN_LED, HIGH);
    digitalWrite(MOTOR_OFF_LED, LOW);
    digitalWrite(AUTO_LED, LOW);
  } else if (!isRunning && !isAutomatic) {
    mode = 'M'; state = 'O';
    digitalWrite(MOTOR_RUN_LED, LOW);
    digitalWrite(MOTOR_OFF_LED, LOW);
    digitalWrite(AUTO_LED, LOW);
  } else if (!isRunning && isAutomatic) {
    mode = 'A'; state = 'I';
    digitalWrite(MOTOR_RUN_LED, LOW);
    digitalWrite(MOTOR_OFF_LED, LOW);
    digitalWrite(AUTO_LED, HIGH);
  }

  // Affichage état
  lcd.setCursor(6, 0); lcd.print("mod:"); lcd.print(mode);
  lcd.setCursor(12, 0); lcd.print("s:"); lcd.print(state);

  if ((isRunning && !isAutomatic)||(!isRunning && isAutomatic)) {
    Ia += ((Va - Ra * Ia - Kt * omega) / La) * Ts;
    omega += ((Kt * Ia - Bm * omega + Tl) / J) * Ts;
    theta += omega * Ts;

    uint32_t signal_Va = (uint32_t)(Va * 4095.0 / 15.0);
    uint32_t signal_Ia = (uint32_t)(Ia * 4095.0 / 0.1);

    signal_Va = constrain(signal_Va, 0, 4095);
    signal_Ia = constrain(signal_Ia, 0, 4095);

    // Utilisation de DAC0 uniquement sur Due
    analogWrite(DAC0, signal_Va);

    lcd.setCursor(0, 1); lcd.print("V:"); lcd.print(Va);
    lcd.setCursor(6, 1); lcd.print("I:"); lcd.print(Ia);
    lcd.setCursor(11, 1); lcd.print("w:"); lcd.print(omega);

    if (isAutomatic) {
      unsigned long currentMillis = millis();
      if (currentMillis - previousMillis >= restartInterval) {
        previousMillis = currentMillis;
        Ia = 0.0f;
        omega = 0.0f;
        theta = 0.0f;
      }
    }

  } else {

    lcd.setCursor(0, 1); lcd.print("V:"); lcd.print(0);
    lcd.setCursor(6, 1); lcd.print("I:"); lcd.print(0);
    lcd.setCursor(11, 1); lcd.print("w:"); lcd.print(0);
  }

  delay(1);
}
