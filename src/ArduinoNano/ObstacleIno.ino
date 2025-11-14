// ============================================================================
// Team Rulo Bot — Park Exit por Encoder -> luego Modo Obstacle (unificado)
// HW: Nano + DRV8871 (IN1=6=REV, IN2=5=FWD), Servo MG995 (D4), LED D10,
//     Botón D2 (a GND, INPUT_PULLUP), Encoder canal A en D3 (INT1).
// Baud: 115200
// ============================================================================

#include <Arduino.h>
#include <Servo.h>
#include <math.h> // roundf()

// ---------------- Pins ----------------
#define MOTOR_IN1    6     // DRV8871 IN1 -> REV (PWM)
#define MOTOR_IN2    5     // DRV8871 IN2 -> FWD (PWM)
#define SERVO_PIN    4     // MG995
#define LED_PIN      10
#define BUTTON_PIN   2     // Push-button a GND (INPUT_PULLUP)
#define ENC_A_PIN    3     // Encoder canal A (INT1)

// ------------- Servo -------------
const int SERVO_CENTER = 84;
const int SERVO_LEFT   = 30;   // extremo IZQ (validado)
const int SERVO_RIGHT  = 138;  // extremo DER (validado)

// ========= Objetos/estado que deben existir ANTES de usarse =========
Servo sv;

enum RunMode {WAIT_BUTTON_EXIT, EXIT_RUNNING, OBSTACLE_RUNNING};
RunMode mode = WAIT_BUTTON_EXIT;

// ========= Prototipos (opcional, pero útil si mueves cosas) =========
void serialPump();
void steerAngle(int ang);

// ================= PARÁMETROS AJUSTABLES =================
// ---- EXIT por encoder ----
int   EXTREME_ANGLE       = SERVO_LEFT; // Se ajusta por Serial con EXT:LEFT/RIGHT

float FWD1_DIST_CM        = 8.0;
float REV1_DIST_CM        = 5.0;
float FWD2_DIST_CM        = 6.7;
float REV2_DIST_CM        = 3.0;
float FWD3_DIST_CM        = 17.0;

int   SPEED_FWD           = 170;
int   SPEED_REV           = 150;

int   PWM_BRAKE           = 180;
int   BRAKE_BURST_MS      = 50;
int   COAST_MS            = 40;
int   DWELL_MS            = 800;

int   EXTREME_WAIT_MS     = 1000;
int   SERVO_SETTLE_MS     = 200;
int   CENTER_SETTLE_MS    = 150;

int   PWM_KICK            = 230;  // ayuda a romper fricción en REV
int   KICK_MS             = 60;

// ---- Obstacle (fase 2) ----
int   OBST_SPEED_FWD      = 200;  // avance constante en obstacle
int   OBST_SPEED_REV      = 150;  // REV segura para comando REV
int   OBST_REV_DEFAULT_MS = 450;  // si no se especifica en REV:ms

// Encoder / rueda
const float WHEEL_DIAMETER_MM = 56.0f; // rueda trasera actual
const long  TICKS_PER_REV     = 180;
// =========================================================

// ========= Estado encoder =========
volatile unsigned long enc_count = 0;
void enc_isr(){ enc_count++; }

// ========= Utils motor/servo =========
inline void motorCoast(){ analogWrite(MOTOR_IN1,0); analogWrite(MOTOR_IN2,0); }
inline void motorFwdPWM(int pwm){ pwm=constrain(pwm,0,255); analogWrite(MOTOR_IN1,0);   analogWrite(MOTOR_IN2,pwm); }
inline void motorRevPWM(int pwm){ pwm=constrain(pwm,0,255); analogWrite(MOTOR_IN1,pwm); analogWrite(MOTOR_IN2,0);  }

void hardBrakeFromFWD(){
  motorRevPWM(PWM_BRAKE);
  unsigned long t0=millis();
  while(millis()-t0 < (unsigned long)BRAKE_BURST_MS){
    if (digitalRead(BUTTON_PIN)==LOW) break;
  }
  motorCoast(); delay(COAST_MS);
}

void hardBrakeFromREV(){
  motorFwdPWM(PWM_BRAKE);
  unsigned long t0=millis();
  while(millis()-t0 < (unsigned long)BRAKE_BURST_MS){
    if (digitalRead(BUTTON_PIN)==LOW) break;
  }
  motorCoast(); delay(COAST_MS);
}

void steerAngle(int ang){
  ang = constrain(ang, SERVO_LEFT, SERVO_RIGHT);
  sv.write(ang);
}

void waitButtonPressRelease(){
  while (digitalRead(BUTTON_PIN) != LOW) { serialPump(); } // permitir EXT:* antes
  delay(20);
  while (digitalRead(BUTTON_PIN) == LOW) { serialPump(); }
  delay(20);
}

bool emergencyStopIfPressed(){
  if (digitalRead(BUTTON_PIN)==LOW){
    motorCoast();
    return true;
  }
  return false;
}

// Encoder: cm -> ticks
unsigned long ticksForDistanceCm(float cm){
  float perim_mm = 3.14159265f * WHEEL_DIAMETER_MM;
  float dist_mm  = cm * 10.0f;
  float revs     = dist_mm / perim_mm;
  unsigned long ticks = (unsigned long) roundf(revs * (float)TICKS_PER_REV);
  return (ticks>0 ? ticks : 1);
}

// Mover por distancia usando encoder
bool moveDistanceCm(float cm, int dir, int pwm, bool kick){
  unsigned long target = ticksForDistanceCm(cm);
  noInterrupts(); enc_count = 0; interrupts();

  if (dir > 0){
    motorFwdPWM(pwm);
  } else {
    if (kick){ motorRevPWM(PWM_KICK); delay(KICK_MS); }
    motorRevPWM(pwm);
  }

  unsigned long wd = millis();
  while (enc_count < target){
    if (emergencyStopIfPressed()) { motorCoast(); return false; }
    if (millis() - wd > 7000UL) {
      Serial.println(F("[WARN] Watchdog — abort tramo"));
      motorCoast(); return false;
    }
  }
  motorCoast();
  return true;
}

// ========= Serial parser =========
String rxLine;

void handleCommand(const String& s){
  if (s.startsWith("EXT:")){
    if (s.endsWith("LEFT")){
      EXTREME_ANGLE = SERVO_LEFT;
      Serial.println(F("[EXT] LEFT seleccionado"));
    } else if (s.endsWith("RIGHT")){
      EXTREME_ANGLE = SERVO_RIGHT;
      Serial.println(F("[EXT] RIGHT seleccionado"));
    }
  } else if (s == "EXT?"){
    Serial.print(F("[EXT] actual = ")); Serial.println(EXTREME_ANGLE);
  }
  // En modo Obstacle, también aceptamos control:
  else if (mode==OBSTACLE_RUNNING){
    if (s.startsWith("STEER:")){
      int ang = s.substring(6).toInt();
      steerAngle(ang);
    } else if (s=="TURN_ON"){
      digitalWrite(LED_PIN, HIGH);
    } else if (s=="TURN_OFF"){
      digitalWrite(LED_PIN, LOW);
    } else if (s=="STOP"){
      motorCoast(); steerAngle(SERVO_CENTER);
    } else if (s.startsWith("REV")){
      int ms = OBST_REV_DEFAULT_MS;
      int idx = s.indexOf(':');
      if (idx>0){ int v = s.substring(idx+1).toInt(); if (v>0) ms=v; }
      // REV segura con servo al centro:
      steerAngle(SERVO_CENTER);
      motorRevPWM(OBST_SPEED_REV);
      unsigned long t0=millis();
      while((int)(millis()-t0) < ms){}
      motorFwdPWM(OBST_SPEED_FWD);
    }
  }
}

void serialPump(){
  while (Serial.available()){
    char c = (char)Serial.read();
    if (c=='\n' || c=='\r'){
      if (rxLine.length()>0){ handleCommand(rxLine); rxLine=""; }
    } else {
      if (rxLine.length()<80) rxLine += c;
    }
  }
}

void setup(){
  Serial.begin(115200);
  pinMode(MOTOR_IN1,OUTPUT);
  pinMode(MOTOR_IN2,OUTPUT);
  pinMode(LED_PIN,OUTPUT);
  pinMode(BUTTON_PIN,INPUT_PULLUP);
  pinMode(ENC_A_PIN,INPUT_PULLUP);

  attachInterrupt(digitalPinToInterrupt(ENC_A_PIN), enc_isr, RISING);

  sv.attach(SERVO_PIN);
  steerAngle(SERVO_CENTER);
  motorCoast();
  digitalWrite(LED_PIN, LOW);

  Serial.println(F("Unificado: Park Exit -> Obstacle. Acepte EXT:LEFT/RIGHT por Serial."));
  Serial.print (F("DIAM(mm)=")); Serial.print(WHEEL_DIAMETER_MM);
  Serial.print (F("  TICKS/REV=")); Serial.println(TICKS_PER_REV);
  Serial.println(F("Pulse el botón para ejecutar la salida por encoder."));
}

void loop(){
  serialPump();

  if (mode==WAIT_BUTTON_EXIT){
    // Espera botón para iniciar la secuencia de salida
    waitButtonPressRelease();
    digitalWrite(LED_PIN, HIGH);
    Serial.println(F("[SEQ] START (Exit)"));
    mode = EXIT_RUNNING;

    // ===== Secuencia por encoder (+REV2/FWD3) =====
    steerAngle(EXTREME_ANGLE);
    delay(EXTREME_WAIT_MS);

    if (!moveDistanceCm(FWD1_DIST_CM, +1, SPEED_FWD, false)) goto end_exit;
    hardBrakeFromFWD(); delay(DWELL_MS);

    steerAngle(SERVO_CENTER); delay(CENTER_SETTLE_MS);
    if (!moveDistanceCm(REV1_DIST_CM, -1, SPEED_REV, true)) goto end_exit;
    hardBrakeFromREV(); delay(DWELL_MS);

    steerAngle(EXTREME_ANGLE); delay(SERVO_SETTLE_MS);
    if (!moveDistanceCm(FWD2_DIST_CM, +1, SPEED_FWD, false)) goto end_exit;
    hardBrakeFromFWD(); delay(DWELL_MS);

    steerAngle(SERVO_CENTER); delay(CENTER_SETTLE_MS);
    if (!moveDistanceCm(REV2_DIST_CM, -1, SPEED_REV, true)) goto end_exit;
    hardBrakeFromREV(); delay(DWELL_MS);

    steerAngle(EXTREME_ANGLE); delay(SERVO_SETTLE_MS);
    if (!moveDistanceCm(FWD3_DIST_CM, +1, SPEED_FWD, false)) goto end_exit;
    hardBrakeFromFWD();

  end_exit:
    motorCoast();
    steerAngle(SERVO_CENTER);
    digitalWrite(LED_PIN, LOW);
    Serial.println(F("[SEQ] END"));     // <- la Pi espera esto para pasar a Obstacle
    delay(1000);                        // 1 s antes de Obstacle
    // ===== Entrar a modo Obstacle =====
    motorFwdPWM(OBST_SPEED_FWD);
    mode = OBSTACLE_RUNNING;
    Serial.println(F("[OBST] RUNNING — listo para STEER:/TURN_ON/OFF/STOP/REV"));

  } else if (mode==OBSTACLE_RUNNING){
    // Mantener avance; control por Serial en handleCommand()
    if (digitalRead(BUTTON_PIN)==LOW){
      motorCoast(); steerAngle(SERVO_CENTER); digitalWrite(LED_PIN, LOW);
      while (digitalRead(BUTTON_PIN)==LOW){} delay(20);
      mode = WAIT_BUTTON_EXIT;
      Serial.println(F("[OBST] STOP by button — regreso a espera de salida."));
    }
  }
}
