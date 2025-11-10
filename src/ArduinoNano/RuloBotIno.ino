// === Team Rulo Bot — Nano (GO button D2) ===
// Forward constante (PWM configurable) + STEER:<ang> continuo desde la Pi
// LED D10 encendido cuando la Pi entra en "modo giro" (TURN_ON / TURN_OFF)
// Centro 84, límites 24–144
// NUEVO: comando REV[:ms] para dar reversa breve segura (por defecto 400–500 ms)

#include <Servo.h>

// ---- Pins ----
#define MOTOR_IN1   6   // DRV8871 IN1 -> REV (PWM en reversa)
#define MOTOR_IN2   5   // DRV8871 IN2 -> FWD (PWM en avance)
#define SERVO_PIN   4   // MG995
#define LED_PIN     10
#define BUTTON_PIN  2   // Start button a GND (INPUT_PULLUP)

// ---- Servo limits ----
const int SERVO_CENTER = 84;
const int SERVO_RIGHT  = 144;
const int SERVO_LEFT   = 24;

// ---- Motion params ----
const int SPEED_FWD      = 200 ;   // avance constante (0-255)
const int SPEED_REV      = 150;   // reversa (0-255) — menor por seguridad
const int REV_DEFAULT_MS = 450;   // duración por defecto si no se especifica

// ---- State ----
enum Mode {IDLE, RUNNING, HALT};
Mode mode = IDLE;

Servo sv;

// ---- Utils ----
void motorStop(){ digitalWrite(MOTOR_IN1,LOW); digitalWrite(MOTOR_IN2,LOW); }
void motorFwdPWM(int pwm){
  pwm = constrain(pwm,0,255);
  analogWrite(MOTOR_IN1,0);        // IN1 LOW (sin PWM)
  analogWrite(MOTOR_IN2,pwm);      // IN2 PWM -> FWD
}
void motorRevPWM(int pwm){
  pwm = constrain(pwm,0,255);
  analogWrite(MOTOR_IN1,pwm);      // IN1 PWM -> REV
  analogWrite(MOTOR_IN2,0);        // IN2 LOW
}
void steerAngle(int ang){
  ang = constrain(ang, SERVO_LEFT, SERVO_RIGHT);
  sv.write(ang);
}

// Botón con anti-rebote (press-release)
void waitButtonPressRelease(){
  while (digitalRead(BUTTON_PIN) == LOW) {}
  int last=HIGH, stable=HIGH; unsigned long t0=millis();
  // espera bajada estable
  for(;;){
    int r=digitalRead(BUTTON_PIN);
    if (r!=last){ last=r; t0=millis(); }
    if (millis()-t0>30){ if (r!=stable){ stable=r; if (stable==LOW) break; } }
  }
  // espera subida estable
  last=LOW; stable=LOW; t0=millis();
  for(;;){
    int r=digitalRead(BUTTON_PIN);
    if (r!=last){ last=r; t0=millis(); }
    if (millis()-t0>30){ if (r!=stable){ stable=r; if (stable==HIGH) break; } }
  }
}

// Serial line reader
bool readLine(char* buf, int buflen, unsigned long timeout_ms){
  int idx=0; unsigned long t0=millis();
  while (millis()-t0<timeout_ms){
    while (Serial.available()){
      char c=(char)Serial.read();
      if (c=='\n'||c=='\r'){ if (idx>0){ buf[idx]=0; return true; } }
      else if (idx<buflen-1) buf[idx++]=c;
    }
  }
  buf[0]=0; return false;
}

bool startsWith(const char* s, const char* pfx){ while(*pfx){ if(*s++!=*pfx++) return false; } return true; }

// ---- Actions ----
void startRun(){
  Serial.println(F("GO"));           // avisa a la Pi
  steerAngle(SERVO_CENTER);
  motorFwdPWM(SPEED_FWD);
  digitalWrite(LED_PIN, LOW);
  mode = RUNNING;
}

void doStop(){
  motorStop();
  steerAngle(SERVO_CENTER);
  digitalWrite(LED_PIN, LOW);
  mode = HALT;
}

void doReverse(int ms){
  // breve reversa segura, con servo en centro para evitar roces
  steerAngle(SERVO_CENTER);
  motorRevPWM(SPEED_REV);
  unsigned long t0 = millis();
  while ((int)(millis()-t0) < ms) {
    // vacío (bloqueante breve)
  }
  motorFwdPWM(SPEED_FWD);  // retoma avance
}

// ---- Setup/Loop ----
void setup(){
  Serial.begin(115200);

  pinMode(MOTOR_IN1,OUTPUT);
  pinMode(MOTOR_IN2,OUTPUT);
  pinMode(LED_PIN,OUTPUT);
  pinMode(BUTTON_PIN,INPUT_PULLUP);

  sv.attach(SERVO_PIN);
  steerAngle(SERVO_CENTER);
  motorStop();
  digitalWrite(LED_PIN, LOW);
  mode = IDLE;
}

void loop(){
  // 1) Arranque por botón: espera press-release, delay 2s, GO y correr
  if (mode==IDLE || mode==HALT){
    waitButtonPressRelease();
    delay(2000);
    startRun();
  }

  // 2) Escucha órdenes de la Pi (STEER/LED/STOP/REV)
  if (mode==RUNNING){
    char line[64];
    while (readLine(line, sizeof(line), 5)){
      // convertir solo tokens a mayúsculas para comparar
      for(char* p=line; *p; ++p) if(*p>='a' && *p<='z') *p -= 32;

      if (startsWith(line,"STEER:")){
        int ang = atoi(line+6);
        steerAngle(ang);
      } else if (!strcmp(line,"TURN_ON")){
        digitalWrite(LED_PIN, HIGH);
      } else if (!strcmp(line,"TURN_OFF")){
        digitalWrite(LED_PIN, LOW);
      } else if (!strcmp(line,"STOP")){
        doStop();
      } else if (startsWith(line,"REV")){
        int ms = REV_DEFAULT_MS;
        char* colon = strchr(line, ':');
        if (colon){ ms = atoi(colon+1); if (ms<=0) ms = REV_DEFAULT_MS; }
        doReverse(ms);
      }
    }
  }
}
