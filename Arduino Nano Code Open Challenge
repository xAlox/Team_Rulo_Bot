// === Team Rulo Bot — Nano (GO button D2) ===
// Simple & stable: Forward always + TURN_[R/L](90°) by ENCODER + NUDGE
// - Sin heartbeat, sin watchdog, sin centrados extra ni lógica de parada automática.
// - STOP solo si llega el comando explícito por serie.
// - AL RECIBIR STOP: avanzar 30 cm y frenar.  ← NUEVO

#include <Servo.h>

// ---- Pins ----
#define MOTOR_IN1   6   // DRV8871 IN1 -> REV (LOW when forward)
#define MOTOR_IN2   5   // DRV8871 IN2 -> FWD (PWM)
#define SERVO_PIN   4   // MG995
#define LED_PIN     10
#define BUTTON_PIN  2   // Start button to GND (INPUT_PULLUP)

// Encoder NXT (quadrature A/B)
#define ENC_A 7        // OUT1
#define ENC_B 3        // OUT2 (interrupt)

// ---- Wheel / Encoder ----
const float WHEEL_DIAM_MM = 56.0f;
const int   PPR           = 360;

// ---- Motion params ----
const int  SERVO_CENTER   = 80;
const int  SERVO_RIGHT    = 130;     // tu valor actual (tune si hace falta)
const int  SERVO_LEFT     = 39;

const int  SPEED_FWD      = 190;     // avance constante
const int  SPEED_TURN     = 190;     // durante giro

// Giro por encoder (ARCO con servo fijo)
float TURN_RADIUS_MM = 200.0f;       // calibra para 90° exactos (radio asociado a SERVO_RIGHT/LEFT)

// Seguridad básica (solo para evitar bucle muerto en giro)
const unsigned long TURN_MAX_MS     = 2500;
const unsigned long START_DELAY_MS  = 2000;

// --- NUEVO: avance extra tras STOP ---
const float EXTRA_AFTER_STOP_MM     = 300.0f;     // 30 cm
const unsigned long EXTRA_DRIVE_MAX_MS = 4000;    // timeout por seguridad

// ---- State ----
enum Mode {IDLE, RUNNING, TURNING, HALT};
Mode mode = IDLE;

Servo sv;
volatile long encoderCount = 0;

// ---- Utils ----
void motorStop(){ digitalWrite(MOTOR_IN1,LOW); digitalWrite(MOTOR_IN2,LOW); }
void motorFwdPWM(int pwm){ pwm = constrain(pwm,0,255); digitalWrite(MOTOR_IN1,LOW); analogWrite(MOTOR_IN2,pwm); }
void steerAngle(int ang){ ang = constrain(ang,0,180); sv.write(ang); }

void resetEncoder(){ noInterrupts(); encoderCount = 0; interrupts(); }
long readEncoder(){ noInterrupts(); long c = encoderCount; interrupts(); return c; }

void encoderISR(){
  int a = digitalRead(ENC_A);
  int b = digitalRead(ENC_B);
  if (a == b) encoderCount++; else encoderCount--;
}

// s = R * theta;  pulses = s / (pi*D) * PPR
long pulsesForArc(float radius_mm, float theta_rad){
  float s = radius_mm * theta_rad;
  float wheel_circ = 3.1415926f * WHEEL_DIAM_MM;
  return (long)((s / wheel_circ) * (float)PPR + 0.5f);
}

// NUEVO: pulsos para distancia lineal
long pulsesForDistance(float dist_mm){
  float wheel_circ = 3.1415926f * WHEEL_DIAM_MM;
  return (long)((dist_mm / wheel_circ) * (float)PPR + 0.5f);
}

// NUEVO: avanzar en línea recta cierta distancia
void driveForwardDistanceMM(float dist_mm, int pwm, unsigned long timeout_ms){
  steerAngle(SERVO_CENTER);
  delay(80);
  long target = pulsesForDistance(dist_mm);
  resetEncoder();
  unsigned long t0 = millis();
  motorFwdPWM(pwm);
  while (true){
    if (readEncoder() >= target) break;
    if (millis() - t0 > timeout_ms) break;
  }
  motorStop();
}

// Botón con anti-rebote (press-release)
void waitButtonPressRelease(){
  while (digitalRead(BUTTON_PIN) == LOW) {}
  int last=HIGH, stable=HIGH; unsigned long t0=millis();
  for(;;){
    int r=digitalRead(BUTTON_PIN);
    if (r!=last){ last=r; t0=millis(); }
    if (millis()-t0>30){ if (r!=stable){ stable=r; if (stable==LOW) break; } }
  }
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

// ---- Actions ----
void startRun(){
  Serial.println(F("GO"));          // avisa a la Pi
  // (sin centrado extra)
  resetEncoder();
  motorFwdPWM(SPEED_FWD);
  mode = RUNNING;
  digitalWrite(LED_PIN, HIGH);
}

// 90° RIGHT (arco con servo fijo)
void doTurnRight90_byEncoder(){
  mode = TURNING;

  steerAngle(SERVO_RIGHT);
  delay(120);                       // dar tiempo a que el servo llegue

  const float THETA = 3.1415926f * 0.5f;
  long targetPulses = pulsesForArc(TURN_RADIUS_MM, THETA);

  resetEncoder();
  unsigned long t0 = millis();
  motorFwdPWM(SPEED_TURN);

  while (true){
    long c = readEncoder();
    if (c >= targetPulses) break;
    if (millis() - t0 > TURN_MAX_MS) break;
  }

  motorStop();
  steerAngle(SERVO_CENTER);         // recenter SOLO al terminar el giro
  delay(120);

  resetEncoder();
  motorFwdPWM(SPEED_FWD);
  mode = RUNNING;
}

// 90° LEFT (arco con servo fijo)
void doTurnLeft90_byEncoder(){
  mode = TURNING;

  steerAngle(SERVO_LEFT);
  delay(120);

  const float THETA = 3.1415926f * 0.5f;
  long targetPulses = pulsesForArc(TURN_RADIUS_MM, THETA);

  resetEncoder();
  unsigned long t0 = millis();
  motorFwdPWM(SPEED_TURN);

  while (true){
    long c = readEncoder();
    if (c >= targetPulses) break;
    if (millis() - t0 > TURN_MAX_MS) break;
  }

  motorStop();
  steerAngle(SERVO_CENTER);
  delay(120);

  resetEncoder();
  motorFwdPWM(SPEED_FWD);
  mode = RUNNING;
}

// NUDGE: solo servo, no cambia la velocidad
void doNudge(int sign, int deg, int ms){
  if (mode != RUNNING) return;
  deg = constrain(deg, 0, 40);
  int target = SERVO_CENTER + (sign>0 ? deg : -deg);
  steerAngle(target);
  delay(ms);
  steerAngle(SERVO_CENTER);
}

// STOP: ahora hace 30 cm extra y frena (servo centrado)  ← NUEVO
void doStop(){
  // Avance extra controlado por encoder
  driveForwardDistanceMM(EXTRA_AFTER_STOP_MM, SPEED_FWD, EXTRA_DRIVE_MAX_MS);
  // Parada final
  motorStop();
  mode = HALT;
  digitalWrite(LED_PIN, LOW);
}

// ---- Setup/Loop ----
void setup(){
  Serial.begin(115200);

  pinMode(MOTOR_IN1,OUTPUT);
  pinMode(MOTOR_IN2,OUTPUT);
  pinMode(LED_PIN,OUTPUT);
  pinMode(BUTTON_PIN,INPUT_PULLUP);

  pinMode(ENC_A,INPUT_PULLUP);
  pinMode(ENC_B,INPUT_PULLUP);
  attachInterrupt(digitalPinToInterrupt(ENC_B), encoderISR, CHANGE);

  sv.attach(SERVO_PIN);
  steerAngle(SERVO_CENTER);   // posición base
  motorStop();
  digitalWrite(LED_PIN, LOW);
  mode = IDLE;
}

// helpers parser
bool startsWith(const char* s, const char* pfx){ while(*pfx){ if(*s++!=*pfx++) return false; } return true; }
bool parseNUDGE(const char* s, int& sign, int& deg, int& ms){
  if (startsWith(s,"NUDGE_R:")){ sign=+1; s+=8; }
  else if (startsWith(s,"NUDGE_L:")){ sign=-1; s+=8; }
  else return false;

  int v1=0, v2=0, i=0;

  // grados
  while (s[i] && s[i] != ':' && i < 5){
    if (s[i] < '0' || s[i] > '9') return false;
    v1 = v1*10 + (s[i] - '0');
    i++;
  }
  if (s[i] != ':') return false;
  i++;

  // milisegundos
  int j = i;
  while (s[j] && j < i + 6){
    if (s[j] < '0' || s[j] > '9') return false;
    v2 = v2*10 + (s[j] - '0');
    j++;
  }

  deg = v1; ms = v2;
  return true;
}

void loop(){
  // 1) Arranque: botón -> delay -> GO -> run
  if (mode==IDLE || mode==HALT){
    waitButtonPressRelease();
    delay(START_DELAY_MS);
    startRun();
  }

  // 2) Escucha órdenes de la Pi (TURN/NUDGE/STOP)
  if (mode==RUNNING || mode==TURNING){
    char line[48];
    while (readLine(line, sizeof(line), 15)){   // simple y suficiente
      for(char* p=line; *p; ++p) if(*p>='a' && *p<='z') *p -= 32; // uppercase

      if (!strcmp(line,"TURN_R")){
        doTurnRight90_byEncoder();
      } else if (!strcmp(line,"TURN_L")){
        doTurnLeft90_byEncoder();
      } else if (!strcmp(line,"STOP")){
        doStop();                             // ahora hace 30 cm y frena
      } else {
        int sign=0,deg=0,ms=0;
        if (parseNUDGE(line,sign,deg,ms)){
          doNudge(sign,deg,ms);
        } else if (startsWith(line,"SET_R:")){
          int r=atoi(line+6);
          if (r>=80 && r<=500) TURN_RADIUS_MM = (float)r; // tuning opcional
        }
      }
    }
  }
}
