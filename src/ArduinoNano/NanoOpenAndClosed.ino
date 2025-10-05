// Team Rulo Bot — Nano control (shared for OPEN & CLOSED rounds)
// - One button on D2 to start (press–release, debounced).
// - Constant forward PWM; steering angle comes from the Pi via serial.
// - TURN_ON / TURN_OFF from the Pi drives LED on D10 (visual turn indicator).
// - Optional REV[:ms] command for a short, safe reverse (servo centered).
// Notes:
//   • We use the SAME sketch for both rounds.
//   • OPEN round forward speed = 255.
//   • CLOSED round forward speed < 255 (TBD). Adjust SPEED_FWD below.

#include <Servo.h>

// --- Pins ---
#define MOTOR_IN1   6   // DRV8871 IN1  (PWM for reverse)
#define MOTOR_IN2   5   // DRV8871 IN2  (PWM for forward)
#define SERVO_PIN   4   // Steering servo
#define LED_PIN     10  // Turn indicator (ON while Pi says TURN_ON)
#define BUTTON_PIN  2   // Start button to GND (INPUT_PULLUP)

// --- Servo limits (MG995) ---
const int SERVO_CENTER = 84;
const int SERVO_RIGHT  = 144;
const int SERVO_LEFT   = 24;

// --- Motion params ---
// OPEN = 255 ; CLOSED < 255 (pick later). Change only SPEED_FWD for the round.
const int SPEED_FWD      = 255;  // forward PWM (0–255)
const int SPEED_REV      = 150;  // reverse PWM (lower for safety)
const int REV_DEFAULT_MS = 450;  // fallback duration if REV has no :ms

// --- State ---
enum Mode {IDLE, RUNNING, HALT};
Mode mode = IDLE;

Servo sv;

// --- Motor helpers ---
inline void motorStop(){ digitalWrite(MOTOR_IN1,LOW); digitalWrite(MOTOR_IN2,LOW); }
inline void motorFwdPWM(int pwm){
  pwm = constrain(pwm,0,255);
  analogWrite(MOTOR_IN1,0);
  analogWrite(MOTOR_IN2,pwm);
}
inline void motorRevPWM(int pwm){
  pwm = constrain(pwm,0,255);
  analogWrite(MOTOR_IN1,pwm);
  analogWrite(MOTOR_IN2,0);
}
inline void steerAngle(int ang){
  sv.write(constrain(ang, SERVO_LEFT, SERVO_RIGHT));
}

// --- Button press–release with simple debounce ---
void waitButtonPressRelease(){
  // wait any current press to release
  while (digitalRead(BUTTON_PIN) == LOW) {}

  int last=HIGH, stable=HIGH; unsigned long t0=millis();
  // wait for stable LOW
  for(;;){
    int r=digitalRead(BUTTON_PIN);
    if (r!=last){ last=r; t0=millis(); }
    if (millis()-t0>30){ if (r!=stable){ stable=r; if (stable==LOW) break; } }
  }
  // wait for stable HIGH
  last=LOW; stable=LOW; t0=millis();
  for(;;){
    int r=digitalRead(BUTTON_PIN);
    if (r!=last){ last=r; t0=millis(); }
    if (millis()-t0>30){ if (r!=stable){ stable=r; if (stable==HIGH) break; } }
  }
}

// --- Serial line reader (blocking, short timeout) ---
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
bool startsWith(const char* s, const char* pfx){
  while(*pfx){ if(*s++!=*pfx++) return false; } return true;
}

// --- Actions ---
void startRun(){
  Serial.println(F("GO"));        // inform Pi we started
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
  // short, safe reverse; keep servo centered to avoid rubbing obstacles
  steerAngle(SERVO_CENTER);
  motorRevPWM(SPEED_REV);
  unsigned long t0 = millis();
  while ((int)(millis()-t0) < ms) { /* brief block */ }
  motorFwdPWM(SPEED_FWD);  // resume forward
}

// --- Setup/Loop ---
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
  // 1) Button starts the run (press–release), small 2s delay, then GO
  if (mode==IDLE || mode==HALT){
    waitButtonPressRelease();
    delay(2000);
    startRun();
  }

  // 2) Listen to Pi while RUNNING:
  //    STEER:<angle> | TURN_ON | TURN_OFF | STOP | REV[:ms]
  if (mode==RUNNING){
    char line[64];
    while (readLine(line, sizeof(line), 5)){
      // upper-case tokens for comparison (keeps numbers intact)
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
