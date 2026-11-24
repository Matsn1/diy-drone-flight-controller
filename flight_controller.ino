// ================================================================
// DIY DRONE FLIGHT CONTROLLER v1.0 (ESP32)
// Dual core:
//   Core 1 -> flight-critical loop (IMU + PID + motors) at 250 Hz
//   Core 0 -> NRF24L01 radio receiver + WiFi PID-tuning page
// Based on pratikPhadte/ESP32-Flight-controller (angle mode), extended.
// ================================================================

#include <Wire.h>
#include <ESP32Servo.h>
#include <WiFi.h>
#include <AsyncTCP.h>
#include <ESPAsyncWebServer.h>
#include <SPIFFS.h>
#include <SPI.h>
#include <RF24.h>

// ================================================================
// USER CONFIG
// ================================================================
#define ANGLE_MODE   0     // 0 = Angle (self-levelling), 1 = Acro
#define LOOP_TIME_US 4000  // 4 ms = 250 Hz

// Motor pins (X frame)
#define MOT1_PIN 27  // Front Right - CCW
#define MOT2_PIN 13  // Rear Right  - CW
#define MOT3_PIN 26  // Rear Left   - CCW
#define MOT4_PIN 12  // Front Left  - CW

// Motor 2 uses a different ESC that starts spinning later
#define MOT2_OFFSET 120

#define THROTTLE_IDLE    1055
#define THROTTLE_CUTOFF  1000
#define THROTTLE_MAX     1900
#define ARM_THROTTLE_MAX 1050   // throttle must be below this to arm
#define FAILSAFE_MS      500    // no radio packet for 500 ms -> cut motors

// WiFi access point (PID tuning only)
const char* ssid     = "DRONE_FC";
const char* password = "12345678";

// NRF24L01
#define NRF_CE  4
#define NRF_CSN 5

// ================================================================
// PID GAINS (starting values, tunable live from the web page)
// ================================================================
// Rate PID (inner loop)
float PRateRoll  = 1.2,  IRateRoll  = 2.5, DRateRoll  = 0.012;
float PRatePitch = 1.2,  IRatePitch = 2.5, DRatePitch = 0.012;
float PRateYaw   = 4.0,  IRateYaw   = 3.0, DRateYaw   = 0.0;
// Angle PID (outer loop)
float PAngleRoll  = 4.0, IAngleRoll  = 0.0, DAngleRoll  = 0.01;
float PAnglePitch = 4.0, IAnglePitch = 0.0, DAnglePitch = 0.01;

// Gyro low-pass filter: 0 = none, closer to 1 = more filtering (more lag)
#define GYRO_LPF 0.7

// ================================================================
// HARDWARE OBJECTS
// ================================================================
Servo mot1, mot2, mot3, mot4;
RF24 radio(NRF_CE, NRF_CSN);
const byte address[6] = "DRONE";

struct ControlData {
  int roll;
  int pitch;
  int throttle;
  int yaw;
  bool armed;
};

// ================================================================
// SHARED BETWEEN CORES
// ================================================================
portMUX_TYPE mux = portMUX_INITIALIZER_UNLOCKED;
volatile int ReceiverValue[5] = {1500, 1500, 1000, 1500, 0};
volatile bool receiverArmed   = false;
volatile unsigned long lastReceivedMs = 0;

enum FlightState { STATE_DISARMED, STATE_ARMING, STATE_ARMED, STATE_FAILSAFE };
volatile FlightState flightState = STATE_DISARMED;

// ================================================================
// IMU STATE
// ================================================================
float RateCalibrationRoll = 0, RateCalibrationPitch = 0, RateCalibrationYaw = 0;
float AccXCal = -0.004, AccYCal = -0.063;   // measured accelerometer offsets
float angleOffsetRoll = 0, angleOffsetPitch = 0;
float complementaryAngleRoll = 0, complementaryAnglePitch = 0;
float RateRoll, RatePitch, RateYaw;
float AccX, AccY, AccZ;
float AngleRoll, AnglePitch;
float t = (float)LOOP_TIME_US / 1000000.0;

// ================================================================
// PID STATE
// ================================================================
float ItermRoll, ItermPitch, ItermYaw;
float PrevErrorRateRoll, PrevErrorRatePitch, PrevErrorRateYaw;
float PrevErrorAngleRoll, PrevErrorAnglePitch;
float ItermAngleRoll, ItermAnglePitch;
float DesiredRateRoll, DesiredRatePitch;
float InputRoll, InputPitch, InputYaw, InputThrottle;
float MotorInput1, MotorInput2, MotorInput3, MotorInput4;

AsyncWebServer server(80);
bool reloadPID = false;

// ================================================================
// SPIFFS (PID gains stored in flash)
// ================================================================
String readFile(fs::FS &fs, const char* path) {
  File file = fs.open(path, "r");
  if (!file || file.isDirectory()) return String();
  String c; while (file.available()) c += (char)file.read();
  file.close(); return c;
}
void writeFile(fs::FS &fs, const char* path, const char* msg) {
  File file = fs.open(path, "w");
  if (!file) return;
  file.print(msg); file.close();
}
void loadPIDFromSPIFFS() {
  float v;
  v = readFile(SPIFFS,"/pGain.txt").toFloat();  if(v>0){PRateRoll=v;  PRatePitch=v;}
  v = readFile(SPIFFS,"/iGain.txt").toFloat();  if(v>0){IRateRoll=v;  IRatePitch=v;}
  v = readFile(SPIFFS,"/dGain.txt").toFloat();  if(v>0){DRateRoll=v;  DRatePitch=v;}
  v = readFile(SPIFFS,"/pAGain.txt").toFloat(); if(v>0){PAngleRoll=v; PAnglePitch=v;}
  v = readFile(SPIFFS,"/iAGain.txt").toFloat(); if(v>0){IAngleRoll=v; IAnglePitch=v;}
  v = readFile(SPIFFS,"/dAGain.txt").toFloat(); if(v>0){DAngleRoll=v; DAnglePitch=v;}
  v = readFile(SPIFFS,"/pYaw.txt").toFloat();   if(v>0) PRateYaw=v;
  v = readFile(SPIFFS,"/iYaw.txt").toFloat();   if(v>0) IRateYaw=v;
  v = readFile(SPIFFS,"/dYaw.txt").toFloat();   if(v>0) DRateYaw=v;
  Serial.println("PID loaded from SPIFFS");
}

// ================================================================
// PID TUNING WEB PAGE (connect to DRONE_FC, open 192.168.4.1)
// ================================================================
const char index_html[] PROGMEM = R"rawliteral(
<!DOCTYPE HTML><html><head>
<title>Drone PID Tuning</title>
<meta name="viewport" content="width=device-width, initial-scale=1">
<style>
  body{font-family:monospace;background:#111;color:#0f0;padding:20px;}
  h2{color:#4af;} h3{color:#fa0;margin-top:16px;} form{margin:6px 0;}
  input[type=number]{background:#222;color:#0f0;border:1px solid #0f0;padding:4px;width:100px;}
  input[type=submit]{background:#0f0;color:#000;border:none;padding:6px 12px;cursor:pointer;font-weight:bold;}
  .note{color:#fa0;font-size:12px;margin-top:20px;}
</style>
<script>function submitMessage(){setTimeout(function(){document.location.reload(false);},500);}</script>
</head><body>
<h2>DRONE PID TUNING</h2>
<h3>RATE PID (inner loop)</h3>
<form action="/get" target="hf">P Rate Roll/Pitch (now: %pGain%): <input type="number" step="any" name="pGain"><input type="submit" value="Set" onclick="submitMessage()"></form>
<form action="/get" target="hf">I Rate Roll/Pitch (now: %iGain%): <input type="number" step="any" name="iGain"><input type="submit" value="Set" onclick="submitMessage()"></form>
<form action="/get" target="hf">D Rate Roll/Pitch (now: %dGain%): <input type="number" step="any" name="dGain"><input type="submit" value="Set" onclick="submitMessage()"></form>
<h3>ANGLE PID (outer loop)</h3>
<form action="/get" target="hf">P Angle Roll/Pitch (now: %pAGain%): <input type="number" step="any" name="pAGain"><input type="submit" value="Set" onclick="submitMessage()"></form>
<form action="/get" target="hf">I Angle Roll/Pitch (now: %iAGain%): <input type="number" step="any" name="iAGain"><input type="submit" value="Set" onclick="submitMessage()"></form>
<form action="/get" target="hf">D Angle Roll/Pitch (now: %dAGain%): <input type="number" step="any" name="dAGain"><input type="submit" value="Set" onclick="submitMessage()"></form>
<h3>YAW PID</h3>
<form action="/get" target="hf">P Yaw (now: %pYaw%): <input type="number" step="any" name="pYaw"><input type="submit" value="Set" onclick="submitMessage()"></form>
<form action="/get" target="hf">I Yaw (now: %iYaw%): <input type="number" step="any" name="iYaw"><input type="submit" value="Set" onclick="submitMessage()"></form>
<form action="/get" target="hf">D Yaw (now: %dYaw%): <input type="number" step="any" name="dYaw"><input type="submit" value="Set" onclick="submitMessage()"></form>
<br><form action="/reload" target="hf"><input type="submit" value="APPLY PID NOW"></form>
<p class="note">Set values, then press APPLY. Saved to flash, no reboot needed.</p>
<iframe style="display:none" name="hf"></iframe>
</body></html>)rawliteral";

String processor(const String& var) {
  if(var=="pGain")  return readFile(SPIFFS,"/pGain.txt");
  if(var=="iGain")  return readFile(SPIFFS,"/iGain.txt");
  if(var=="dGain")  return readFile(SPIFFS,"/dGain.txt");
  if(var=="pAGain") return readFile(SPIFFS,"/pAGain.txt");
  if(var=="iAGain") return readFile(SPIFFS,"/iAGain.txt");
  if(var=="dAGain") return readFile(SPIFFS,"/dAGain.txt");
  if(var=="pYaw")   return readFile(SPIFFS,"/pYaw.txt");
  if(var=="iYaw")   return readFile(SPIFFS,"/iYaw.txt");
  if(var=="dYaw")   return readFile(SPIFFS,"/dYaw.txt");
  return String();
}

// ================================================================
// IMU: raw registers -> rates -> low-pass -> complementary filter
// ================================================================
void readIMU() {
  // Accelerometer (DLPF 10 Hz, +-8 g)
  Wire.beginTransmission(0x68); Wire.write(0x1A); Wire.write(0x05); Wire.endTransmission();
  Wire.beginTransmission(0x68); Wire.write(0x1C); Wire.write(0x10); Wire.endTransmission();
  Wire.beginTransmission(0x68); Wire.write(0x3B); Wire.endTransmission();
  Wire.requestFrom(0x68, 6);
  int16_t AxLSB = Wire.read()<<8|Wire.read();
  int16_t AyLSB = Wire.read()<<8|Wire.read();
  int16_t AzLSB = Wire.read()<<8|Wire.read();

  // Gyro (+-500 deg/s -> 65.5 LSB per deg/s)
  Wire.beginTransmission(0x68); Wire.write(0x1B); Wire.write(0x8); Wire.endTransmission();
  Wire.beginTransmission(0x68); Wire.write(0x43); Wire.endTransmission();
  Wire.requestFrom(0x68, 6);
  int16_t Gx = Wire.read()<<8|Wire.read();
  int16_t Gy = Wire.read()<<8|Wire.read();
  int16_t Gz = Wire.read()<<8|Wire.read();

  float rawRoll  = (float)Gy/65.5 - RateCalibrationRoll;
  float rawPitch = (float)Gx/65.5 - RateCalibrationPitch;
  float rawYaw   = (float)Gz/65.5 - RateCalibrationYaw;

  // First-order low-pass on gyro
  RateRoll  = GYRO_LPF*RateRoll  + (1.0-GYRO_LPF)*rawRoll;
  RatePitch = GYRO_LPF*RatePitch + (1.0-GYRO_LPF)*rawPitch;
  RateYaw   = GYRO_LPF*RateYaw   + (1.0-GYRO_LPF)*rawYaw;

  // Angles from gravity vector (+-8 g -> 4096 LSB per g)
  AccX = (float)AyLSB/4096 - AccXCal;
  AccY = (float)AxLSB/4096 - AccYCal;
  AccZ = (float)AzLSB/4096;
  AngleRoll  = -atan(AccY/sqrt(AccX*AccX+AccZ*AccZ))*57.29 - angleOffsetRoll;
  AnglePitch = -atan(AccX/sqrt(AccY*AccY+AccZ*AccZ))*57.29 + angleOffsetPitch;

  // Complementary filter: trust gyro short-term, accelerometer long-term
  complementaryAngleRoll  = 0.991*(complementaryAngleRoll  + RateRoll *t) + 0.009*AngleRoll;
  complementaryAnglePitch = 0.991*(complementaryAnglePitch + RatePitch*t) + 0.009*AnglePitch;
  complementaryAngleRoll  = constrain(complementaryAngleRoll,  -45, 45);
  complementaryAnglePitch = constrain(complementaryAnglePitch, -45, 45);
}

// ================================================================
// PID with integral clamping (anti-windup) and output limit
// ================================================================
float calcPID(float error, float &iTerm, float &prevError,
              float P, float I, float D, float dt) {
  float pTerm = P * error;
  iTerm += I * error * dt;
  iTerm  = constrain(iTerm, -400, 400);
  float dTerm = D * (error - prevError) / dt;
  prevError = error;
  return constrain(pTerm + iTerm + dTerm, -400, 400);
}

void runPID() {
  float desiredAngleRoll  = 0.1  * (ReceiverValue[0] - 1500);  // +-50 deg
  float desiredAnglePitch = 0.1  * (ReceiverValue[1] - 1500);
  InputThrottle           = ReceiverValue[2];
  float desiredRateYaw    = 0.15 * (ReceiverValue[3] - 1500);

#if ANGLE_MODE == 0
  // Outer loop: angle error -> desired rotation rate
  DesiredRateRoll  = calcPID(desiredAngleRoll  - complementaryAngleRoll,  ItermAngleRoll,  PrevErrorAngleRoll,
                             PAngleRoll,  IAngleRoll,  DAngleRoll,  t);
  DesiredRatePitch = calcPID(desiredAnglePitch - complementaryAnglePitch, ItermAnglePitch, PrevErrorAnglePitch,
                             PAnglePitch, IAnglePitch, DAnglePitch, t);
#else
  DesiredRateRoll  = 0.15 * (ReceiverValue[0] - 1500);
  DesiredRatePitch = 0.15 * (ReceiverValue[1] - 1500);
#endif

  // Inner loop: rate error -> motor correction
  InputRoll  = calcPID(DesiredRateRoll  - RateRoll,  ItermRoll,  PrevErrorRateRoll,  PRateRoll,  IRateRoll,  DRateRoll,  t);
  InputPitch = calcPID(DesiredRatePitch - RatePitch, ItermPitch, PrevErrorRatePitch, PRatePitch, IRatePitch, DRatePitch, t);
  InputYaw   = calcPID(desiredRateYaw   - RateYaw,   ItermYaw,   PrevErrorRateYaw,   PRateYaw,   IRateYaw,   DRateYaw,   t);
}

void resetPID() {
  ItermRoll = ItermPitch = ItermYaw = 0;
  ItermAngleRoll = ItermAnglePitch = 0;
  PrevErrorRateRoll = PrevErrorRatePitch = PrevErrorRateYaw = 0;
  PrevErrorAngleRoll = PrevErrorAnglePitch = 0;
  InputRoll = InputPitch = InputYaw = 0;
}

// ================================================================
// MOTOR MIXING (X frame)
// ================================================================
void runMotors() {
  if (flightState == STATE_ARMED) {
    InputThrottle = constrain(InputThrottle, 1000, THROTTLE_MAX);
    if (InputThrottle > THROTTLE_IDLE) {
      MotorInput1 = InputThrottle - InputRoll - InputPitch - InputYaw; // FR CCW
      MotorInput2 = InputThrottle - InputRoll + InputPitch + InputYaw; // RR CW
      MotorInput3 = InputThrottle + InputRoll + InputPitch - InputYaw; // RL CCW
      MotorInput4 = InputThrottle + InputRoll - InputPitch + InputYaw; // FL CW
      MotorInput1 = constrain(MotorInput1,               1000, 1999);
      MotorInput2 = constrain(MotorInput2 + MOT2_OFFSET, 1000, 1999);
      MotorInput3 = constrain(MotorInput3,               1000, 1999);
      MotorInput4 = constrain(MotorInput4,               1000, 1999);
    } else {
      MotorInput1 = MotorInput3 = MotorInput4 = THROTTLE_IDLE;
      MotorInput2 = THROTTLE_IDLE + MOT2_OFFSET;
      resetPID();
    }
  } else {
    MotorInput1 = MotorInput2 = MotorInput3 = MotorInput4 = THROTTLE_CUTOFF;
    resetPID();
  }
  mot1.writeMicroseconds((int)MotorInput1);
  mot2.writeMicroseconds((int)MotorInput2);
  mot3.writeMicroseconds((int)MotorInput3);
  mot4.writeMicroseconds((int)MotorInput4);
}

// ================================================================
// ARM / DISARM / FAILSAFE STATE MACHINE
// ================================================================
void updateArmState() {
  bool rxArmed  = receiverArmed;
  bool rxLowThr = ReceiverValue[2] < ARM_THROTTLE_MAX;
  bool fsActive = (millis() - lastReceivedMs) > FAILSAFE_MS;

  if (fsActive && flightState == STATE_ARMED) {
    flightState = STATE_FAILSAFE;
    Serial.println("FAILSAFE - no signal, motors cut");
    return;
  }
  if (fsActive) return;

  if (flightState == STATE_FAILSAFE) {          // signal back -> stay safe
    flightState = STATE_DISARMED;
    Serial.println("Signal returned - DISARMED");
    return;
  }
  if (flightState == STATE_DISARMED && rxArmed && rxLowThr) {
    flightState = STATE_ARMED;
    Serial.println("ARMED");
  }
  if (flightState == STATE_ARMED && !rxArmed) {
    flightState = STATE_DISARMED;
    Serial.println("DISARMED");
    resetPID();
  }
}

// ================================================================
// CORE 1: FLIGHT TASK (250 Hz)
// ================================================================
void flightTask(void* param) {
  Wire.setClock(400000);
  Wire.begin(21, 22);
  delay(250);
  Wire.beginTransmission(0x68); Wire.write(0x6B); Wire.write(0x00); Wire.endTransmission();
  delay(100);

  // Gyro bias: average 500 samples while still
  Serial.println("CALIBRATING - keep still...");
  float sumX=0, sumY=0, sumZ=0;
  for (int i=0; i<500; i++) {
    Wire.beginTransmission(0x68); Wire.write(0x1B); Wire.write(0x8); Wire.endTransmission();
    Wire.beginTransmission(0x68); Wire.write(0x43); Wire.endTransmission();
    Wire.requestFrom(0x68,6);
    sumX += (int16_t)(Wire.read()<<8|Wire.read());
    sumY += (int16_t)(Wire.read()<<8|Wire.read());
    sumZ += (int16_t)(Wire.read()<<8|Wire.read());
    delay(4);
  }
  RateCalibrationRoll  = sumY/500.0/65.5;
  RateCalibrationPitch = sumX/500.0/65.5;
  RateCalibrationYaw   = sumZ/500.0/65.5;

  // Level offset: average 100 accelerometer samples
  float aX=0, aY=0, aZ=0;
  for (int i=0; i<100; i++) {
    Wire.beginTransmission(0x68); Wire.write(0x1C); Wire.write(0x10); Wire.endTransmission();
    Wire.beginTransmission(0x68); Wire.write(0x3B); Wire.endTransmission();
    Wire.requestFrom(0x68,6);
    aX += (int16_t)(Wire.read()<<8|Wire.read());
    aY += (int16_t)(Wire.read()<<8|Wire.read());
    aZ += (int16_t)(Wire.read()<<8|Wire.read());
    delay(4);
  }
  float iAX = (aY/100.0)/4096.0 - AccXCal;
  float iAY = (aX/100.0)/4096.0 - AccYCal;
  float iAZ = (aZ/100.0)/4096.0;
  angleOffsetRoll  = -atan(iAY/sqrt(iAX*iAX+iAZ*iAZ))*57.29;
  angleOffsetPitch = -atan(iAX/sqrt(iAY*iAY+iAZ*iAZ))*57.29;
  Serial.println("IMU READY");

  // ESCs at 500 Hz
  ESP32PWM::allocateTimer(0); ESP32PWM::allocateTimer(1);
  ESP32PWM::allocateTimer(2); ESP32PWM::allocateTimer(3);
  delay(1000);
  mot1.attach(MOT1_PIN,1000,2000); delay(1000); mot1.setPeriodHertz(500);
  mot2.attach(MOT2_PIN,1000,2000); delay(1000); mot2.setPeriodHertz(500);
  mot3.attach(MOT3_PIN,1000,2000); delay(1000); mot3.setPeriodHertz(500);
  mot4.attach(MOT4_PIN,1000,2000); delay(1000); mot4.setPeriodHertz(500);
  mot1.writeMicroseconds(1000); mot2.writeMicroseconds(1000);
  mot3.writeMicroseconds(1000); mot4.writeMicroseconds(1000);
  delay(500);

  uint32_t loopTimer = micros();
  while (true) {
    while (micros() - loopTimer < LOOP_TIME_US);   // fixed 4 ms loop
    loopTimer = micros();

    if (reloadPID) { loadPIDFromSPIFFS(); reloadPID = false; }
    readIMU();
    updateArmState();
    if (flightState == STATE_ARMED) runPID();
    runMotors();

    static uint32_t lastDebug = 0;
    if (millis() - lastDebug > 500) {
      lastDebug = millis();
      const char* states[] = {"DISARMED","ARMING","ARMED","FAILSAFE"};
      Serial.printf("[%s] R:%.1f P:%.1f | M1:%d M2:%d M3:%d M4:%d\n", states[flightState],
        complementaryAngleRoll, complementaryAnglePitch,
        (int)MotorInput1, (int)MotorInput2, (int)MotorInput3, (int)MotorInput4);
    }
  }
}

// ================================================================
// CORE 0: COMMUNICATION TASK
// ================================================================
void commTask(void* param) {
  if (radio.begin()) {
    radio.openReadingPipe(0, address);
    radio.setPALevel(RF24_PA_MAX);
    radio.setDataRate(RF24_250KBPS);
    radio.startListening();
    Serial.println("NRF24L01 READY");
  } else {
    Serial.println("NRF24L01 NOT FOUND - check wiring");
  }

  WiFi.softAP(ssid, password);
  server.on("/", HTTP_GET, [](AsyncWebServerRequest* req){ req->send_P(200,"text/html",index_html,processor); });
  server.on("/get", HTTP_GET, [](AsyncWebServerRequest* req){
    const char* keys[] = {"pGain","iGain","dGain","pAGain","iAGain","dAGain","pYaw","iYaw","dYaw"};
    for (const char* k : keys) {
      if (req->hasParam(k)) {
        String path = String("/") + k + ".txt";
        writeFile(SPIFFS, path.c_str(), req->getParam(k)->value().c_str());
        break;
      }
    }
    req->send(200,"text/plain","OK");
  });
  server.on("/reload", HTTP_GET, [](AsyncWebServerRequest* req){ reloadPID = true; req->send(200,"text/plain","PID applied"); });
  server.begin();

  while (true) {
    if (radio.available()) {
      ControlData received;
      radio.read(&received, sizeof(received));
      portENTER_CRITICAL(&mux);
      ReceiverValue[0] = constrain(received.roll,     1000, 2000);
      ReceiverValue[1] = constrain(received.pitch,    1000, 2000);
      ReceiverValue[2] = constrain(received.throttle, 1000, 2000);
      ReceiverValue[3] = constrain(received.yaw,      1000, 2000);
      receiverArmed    = received.armed;
      lastReceivedMs   = millis();
      portEXIT_CRITICAL(&mux);
    }
    delay(2);
  }
}

void setup() {
  Serial.begin(115200);
  delay(500);
  if (!SPIFFS.begin(true)) Serial.println("SPIFFS failed!");
  loadPIDFromSPIFFS();
  xTaskCreatePinnedToCore(flightTask, "FlightTask", 8192, NULL, 2, NULL, 1);
  xTaskCreatePinnedToCore(commTask,   "CommTask",   8192, NULL, 1, NULL, 0);
}

void loop() { vTaskDelete(NULL); }
