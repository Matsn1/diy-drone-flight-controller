// DIY radio controller (transmitter): ESP32 + NRF24L01 + 2 joysticks + arm switch
#include <SPI.h>
#include <RF24.h>

RF24 radio(4, 5);                 // CE, CSN
const byte address[6] = "DRONE";

// Joystick pins
#define JOY1_VRX 34  // Throttle
#define JOY1_VRY 35  // Yaw
#define JOY2_VRX 25  // Roll
#define JOY2_VRY 26  // Pitch
#define JOY1_SW  32
#define JOY2_SW  33
#define ARM_SW   27

struct ControlData {   // must match the drone side exactly
  int roll;
  int pitch;
  int throttle;
  int yaw;
  bool armed;
};

ControlData data;

int mapJoystick(int raw) {        // 12-bit ADC -> 1000..2000 us (RC standard)
  return map(raw, 0, 4095, 1000, 2000);
}

void setup() {
  Serial.begin(115200);
  pinMode(JOY1_SW, INPUT_PULLUP);
  pinMode(JOY2_SW, INPUT_PULLUP);
  pinMode(ARM_SW,  INPUT_PULLDOWN);

  radio.begin();
  radio.openWritingPipe(address);
  radio.setPALevel(RF24_PA_MAX);
  radio.setDataRate(RF24_250KBPS); // lowest rate = longest range
  radio.stopListening();
  Serial.println("Controller ready!");
}

void loop() {
  data.throttle = mapJoystick(analogRead(JOY1_VRX));
  data.yaw      = mapJoystick(analogRead(JOY1_VRY));
  data.roll     = mapJoystick(analogRead(JOY2_VRX));
  data.pitch    = mapJoystick(analogRead(JOY2_VRY));
  data.armed    = digitalRead(ARM_SW);

  if (!data.armed) data.throttle = 1000;   // safety: no throttle while disarmed

  bool sent = radio.write(&data, sizeof(data));

  Serial.print("Roll:");      Serial.print(data.roll);
  Serial.print(" Pitch:");    Serial.print(data.pitch);
  Serial.print(" Throttle:"); Serial.print(data.throttle);
  Serial.print(" Yaw:");      Serial.print(data.yaw);
  Serial.print(" Armed:");    Serial.print(data.armed ? "YES" : "NO");
  Serial.println(sent ? " SENT" : " FAILED");

  delay(20); // 50 Hz
}
