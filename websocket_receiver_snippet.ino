// Drone side of the phone controller (excerpt from the WiFi flight code).
// ESP32 runs a WiFi access point "DRONE_FC" and a WebSocket server on port 81.
// The phone sends "roll,pitch,throttle,yaw,mission" every 50 ms.

void onWebSocketEvent(uint8_t client, WStype_t type, uint8_t* payload, size_t length) {
  if (type == WStype_CONNECTED) {
    Serial.println("Controller connected!");
  }
  else if (type == WStype_DISCONNECTED) {
    // Safety: phone disconnected -> sticks centred, throttle to zero
    Serial.println("Controller disconnected! Motors stopping.");
    ReceiverValue[0] = 1500;
    ReceiverValue[1] = 1500;
    ReceiverValue[2] = 1000;
    ReceiverValue[3] = 1500;
  }
  else if (type == WStype_TEXT) {
    int ch1, ch2, ch3, ch4, mission;
    if (sscanf((char*)payload, "%d,%d,%d,%d,%d", &ch1, &ch2, &ch3, &ch4, &mission) == 5) {
      ReceiverValue[0] = constrain(ch1, 1000, 2000); // Roll
      ReceiverValue[1] = constrain(ch2, 1000, 2000); // Pitch
      ReceiverValue[2] = constrain(ch3, 1000, 2000); // Throttle
      ReceiverValue[3] = constrain(ch4, 1000, 2000); // Yaw
      ReceiverValue[5] = mission;
    }
  }
}
