#include <Arduino.h>
#include <Servo.h>

Servo servos[6];
const int servoPins[6] = {2, 3, 4, 5, 6, 7};
const int initialPositions[6] = {90, 90, 90, 90, 90, 90};  // 初始位置，可以根据需要调整

void setup() {
  Serial.begin(115200);
  
  for (int i = 0; i < 6; i++) {
    servos[i].attach(servoPins[i]);
    servos[i].write(initialPositions[i]);  // 将舵机移动到初始位置
    delay(500);  // 给予足够的时间移动到初始位置
  }
  
  Serial.println("Arduino initialized and ready.");
}

void loop() {
  if (Serial.available() >= 12) {  // 6 joints * 2 bytes per angle
    for (int i = 0; i < 6; i++) {
      int angle = Serial.read() << 8 | Serial.read();  // Read 2 bytes and combine them
      angle = constrain(angle, 0, 180);  // Ensure angle is within 0-180 range
      servos[i].write(angle);
    }
  }
}