#include <Servo.h>
#define LED_PIN 13

Servo a, b;

// Default central position of servos A and B
// Can be set to different, calibrated values by a command
int centerPosA = 90;
int centerPosB = 90;

// Limit the angle range and protect servo and device from program bugs
const int minAngle = 7;  // smaller values interpreted as 30
const int maxAngle = 132; // larger values interpreted as 102

void setup()
{
  Serial.begin(115200);
  Serial.setTimeout(80);

  // use ports 9 and 10 (both featuring PWM) for the two servos
  a.attach(9);
  b.attach(10);
  delay(100);   // magic delay to make the next two commands work
  a.write(centerPosA);
  b.write(centerPosB);
  pinMode(LED_PIN, OUTPUT);
}

// Simple command interpreter
// Each command, which is a sequence of bytes, starts with 255, 
// followed by the action and its arguments, and ends with 0
// Action numbers (23-25,99) are for historical reasons
void loop()
{
  int c;
  int i;
  byte buf[10];
  
  if (Serial.available() > 0) {
    Serial.readBytesUntil(0, (char *)buf, 5);
    // if (buf[0] != 255) break;
    c = buf[1];
   
    switch (c) {
      case 23:  // set center angles of two servos
        centerPosA = buf[2];
        centerPosB = buf[3];
        break;
      case 24:  // move servo A
        c = buf[2];
        if (c < minAngle) c = minAngle;
        else if (c > maxAngle) c = maxAngle;
        a.write(centerPosA + c - 90);
        break;
      case 25:  // move servo B
        c = buf[2];
        if (c < minAngle) c = minAngle;
        else if (c > maxAngle) c = maxAngle;
        b.write(centerPosB + c - 90);
        break;
      case 99:  // blink on-board LED (for testing command interpreter)
        for (i=0; i<buf[2]; i++) {
          digitalWrite(LED_PIN, HIGH);
          delay(100);
          digitalWrite(LED_PIN, LOW);
          delay(233);
        }
        break;
      default:
          digitalWrite(LED_PIN, HIGH);
          delay(750);
          digitalWrite(LED_PIN, LOW);
          delay(250);
        break;    
      }
  }
}
