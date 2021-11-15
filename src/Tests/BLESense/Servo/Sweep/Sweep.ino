/* Sweep
 by BARRAGAN <http://barraganstudio.com>
 This example code is in the public domain.

 modified 8 Nov 2013
 by Scott Fitzgerald
 http://www.arduino.cc/en/Tutorial/Sweep
*/

#include <Servo.h>
#define REFRESH_INTERVAL 3003 // KST servos operate at 333Hz --> 1/333 * 1e6

Servo myservo;  // create servo object to control a servo
// twelve servo objects can be created on most boards

int pos = 0;    // variable to store the servo position

void setup() {
  myservo.attach(D9);  // attaches the servo on pin 9 to the servo object
  myservo.writeMicroseconds(1500);
}

void loop() {
  for (pos = 800; pos <= 2200; pos += 1) { // goes from 0 degrees to 180 degrees
    // in steps of 1 degree
    myservo.writeMicroseconds(pos);              // tell servo to go to position in variable 'pos'
    delay(1);                       // waits 15ms for the servo to reach the position
  }
  for (pos = 2200; pos >= 800; pos -= 1) { // goes from 180 degrees to 0 degrees
    myservo.writeMicroseconds(pos);              // tell servo to go to position in variable 'pos'
    delay(1);                       // waits 15ms for the servo to reach the position
  }
  myservo.writeMicroseconds(1520);
  delay(5000);
}

