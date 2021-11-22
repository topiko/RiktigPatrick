#include <Wire.h>
#include <Arduino_LSM9DS1.h>
#include <Servo.h>

#define REFRESH_INTERVAL 3003 // KST servos operate at 333Hz --> 1/333 * 1e6
#define SERVODISABLEPERIOD 3003 // Either 0 orREFRESH_INTERVAL ? 

#define I2C_SLAVE_ADDRESS 11 
#define NSERVOS 2
#define SERVOUPDATEPERIOD 20 // in ms
#define COMMTIMEOUT 1000

float x, y, z;
float wx, wy, wz;
int n;
uint32_t lastInput = 0;
int servopins[2] = {D8, D9};
uint8_t mode = 0; // Operation mode: 0 == FAILSAFE
bool servosattached = false;

struct ServoCtrlStruct {
  int prevUpdate;
  int speed;
  int minlim;
  int maxlim;
  int idx;
  uint16_t pos;
  float maxSpeed;
  Servo curservo;
};


struct StateStruct {
  float accel[3];             // 4*3 = 12
  float w[3];                 // 12
  uint16_t servopos[NSERVOS]; // 2*2 = 4
  //byte paddind[4];            // 4
                              //------
                              // 32
};

StateStruct state;
ServoCtrlStruct servos[NSERVOS];

void setup()
{
  Wire.begin(I2C_SLAVE_ADDRESS);
  Wire.setClock(400000L);
  Serial.begin(9600);
  Serial.println("BLESense intitalizing:");
  delay(1000);
  Wire.onRequest(requestEvents);
  Wire.onReceive(receiveEvents);

  if (!IMU.begin()) {
    Serial.println("Failed to initialize IMU!");
    while (1);
  }

  Serial.print("Accelerometer sample rate = ");
  Serial.print(IMU.accelerationSampleRate());
  Serial.println(" Hz");

  // Init servos:
  initServos();
}

void setServoParams(int selector, short val, ServoCtrlStruct *servo){
  // Set servo params:
 
  switch(selector) {
    case 0: servo->minlim = val; break;
    case 1: servo->maxlim = val; break;
    case 2: servo->maxSpeed = val; break;
  }


}

void setCtrlVals(short selector, short val1, short val2){

  switch(selector) {
    case 0: servos[0].speed = val1; 
            servos[1].speed = val2;
            break;
    // TODO: set motor speeds:
  }
}

//void 

void attachServos(){
  // Attach all servos:
  for (int i=0; i<NSERVOS; i++){
    servos[i].curservo.attach(servopins[i]);
  }
  servosattached = true;
  Serial.println("Servos attached.");
}
void detachServos(){
  // Detach all servos:
  for (int i=0; i<NSERVOS; i++){
    servos[i].curservo.writeMicroseconds(SERVODISABLEPERIOD); 
    servos[i].curservo.detach();
  }
  servosattached = false;
  Serial.println("Servos detached.");
}

void setMode(short selectmode){
  
  if (mode==selectmode) return;

  switch (selectmode) {
    // FAILSAFE:
    case 0: mode = 0;

            if (servosattached) detachServos();
            Serial.println("Entering failsafe mode.");
            break;
    // NORMAL:
    case 1: mode = 1;
            if (!servosattached) attachServos();
            Serial.println("Entering normal mode.");
            break;
  }
}

void initServos(){

  for (int i=0; i<NSERVOS; i++){
    // Initialize the various servo paramaters to some values:
    servos[i].prevUpdate = 0;
    servos[i].speed = 0;
    servos[i].minlim = 1400;
    servos[i].maxlim = 1600;
    servos[i].idx = i;
    servos[i].pos = 1500;
    servos[i].maxSpeed = float(servos[i].maxlim - servos[i].minlim)/1000.; 
    
    // Disable servo:
    // servos[i].curservo.writeMicroseconds(1500);
  }
}

void runServo(ServoCtrlStruct *servo){
  
  
  int sinceUpdate = millis() - servo->prevUpdate; 

  if (SERVOUPDATEPERIOD < sinceUpdate){
    
    // Compute position based on control integer:
    float speedScale = float(servo->speed) / 32768.;
    float speed = servo->maxSpeed * speedScale; 
    int16_t updatePos = sinceUpdate * speed;
    
    // Compute the new pos in terms of microseconds:
    servo->pos = constrain(servo->pos + updatePos, servo->minlim, servo->maxlim); 

    // Write the new servo pos to state struct:
    state.servopos[servo->idx] = servo->pos;
    
    // Update the servo pos:
    if (mode != 0) {
      Serial.println("Servo pos: ");
      Serial.println(servo->pos);
      Serial.println(servo->speed);
      Serial.println(speedScale);
      Serial.println(speed);
      Serial.println(updatePos);
      Serial.println();

      servo->curservo.writeMicroseconds(servo->pos); 
      //servo->curservo.writeMicroseconds(1500); 
      
      // Store the update time: (Risk of overflow in??)
      servo->prevUpdate = millis();
    }
    
  }
}

void loop(){
  
  // Get acceleration:
  while (IMU.accelerationAvailable()){
    IMU.readAcceleration(x, y, z);
    state.accel[0] = x;
    state.accel[1] = y;
    state.accel[2] = z;
  }

  // Get gyro:
  while (IMU.gyroscopeAvailable()){
    IMU.readGyroscope(wx, wy, wz);
    state.w[0] = wx;
    state.w[1] = wy;
    state.w[2] = wz;
  }

  // Drive the servos: 
  for (int i=0;i<NSERVOS;i++){runServo(&servos[i]);}

  //TODO: Drive motors.
  
  if ((millis() - lastInput) > COMMTIMEOUT){
    //Go to failsafe if too long since previous ctrl input.
    if (mode != 0) setMode(0);
  }
}

void requestEvents(){
  // Send state array to i2c:
  // Serial.println("Event requested!");
  Wire.write((byte*) &state, sizeof(state));
}

void receiveEvents(int numBytes){


  
  lastInput = millis();

  int bufferlen = 6;
  byte buffer[bufferlen];
  Wire.readBytes(buffer, bufferlen);


  short selector = buffer[1];
  short val1 = (buffer[2] << 8) | buffer[3]; // Note: conversion from big-endian to little endian system
  short val2 = (buffer[4] << 8) | buffer[5]; // Note: conversion from big-endian to little endian system

  if (selector<16){ // SET MODE:
    setMode(selector);
    //mode = selector;
  }
  else if (16<=selector & selector<64){
    selector = selector - 16;
    setCtrlVals(selector, val1, val2);
  }
  else if (64<=selector){ // SET PARAMS:
    selector = selector - 64;
    setServoParams(selector, val2, &servos[val1]);
  }

}
