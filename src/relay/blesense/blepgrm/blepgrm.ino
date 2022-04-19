#include <Wire.h>
#include <Arduino_LSM9DS1.h>
#include <Servo.h>

#define REFRESH_INTERVAL 3003 // KST servos operate at 333Hz --> 1/333 * 1e6
#define SERVODISABLEPERIOD 0 // Either 0 orREFRESH_INTERVAL ? 

#define I2C_SLAVE_ADDRESS 11 
#define NSERVOS 2
#define SERVOUPDATEPERIOD 20 // in ms
#define COMMTIMEOUT 100

float x, y, z;
float wx, wy, wz;
int n;
bool servoreported;
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
  int maxChange;
  Servo curservo;
};


struct StateStruct {
  float accel[3];            // 4*3 = 12
  float w[3];                 // 12
  uint16_t servopos[NSERVOS]; // 2*2 = 4
  uint8_t mode;               // 1
                              //------
                              // 29
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
    case 64: servo->minlim = int(val); break;
    case 65: servo->maxlim = int(val); break;
    case 66: servo->maxSpeed = float(val)/1000.; 
             servo->maxChange = servo->maxSpeed * SERVOUPDATEPERIOD;
             break;
  }


}

void setCtrlVals(short selector, short val1, short val2){
  
  switch(selector) {
    case 16: servos[0].speed = int(val1); 
             servos[1].speed = int(val2);
	     Serial.println("SET SPEEDS:");
             Serial.println(servos[0].speed);
             Serial.println();

             break;
    // TODO: set motor speeds:
  }
}

//void 

void attachServos(){
  // Attach all servos:
  for (int i=0; i<NSERVOS; i++){
    servos[i].curservo.attach(servopins[i]);  
    servos[i].prevUpdate = millis();
  }
  servosattached = true;
  Serial.println("Servos attached.");
}
void detachServos(){
  // Detach all servos:
  for (int i=0; i<NSERVOS; i++){
    //int middle = (servos[i].minlim + servos[i].maxlim)/2;
    //servos[i].curservo.writeMicroseconds(SERVODISABLEPERIOD); //TODO: 0 or 3003 do not disable any other ideas?
    //servos[i].curservo.writeMicroseconds(middle); 
    servos[i].curservo.detach();
  }
  servosattached = false;
  Serial.println("Servos detached.");
}

void setMode(short selectmode){
  
  Serial.println("Set mode:");
  Serial.print("mode: "); Serial.println(mode);
  Serial.print("new mode: "); Serial.println(selectmode);
  Serial.print("Setting mode: "); Serial.println(selectmode);

  //TODO: get rid of mode and only use state.mode
  state.mode = selectmode;


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
    // Pick the servo:
    ServoCtrlStruct *servo = &servos[i];

    // Initialize the various servo paramaters to some values:
    servo->prevUpdate = 0;
    servo->speed = 0;
    servo->minlim = 1400;
    servo->maxlim = 1600;
    servo->idx = i;
    servo->pos = 1500;
    servo->maxChange = 10;
    servo->maxSpeed = float(servo->maxlim - servo->minlim)/10.; 
    
    // Disable servo:
    // servo->curservo.writeMicroseconds(1500);
  }
}

void runServos(){
  // Drive allthe servos: 
  for (int i=0;i<NSERVOS;i++){runServo(&servos[i]);}
}

void runServo(ServoCtrlStruct *servo){
  
  
  int sinceUpdate = millis() - servo->prevUpdate; 

  if (SERVOUPDATEPERIOD < sinceUpdate){
    
    // Compute position based on control integer:
    float speedScale = float(servo->speed) / 32768.;
    float speed = servo->maxSpeed * speedScale; 
    int16_t updatePos = sinceUpdate * speed;
    
    updatePos = constrain(updatePos, -servo->maxChange, servo->maxChange);
 
    if (mode != 0) { 
      if (servoreported){
        Serial.print("Updating servo : "); Serial.println(servo->idx);
        Serial.println(servo->speed);
        Serial.println(speedScale);
        Serial.println(servo->maxSpeed);
        Serial.println(speed);
        Serial.println(updatePos);
        Serial.println("");
        servoreported = false;
      }

      // Compute the new pos in terms of microseconds:
      servo->pos = constrain(servo->pos + updatePos, servo->minlim, servo->maxlim); 

      // Write the new servo pos to state struct:
      state.servopos[servo->idx] = servo->pos;
    
      // Update the servo pos:
      servo->curservo.writeMicroseconds(servo->pos); 
      
      // Store the update time: (Risk of overflow in??)
      if (abs(updatePos)>0) servo->prevUpdate = millis();
    }
    
  }
}

void reportServo(ServoCtrlStruct *servo){
  
  Serial.println("Reporting servo:");
  Serial.print("Servo idx      "); Serial.println(servo->idx);
  Serial.print("Servo speed    "); Serial.println(servo->speed);
  Serial.print("Servo maxspeed "); Serial.println(servo->maxSpeed);
  Serial.print("Servo pos      "); Serial.println(servo->pos);
  Serial.print("Servo min      "); Serial.println(servo->minlim);
  Serial.print("Servo max      "); Serial.println(servo->maxlim);
  Serial.println("");

  servoreported = true;

}

void showReport(short selector){
  Serial.println("REPORT:");
  Serial.print("Operation mode: "); Serial.println(mode);
  Serial.print("Servos attahed: "); Serial.println(servosattached);
  
  for (int i=0; i<NSERVOS; i++){
    reportServo(&servos[i]);
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

  // Run the servos according to ServoCtrlStruct:
  runServos();

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

  Serial.print("Selector : "); Serial.println(selector);

  if (selector<8){ // SET MODE:
    setMode(selector);
  }
  else if (8<=selector & selector < 16){ // Show reports:
    //showReport(selector);
  }
  else if (16<=selector & selector<64){ // Set control input:
    Serial.print("Setting control!");
    setCtrlVals(selector, val1, val2);
  }
  else if (64<=selector){ // SET PARAMS:
    setServoParams(selector, val2, &servos[val1]);
  }

}
