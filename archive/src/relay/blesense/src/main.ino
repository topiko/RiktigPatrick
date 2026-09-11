#include <Wire.h>
#include <Arduino_LSM9DS1.h>
#include <Servo.h>
#include <AccelStepper.h>

#define REFRESH_INTERVAL 3003 // KST servos operate at 333Hz --> 1/333 * 1e6
#define SERVODISABLEPERIOD 0 // Either 0 orREFRESH_INTERVAL ? 

#define I2C_SLAVE_ADDRESS 11 
#define NSERVOS 2
#define SERVOUPDATEPERIOD 20 // in ms
#define COMMTIMEOUT 300
#define I2CBUFFERLEN 10
#define MOTORENABLE 5

float x, y, z;
float wx, wy, wz;
//int n;
bool servoreported;
uint32_t lastInput = 0;
static uint8_t servopins[2] = {D8, D9};
static uint8_t dirpins[2] = {D5, D7};
static uint8_t steppins[2] = {D4, D6};
uint8_t mode = 0; // Operation mode: 0 == FAILSAFE
bool servosattached = false;

struct ServoCtrlStruct {
  int prevUpdate;
  float speed;
  int minlim;
  int maxlim;
  int idx;
  uint16_t pos;
  float maxSpeed;
  int maxChange;
  Servo curservo;
};

struct MotorCtrlStruct {
  int speed;
  int maxSpeed;
  bool speedUpdate;
  AccelStepper curmotor; 
};

struct StateStruct {
  float accel[3];             // 4*3 = 12
  float w[3];                 // 12
  uint16_t servopos[NSERVOS]; // 2*2 = 4
  uint32_t readtime;          // 4			
  uint8_t mode;               // 1
                              //------
                              // 33
};

StateStruct state;
ServoCtrlStruct servos[NSERVOS];
MotorCtrlStruct motors[2];

void setMode(short selectmode){
  
  //Serial.println("Set mode:");
  //Serial.print("mode: "); Serial.println(mode);
  //Serial.print("new mode: "); Serial.println(selectmode);
  //Serial.print("Setting mode: "); Serial.println(selectmode);

  //TODO: get rid of mode and only use state.mode
  state.mode = selectmode;


  switch (selectmode) {
    // FAILSAFE:
    case 0: mode = 0;
            if (servosattached) detachServos();
            disableMotors();
            //Serial.println("Entering failsafe mode.");
            break;
    // NORMAL:
    case 1: mode = 1;
            if (!servosattached) attachServos();
            enableMotors();
            //Serial.println("Entering normal mode.");
            break;
  }
}


void requestEvents(){
  // Send state array to i2c:
  //Serial.println("Event requested!");
  Wire.write((byte*) &state, sizeof(state));
}

void receiveEvents(int numBytes){

  lastInput = millis();

  byte buffer[I2CBUFFERLEN];
  Wire.readBytes(buffer, I2CBUFFERLEN);


  short selector = buffer[1];
  short val1 = (buffer[2] << 8) | buffer[3]; // Note: conversion from big-endian to little endian system
  short val2 = (buffer[4] << 8) | buffer[5]; // Note: conversion from big-endian to little endian system
  short val3 = (buffer[6] << 8) | buffer[7]; // Note: conversion from big-endian to little endian system
  short val4 = (buffer[8] << 8) | buffer[9]; // Note: conversion from big-endian to little endian system

  //Serial.print("Selector : "); Serial.println(selector);

  if (selector<8){ // SET MODE:
    setMode(selector);
  }
  else if ((8<=selector) & (selector<16)){ // Show reports:
    showReport(selector);
  }
  else if ((16<=selector) & (selector<64)){ // Set control input:
    //Serial.println("Setting control:");
    setCtrlVals(selector, val1, val2, val3, val4);
  }
  else if (64<=selector){ // SET PARAMS:
    setServoParams(selector, val2, &servos[val1]);
  }

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

void setCtrlVals(short selector, short servoSpeed1, short servoSpeed2, short motorSpeed1, short motorSpeed2){
  
  switch(selector) {
    case 16: servos[0].speed = getServoSpeed(&servos[0], servoSpeed1); // int(servoSpeed1); 
             servos[1].speed = getServoSpeed(&servos[1], servoSpeed2); // int(servoSpeed2);
             motors[0].speed = getMotorSpeed(&motors[0], motorSpeed1); // motorSpeed1/  
             motors[1].speed = getMotorSpeed(&motors[1], motorSpeed2); // motorSpeed1/  
	     //Serial.println("SET SPEEDS:");
             //Serial.println(servos[0].speed);
             //Serial.println(servos[1].speed);
             //Serial.println();

             break;
    // TODO: set motor speeds:
  }
}


void attachServos(){
  // Attach all servos:
  for (int i=0; i<NSERVOS; i++){
    servos[i].curservo.attach(servopins[i]);  
    servos[i].prevUpdate = millis();
  }
  servosattached = true;
  Serial.println("Servos attached.");
}

void enableMotors(){
  digitalWrite(MOTORENABLE, HIGH);
}
void disableMotors(){
  digitalWrite(MOTORENABLE, LOW);
  motors[0].speed = 0.;
  motors[1].speed = 0.;
}

void detachServos(){
  // Detach all servos:
  for (int i=0; i<NSERVOS; i++){
    servos[i].curservo.detach();
  }
  servosattached = false;
  Serial.println("Servos detached.");
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
    servo->pos = 1601;
    servo->maxChange = 10;
    servo->maxSpeed = float(servo->maxlim - servo->minlim)/10.; 
  }

}

void initMotors(){

  for (int i=0; i<NSERVOS; i++){
    // Pick the servo:
    MotorCtrlStruct *motor = &motors[i];

    // Initialize the various servo paramaters to some values:
    motor->speed = 0;
    //motor->idx = i;
    motor->maxSpeed = 1000;
    motor->curmotor = AccelStepper(1, steppins[i], dirpins[i]);
    motor->curmotor.setMaxSpeed(motor->maxSpeed);
    //motor->curmotor.setDirPin(5);
  }
  pinMode(OUTPUT, MOTORENABLE);
  disableMotors();
}


float getServoSpeed(ServoCtrlStruct *servo, short speed){

  // Compute position based on control integer:
  float speedScale = float(speed) / 32768.;
  return float(servo->maxSpeed * speedScale); 

  //return speed_f;
}
	
int getMotorSpeed(MotorCtrlStruct *motor, short speed){
  float speedScale = float(speed) / 32768.;
  int newSpeed = int(motor->maxSpeed * speedScale); 
  
  // If speed changed it needs to be updated:
  if (newSpeed != motor->speed) motor->speedUpdate = true;
  
  return newSpeed;
}

void runServo(ServoCtrlStruct *servo){
  
  
  int sinceUpdate = millis() - servo->prevUpdate; 

  if (SERVOUPDATEPERIOD < sinceUpdate){
    
    // Compute position based on control integer:
    // MOved to getServoSpeed...
    //float speedScale = float(servo->speed) / 32768.;
    //float speed = servo->maxSpeed * speedScale; 
    int16_t updatePos = sinceUpdate * servo->speed;
    
    updatePos = constrain(updatePos, -servo->maxChange, servo->maxChange);
    if (abs(updatePos)>0){ 
        if (servoreported){
          Serial.print("Updating servo : "); 
          Serial.println(servo->idx);
          Serial.println(sinceUpdate);
          Serial.println(servo->speed);
          //Serial.println(speedScale);
          Serial.println(servo->maxSpeed);
          //Serial.println(speed);
          Serial.println(servo->pos);
          Serial.println(updatePos);
          Serial.println("");
          servoreported = false;
        }

        // Compute the new pos in terms of microseconds:
        servo->pos = constrain(servo->pos + updatePos, servo->minlim, servo->maxlim); 

        // Write the new servo pos to state struct:
        state.servopos[servo->idx] = servo->pos;
      
        // Update the servo pos (Only if not in failsafe):
        if (mode != 0) servo->curservo.writeMicroseconds(servo->pos); 

        // Store the update time: (Risk of overflow in??)
        servo->prevUpdate = millis();
    }
  }
}

void runServos(){
  // Drive allthe servos: 
  for (int i=0;i<NSERVOS;i++){runServo(&servos[i]);}
}


void runMotor(MotorCtrlStruct *motor){
  if (motor->speedUpdate) {
    motor->curmotor.setSpeed(motor->speed);
    motor->speedUpdate=false;
  }
}

void runMotors(){
  for (int i=0;i<2;i++){runMotor(&motors[i]);}

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

  // Init servos+motors:
  initServos();
  initMotors();

}



void loop(){
  
  // Get acceleration:
  while (IMU.accelerationAvailable()){
    IMU.readAcceleration(x, y, z);
    state.accel[0] = z;
    state.accel[1] = y;
    state.accel[2] = x;
    state.readtime = micros();
  }

  // Get gyro:
  while (IMU.gyroscopeAvailable()){
    IMU.readGyroscope(wx, wy, wz);
    state.w[0] = wz;
    state.w[1] = wy;
    state.w[2] = wx;
  }

  
 
  runServos();

  //TODO: Drive motors.
  runMotors();
  
  if ((millis() - lastInput) > COMMTIMEOUT){
    //Go to failsafe if too long since previous ctrl input.
    if (mode != 0) setMode(0);
  }
}

