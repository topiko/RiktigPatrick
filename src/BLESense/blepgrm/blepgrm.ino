#include <Wire.h>
#include <Arduino_LSM9DS1.h>
#include <Servo.h>

#define REFRESH_INTERVAL 3003 // KST servos operate at 333Hz --> 1/333 * 1e6

#define I2C_SLAVE_ADDRESS 11 
#define NSERVOS 2
#define SERVOUPDATEPERIOD 20 // in ms

float x, y, z;
int n;
int servopins[2] = {11, 12};

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
  byte paddind[4];            // 4
                              //------
                              // 32
};

StateStruct state;
ServoCtrlStruct servos[NSERVOS];

void setup()
{
  Wire.begin(I2C_SLAVE_ADDRESS);
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
  for (int i=0;i<NSERVOS;i++) initServos(i, &servos[i]);
}

void initServos(int idx, ServoCtrlStruct *servo){
  // Initialize the various servo paramaters to some values:
  servo->prevUpdate = 0;
  servo->speed = 0;
  servo->minlim = 1400;
  servo->maxlim = 1600;
  servo->idx = idx;
  servo->pos = 1500;
  servo->maxSpeed = (servo->maxlim - servo->minlim)/1000 * 1/128; 
  servo->curservo.attach(servopins[idx]);
  
  // Disable servo:
  servo->curservo.writeMicroseconds(0);
}

void runServo(ServoCtrlStruct *servo){
  
  
  int sinceUpdate = millis() - servo->prevUpdate; 

  if (SERVOUPDATEPERIOD < sinceUpdate){
    
    // Compute position based on control integer:
    float speedScale = float(servo->speed / 128);
    float speed = servo->maxSpeed * speedScale; 
    int16_t updatePos = sinceUpdate * speed;
    
    // Compute the new pos in terms of microseconds:
    servo->pos = constrain(servo->pos + updatePos, servo->minlim, servo->maxlim); 

    // Write the new servo pos to state struct:
    state.servopos[servo->idx] = servo->pos;
    
    // Update the servo pos:
    servo->curservo.writeMicroseconds(servo->pos); 
    
    // Store the update time: (Risk of overflow in??)
    servo->prevUpdate = millis();
  }
}

void loop(){
  
  // Get acceleration:
  if (IMU.accelerationAvailable()){
    IMU.readAcceleration(x, y, z);
    state.accel[0] = x;
    state.accel[1] = y;
    state.accel[2] = z;
    Serial.println("Main loop acc:");
    Serial.println(state.accel[0]);
    Serial.println(state.accel[1]);
    Serial.println(state.accel[2]);
    
  }
 
  // Drive the servos: 
  // TODO: IMU not avail while driveServos is called!?!?
  for (int i=0;i<NSERVOS;1){runServo(&servos[i]);}
}

void requestEvents(){
  // Send state array to i2c:
  Serial.println("Event requested!");
  Serial.println(state.accel[0]);
  Serial.println(state.accel[1]);
  Serial.println(state.accel[2]);
  Wire.write((byte*) &state, sizeof(state));
}

void receiveEvents(int numBytes){

  byte buffer[5];
  Wire.readBytes(buffer, 5);

  Serial.println("Read 5 bytes?");
  
  for (int i=0;i<5;i++) Serial.println(buffer[i]);
  /*
  while (Wire.available()) { // slave may send less than requested
    char c = Wire.read(); // receive a byte as character
    Serial.print(c);         // print the character
  }*/
  Serial.println("");
}
