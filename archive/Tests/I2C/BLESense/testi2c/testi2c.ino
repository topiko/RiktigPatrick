#include <Wire.h>
#include <Arduino_LSM9DS1.h>

#define I2C_SLAVE_ADDRESS 11 // 12 pour l'esclave 2 et ainsi de suite
//#define PAYLOAD_SIZE 2

float x, y, z;
int n;


struct StateStruct {
  float accel[3];       // 4*3 = 12
  float w[3];           // 12
  uint16_t servopos[2]; // 2*2 = 4
  byte paddind[4];      // 4
                        //------
                        // 32
};

StateStruct state;

void setup()
{
  Wire.begin(I2C_SLAVE_ADDRESS);
  Serial.begin(9600);
  Serial.println("-------------------------------------I am Slave1");
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
  Serial.println();
  Serial.println("Acceleration in G's");
  Serial.println("X\tY\tZ");
}

void loop(){
  state.servopos[0] = (uint16_t)1;
  if (IMU.accelerationAvailable()){
    IMU.readAcceleration(x, y, z);
    state.accel[0] = x;
    state.accel[1] = y;
    state.accel[2] = z;
  }
}

void requestEvents()
{
  //Serial.print(F("sending value : "));
  //char txData = 0;
  //int16_t ax = (int)(100*x);
  //Wire.write(ax);
  Wire.write((byte*) &state, sizeof(state));
  //for (int i=4;i<6;i++){
  //  Wire.write(i);
  //}
  Serial.println(F("---> recieved request"));
  //Serial.print("int ax ="); Serial.println(ax);
  //Serial.print("ax ="); Serial.println(x);
}

void receiveEvents(int numBytes)
{
  Serial.println(F("---> recieved events"));
  Serial.print(numBytes);
  Serial.println(F("bytes recieved"));
  Serial.print(F("recieved value : "));
  while (Wire.available()) { // slave may send less than requested
    char c = Wire.read(); // receive a byte as character
    Serial.print(c);         // print the character
  }
  Serial.println("");
}
