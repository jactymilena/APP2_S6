// I2C Library
#include <Wire.h>

void setup() {
  // put your setup code here, to run once:
  Wire.begin();                // join i2c bus as master
  Serial.begin(115200);        // start serial for output
}

void loop() {

  Wire.beginTransmission(0x77);
  Wire.write(0x0D);
  Wire.endTransmission();

  Wire.requestFrom(0x77, 1);  // request 1 bytes from slave 

  // Serial
  while(Wire.available()) {   // slave may send less than requested
    int c = Wire.read();      // receive a byte as character
      
    Serial.println(c);        // print the character
  }

  delay(500);
}
