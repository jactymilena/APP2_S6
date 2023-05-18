// I2C Library
#include <Wire.h>
#define DEVICE_ADDRESS 0x77



void read_coefficients(uint8_t *buffer) {
  
  uint16_t c0 = buffer[0] << 4 | ((buffer[1] >> 4) & 0x0F);
  uint16_t c1 = ((buffer[1] & 0x0F) << 8) | buffer[2];
  uint32_t c00 = buffer[3] << 12 | buffer[4] << 4 | ((buffer[5] >> 4) & 0x0F); 
  uint32_t c10 = ((buffer[5] & 0x0F) << 16) | buffer[6] << 8 | buffer[7];
  uint16_t c01 = buffer[8] << 8 | buffer[9];
  uint16_t c11 = buffer[10] << 8 | buffer[11];
  uint16_t c20 = buffer[12] << 8 | buffer[13];
  uint16_t c21 = buffer[14] << 8 | buffer[15];
  uint16_t c30 = buffer[16] << 8 | buffer[17];

  Serial.printf("c0 %u\n", c0);
  Serial.printf("c1 %u\n", c1);
  Serial.printf("c00 %u\n", c00);
  Serial.printf("c10 %u\n", c10);
  Serial.printf("c01 %u\n", c01);
  Serial.printf("c11 %u\n", c11);
  Serial.printf("c20 %u\n", c20);
  Serial.printf("c21 %u\n", c21);
  Serial.printf("c30 %u\n", c30);
}



void setup() {
  // put your setup code here, to run once:
  Wire.begin();                // join i2c bus as master
  Serial.begin(115200);        // start serial for output

  // Get calibration coefficients
  Wire.beginTransmission(DEVICE_ADDRESS);
  Wire.write(0x10);
  Wire.endTransmission();

  Wire.requestFrom(DEVICE_ADDRESS, 17);  // request 17 bytes from slave 

  int i = 0;
  uint8_t buffer[17];
  while(Wire.available()) {   
    buffer[i] = Wire.read();
    i++;
  }
  read_coefficients(buffer);

}

void loop() {

  // Wire.beginTransmission(0x77);
  // Wire.write(0x06);


  delay(500);
}
