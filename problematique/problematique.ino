// I2C Library
#include <Wire.h>
#include <vector>
#define DEVICE_ADDRESS 0x77
#define NUMBER_COEFFICIENTS 18
#define KT 7864320.0f
#define LIGHT_PIN 34
#define RAIN_PIN 23
#define WIND_DIRECTION_PIN 35
#define WIND_SPEED_PIN 27
#define RAIN_CST 0.2794f
#define WIND_SPEED_REF 2.4
int nb_contact = 0;

std::vector<int32_t> coefficients;


void readCoefficients(uint8_t *buffer) {
  // ajout d'un check pour les chiffres negatifs (C1 devrait toujours etre negatif)

  coefficients.push_back(checkNegative(buffer[0] << 4 | ((buffer[1] >> 4) & 0x0F), 12));                    // c0
  coefficients.push_back(checkNegative((((buffer[1] & 0x0F) << 8) | buffer[2]), 12));                       // c1
  coefficients.push_back(checkNegative(buffer[3] << 12 | buffer[4] << 4 | ((buffer[5] >> 4) & 0x0F), 20));  // c00
  coefficients.push_back(checkNegative(((buffer[5] & 0x0F) << 16) | buffer[6] << 8 | buffer[7], 20));       // c10

  for (int i = 8; i < NUMBER_COEFFICIENTS - 1; i += 2)  // c01, c11, c20, c21, c30
    coefficients.push_back(checkNegative(buffer[i] << 8 | buffer[i + 1], 16));

  // for(int i = 0; i < coefficients.size(); i++)
  //   Serial.printf("coefficients %d i %d\n", coefficients[i], i);
}


int32_t checkNegative(int32_t c, int nb_bits) {
  if (c > pow(2, nb_bits - 1) - 1)
    return c - pow(2, nb_bits);
  return c;
}

void setupI2c() {
  // Get calibration coefficients
  Wire.beginTransmission(DEVICE_ADDRESS);
  Wire.write(0x10);
  Wire.endTransmission();

  Wire.requestFrom(DEVICE_ADDRESS, NUMBER_COEFFICIENTS);  // request 17 bytes from slave

  int i = 0;
  uint8_t buffer[NUMBER_COEFFICIENTS];
  while (Wire.available()) {
    buffer[i] = Wire.read();
    i++;
  }
  readCoefficients(buffer);

  // PRS_CFG
  Wire.beginTransmission(DEVICE_ADDRESS);
  Wire.write(0x06);
  Wire.write(0x03 << 4 | 0x03);
  // Serial.printf("PRS_CFG %u\n", 0x03 << 4 |  0x03);
  Wire.endTransmission();

  // TMP_CFG
  Wire.beginTransmission(DEVICE_ADDRESS);
  Wire.write(0x07);
  Wire.write(0x01 << 7 | 0x03 << 4 | 0x03);
  Wire.endTransmission();

  // CFG_REG
  // Wire.beginTransmission(DEVICE_ADDRESS);
  // Wire.write(0x09);
  // Wire.write( 0x00 << 7 | 0x00 << 6  | 0x00 << 5 | 0x00 << 4 | 0x00 << 3 | 0x00 << 2 | 0x00 << 1 | 0x00);
  // Wire.endTransmission();
}

float i2cTemperature() {
  // Get temperature values
  Wire.beginTransmission(DEVICE_ADDRESS);
  Wire.write(0x03);
  Wire.endTransmission();

  Wire.requestFrom(DEVICE_ADDRESS, 3);  // request 3 bytes from slave

  int i = 0;
  int8_t tmp[3];
  while (Wire.available()) {
    tmp[i] = Wire.read();
    i++;
  }

  int t_raw = checkNegative(tmp[0] << 16 | tmp[1] << 8 | tmp[2], 24);
  // Calculate real temperature
  float t_raw_sc = (float)t_raw / KT;

  // Serial.printf("t_raw %f\n", t_raw);
  // Serial.printf("t_raw_sc %f\n", t_raw_sc);

  return t_raw_sc;
}

float i2cPressure(float t_raw_sc) {

  // Get pressure values
  Wire.beginTransmission(DEVICE_ADDRESS);
  Wire.write(0x00);
  Wire.endTransmission();

  Wire.requestFrom(DEVICE_ADDRESS, 3);  // request 3 bytes from slave

  int i = 0;
  int8_t tmp[3];
  while (Wire.available()) {
    tmp[i] = Wire.read();
    i++;
  }

  int p_raw = checkNegative(tmp[0] << 16 | tmp[1] << 8 | tmp[2], 24);
  // Calculate real pressure
  int p_raw_sc = p_raw / KT;

  // Serial.printf("p_raw %f\n", p_raw);
  // Serial.printf("p_raw_sc %f\n", p_raw_sc);

  return coefficients[2] + p_raw_sc * (coefficients[3] + p_raw_sc * (coefficients[6] + p_raw_sc * coefficients[8])) + t_raw_sc * coefficients[4] + t_raw_sc * p_raw_sc * (coefficients[5] + p_raw_sc * coefficients[7]);
}

void i2cSensor() {
  Wire.beginTransmission(DEVICE_ADDRESS);
  Wire.write(0x08);
  Wire.write(0x02);
  Wire.endTransmission();

  float t_raw_sc = i2cTemperature();
  float t_comp = coefficients[0] * 0.5f + (float)coefficients[1] * t_raw_sc;

  // Serial.printf("Temperature %f C\n", t_comp);

  Wire.beginTransmission(DEVICE_ADDRESS);
  Wire.write(0x08);
  Wire.write(0x01);
  Wire.endTransmission();

  float p_comp = i2cPressure(t_raw_sc);

  // Serial.printf("Pressure %f Pa\n", p_comp);
}

void lightSensor() {
  int light = analogRead(LIGHT_PIN);
  // Serial.printf("Ensoleillement %d\n", light);
}

void setUpRainSensor() {
  pinMode(RAIN_PIN, INPUT);
  attachInterrupt(digitalPinToInterrupt(RAIN_PIN), incrementNbContact, RISING);
}

void incrementNbContact() {
  nb_contact++;
  // Serial.printf("incre %d\n", nb_contact);
}

float rainGauge() {
  float rain = nb_contact * RAIN_CST;
  // Serial.printf("rain %f\n", rain);
  return rain;
}

void windDirectionSensor() {
  int val = analogRead(WIND_DIRECTION_PIN);
  float voltage = (val * 3.3) / 4096;
  // Serial.printf("voltage %f\n", voltage);
  if (voltage >= 2.09 && voltage <= 2.39) {
    Serial.println("NORD");
  } else if (voltage >= 1.15 && voltage <= 1.34) {
    Serial.println("NORD-EST");
  } else if (voltage >= 0.08 && voltage <= 0.17) {
    Serial.println("EST");
  } else if (voltage >= 0.44 && voltage <= 0.47) {
    Serial.println("SUD-EST");
  } else if (voltage >= 0.27 && voltage <= 0.78) {
    Serial.println("SUD");
  } else if (voltage >= 1.76 && voltage <= 1.87) {
    Serial.println("SUD-OUEST");
  } else if (voltage >= 3.09 && voltage <= 3.12) {
    Serial.println("OUEST");
  } else if (voltage >= 2.52 && voltage <= 2.81) {
    Serial.println("NORD-OUEST");
  } else
    Serial.println("AUTRES THE FUCK");
}

void windSpeedSensor() {
  int last_val = analogRead(WIND_SPEED_PIN);
  int cpt = 0;
  float init_time = millis();
  int curr_val = 0;
  while (millis() < (init_time + 1000)) {
    curr_val = analogRead(WIND_SPEED_PIN);
    if (curr_val != last_val) {
      cpt++;
      last_val = curr_val;
    }
  }
  float speed = WIND_SPEED_REF * cpt;
  Serial.printf("Vitesse du vent %f\n", speed);
}

void setup() {
  // put your setup code here, to run once:
  Wire.begin();          // join i2c bus as master
  Serial.begin(115200);  // start serial for output

  setupI2c();
  // Pluie
  setUpRainSensor();
}

void loop() {
  // Temperature et pression
  i2cSensor();

  // Ensoleillement
  lightSensor();

  // Pluie
  rainGauge();

  // Direction du vent
  // windDirectionSensor();

  // Vitesse du vent
  windSpeedSensor();

  delay(500);
}
