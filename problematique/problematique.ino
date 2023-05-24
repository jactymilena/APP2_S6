/*
* Meteo Station
* authors : Jacty Saenz, saej3101
*           Laurence Milette, mill3003
*/


// I2C Library
#include <Wire.h>
#include <vector>
#include "BLEDevice.h"


#define DEVICE_ADDRESS 0x77
#define NUMBER_COEFFICIENTS 18
#define KT 7864320.0f
#define LIGHT_PIN 34
#define RAIN_PIN 23
#define WIND_DIRECTION_PIN 35
#define WIND_SPEED_PIN 27
#define RAIN_CST 0.2794f
#define WIND_SPEED_REF 2.4

int                  nb_contact = 0;
std::vector<int32_t> coefficients;


static BLEUUID       serviceUUID("6E400001-B5A3-F393-E0A9-E50E24DCCA9E");
static BLEUUID       charUUID("6E400002-B5A3-F393-E0A9-E50E24DCCA9E");

static boolean doConnect = false;
static boolean connected = false;
static boolean doScan = false;
static BLERemoteCharacteristic* pRemoteCharacteristic;
static BLEAdvertisedDevice* myDevice;

struct HumiditySensorResponse {
  float temperature;
  float humidity;

  HumiditySensorResponse(const HumiditySensorResponse& res) : temperature(res.temperature), humidity(res.humidity) {}
  HumiditySensorResponse(float temp, float hum) : temperature(temp), humidity(hum) {}
};

// bool operator==(const HumiditySensorResponse& r1, const HumiditySensorResponse& r2)
// {
//   return true;
//   // return (r1.humidity == r2.humidity) && (r1.temperature == r2.temperature);
// }

struct I2cResponse {
  float temperature;
  float pressure;

  I2cResponse(const I2cResponse& res) : temperature(res.temperature), pressure(res.pressure) {}
  I2cResponse(float temp, float press) : temperature(temp), pressure(press) {}
};

bool operator==(const I2cResponse& r1, const I2cResponse& r2)
{
  return (r1.pressure == r2.pressure) && (r1.temperature == r2.temperature);
}

struct MeteoStationValues {
  I2cResponse i2cRes;
  int light;
  float rain;
  std::string windDirection;
  float windSpeed;
  HumiditySensorResponse humidityRes;

  MeteoStationValues(I2cResponse resI2C, int l, float r, std::string wDir, float wSpeed, HumiditySensorResponse humRes) : i2cRes(resI2C), light(l), rain(r), windDirection(wDir), windSpeed(wSpeed), humidityRes(humRes)  {}

  MeteoStationValues& operator =(const MeteoStationValues& m)
  {
      i2cRes.pressure = m.i2cRes.pressure;
      i2cRes.temperature = m.i2cRes.temperature;
      light = m.light;
      rain = m.rain;
      windDirection = m.windDirection;
      windSpeed = m.windSpeed;
      humidityRes.humidity = m.humidityRes.humidity;
      humidityRes.temperature = m.humidityRes.temperature;

      return *this;
  }
};

bool operator==(const MeteoStationValues& m1, const MeteoStationValues& m2)
{
    return (m1.i2cRes == m2.i2cRes) &&
           (m1.light == m2.light) &&
           (m1.rain == m2.rain) &&
           (m1.windDirection == m2.windDirection) &&
           (m1.windSpeed == m2.windSpeed) &&
           (m1.humidityRes.temperature == m2.humidityRes.temperature) &&
           (m1.humidityRes.humidity == m2.humidityRes.humidity);
}

MeteoStationValues lastStationValues(I2cResponse(0, 0), 0, 0, "NORD", 0, HumiditySensorResponse(0, 0));

static void notifyCallback(
  BLERemoteCharacteristic* pBLERemoteCharacteristic,
  uint8_t* pData,
  size_t length,
  bool isNotify) {
    Serial.print("Notify callback for characteristic ");
    Serial.print(pBLERemoteCharacteristic->getUUID().toString().c_str());
    Serial.print(" of data length ");
    Serial.println(length);
    Serial.print("data: ");
    Serial.write(pData, length);
    Serial.println();
}

class MyClientCallback : public BLEClientCallbacks {
  void onConnect(BLEClient* pclient) {
  }

  void onDisconnect(BLEClient* pclient) {
    connected = false;
    Serial.println("onDisconnect");
  }
};

bool connectToServer() {
    Serial.print("Forming a connection to ");
    Serial.println(myDevice->getAddress().toString().c_str());
    
    BLEClient*  pClient  = BLEDevice::createClient();
    Serial.println(" - Created client");

    pClient->setClientCallbacks(new MyClientCallback());

    // Connect to the remove BLE Server.
    pClient->connect(myDevice);  // if you pass BLEAdvertisedDevice instead of address, it will be recognized type of peer device address (public or private)
    Serial.println(" - Connected to server");
    pClient->setMTU(517); //set client to request maximum MTU from server (default is 23 otherwise)
  
    // Obtain a reference to the service we are after in the remote BLE server.
    BLERemoteService* pRemoteService = pClient->getService(serviceUUID);
    if (pRemoteService == nullptr) {
      Serial.print("Failed to find our service UUID: ");
      Serial.println(serviceUUID.toString().c_str());
      pClient->disconnect();
      return false;
    }
    Serial.println(" - Found our service");


    // Obtain a reference to the characteristic in the service of the remote BLE server.
    pRemoteCharacteristic = pRemoteService->getCharacteristic(charUUID);
    if (pRemoteCharacteristic == nullptr) {
      Serial.print("Failed to find our characteristic UUID: ");
      Serial.println(charUUID.toString().c_str());
      pClient->disconnect();
      return false;
    }
    Serial.println(" - Found our characteristic");

    // Read the value of the characteristic.
    if(pRemoteCharacteristic->canRead()) {
      std::string value = pRemoteCharacteristic->readValue();
      Serial.print("The characteristic value was: ");
      Serial.println(value.c_str());
    }

    if(pRemoteCharacteristic->canNotify())
      pRemoteCharacteristic->registerForNotify(notifyCallback);

    connected = true;
    return true;
}

/**
 * Scan for BLE servers and find the first one that advertises the service we are looking for.
 */
class MyAdvertisedDeviceCallbacks: public BLEAdvertisedDeviceCallbacks {
 /**
   * Called for each advertising BLE server.
   */
  void onResult(BLEAdvertisedDevice advertisedDevice) {
    Serial.print("BLE Advertised Device found: ");
    Serial.println(advertisedDevice.toString().c_str());

    // We have found a device, let us now see if it contains the service we are looking for.

    if (advertisedDevice.haveServiceUUID() && advertisedDevice.isAdvertisingService(serviceUUID)) {
      Serial.print("Found our server ");

      BLEDevice::getScan()->stop();
      myDevice = new BLEAdvertisedDevice(advertisedDevice);
      doConnect = true;
      doScan = true;


    } // Found our server
  } // onResult
}; // MyAdvertisedDeviceCallbacks


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

  return coefficients[2] + p_raw_sc * (coefficients[3] + p_raw_sc * (coefficients[6] + p_raw_sc * coefficients[8])) + t_raw_sc * coefficients[4] + t_raw_sc * p_raw_sc * (coefficients[5] + p_raw_sc * coefficients[7]);
}

I2cResponse i2cSensor() {
  Wire.beginTransmission(DEVICE_ADDRESS);
  Wire.write(0x08);
  Wire.write(0x02);
  Wire.endTransmission();

  float t_raw_sc = i2cTemperature();
  float t_comp = coefficients[0] * 0.5f + (float)coefficients[1] * t_raw_sc;

  Wire.beginTransmission(DEVICE_ADDRESS);
  Wire.write(0x08);
  Wire.write(0x01);
  Wire.endTransmission();

  float p_comp = i2cPressure(t_raw_sc);

  return I2cResponse(t_comp, p_comp);
}

int lightSensor() {
  return analogRead(LIGHT_PIN);  
}

void setUpRainSensor() {
  pinMode(RAIN_PIN, INPUT);
  attachInterrupt(digitalPinToInterrupt(RAIN_PIN), incrementNbContact, RISING);
}

void incrementNbContact() {
  nb_contact++;
}

float rainGauge() {
  return nb_contact * RAIN_CST;
}

std::string windDirectionSensor() {
  int val = analogRead(WIND_DIRECTION_PIN);
  float voltage = (val * 3.3) / 4096;

  if (voltage >= 2.09 && voltage <= 2.39)       return "NORD";
  else if (voltage >= 1.15 && voltage <= 1.34)  return "NORD-EST";
  else if (voltage >= 0.08 && voltage <= 0.17)  return "EST";
  else if (voltage >= 0.44 && voltage <= 0.47)  return "SUD-EST";
  else if (voltage >= 0.27 && voltage <= 0.78)  return "SUD";
  else if (voltage >= 1.76 && voltage <= 1.87)  return "SUD-OUEST";
  else if (voltage >= 3.09 && voltage <= 3.12)  return "OUEST";
  else if (voltage >= 2.52 && voltage <= 2.81)  return "NORD-OUEST";
  else                                          return "OTHER";
}

HumiditySensorResponse humiditySensor() {
  int i, j;
  int duree[42];
  unsigned long pulse;
  byte data[5];
  float humidite;
  float temperature;
  int broche = 16;

  delay(200);
  
  pinMode(broche, OUTPUT_OPEN_DRAIN);
  digitalWrite(broche, HIGH);
  delay(250);
  digitalWrite(broche, LOW);
  delay(20);
  digitalWrite(broche, HIGH);
  delayMicroseconds(40);
  pinMode(broche, INPUT_PULLUP);
  
  while (digitalRead(broche) == HIGH);
  i = 0;

  do {
        pulse = pulseIn(broche, HIGH);
        duree[i] = pulse;
        i++;
  } while (pulse != 0);
 
  if (i != 42) 
    Serial.printf(" Erreur timing \n"); 

  for (i=0; i<5; i++) {
    data[i] = 0;
    for (j = ((8*i)+1); j < ((8*i)+9); j++) {
      data[i] = data[i] * 2;
      if (duree[j] > 50) {
        data[i] = data[i] + 1;
      }
    }
  }

  if ( (data[0] + data[1] + data[2] + data[3]) != data[4] ) 
    Serial.println(" Erreur checksum (HUMIDITY SENSOR)");

  humidite = data[0] + (data[1] / 256.0);
  temperature = data [2] + (data[3] / 256.0);

  return HumiditySensorResponse(temperature, humidite);
}

float windSpeedSensor() {
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
  return WIND_SPEED_REF * cpt;
}


void setupBLE() {
  Serial.println("Starting Arduino BLE Client application...");
  BLEDevice::init("");

  BLEScan* pBLEScan = BLEDevice::getScan();
  pBLEScan->setAdvertisedDeviceCallbacks(new MyAdvertisedDeviceCallbacks());
  pBLEScan->setInterval(1349);
  pBLEScan->setWindow(449);
  pBLEScan->setActiveScan(true);
  pBLEScan->start(5, false);
}

void checkServerConnection() {
  if (doConnect == true) {
    if (connectToServer()) {
      Serial.println("We are now connected to the BLE Server.");
    } else {
      Serial.println("We have failed to connect to the server; there is nothin more we will do.");
    }
    doConnect = false;
  }
}

void setup() {
  // put your setup code here, to run once:
  Wire.begin();          // join i2c bus as master
  Serial.begin(115200);  // start serial for output

  setupBLE();
  setupI2c();
  setUpRainSensor();
}

void loop() {
  checkServerConnection();

  if (connected) {
    I2cResponse i2cRes = i2cSensor();
    int light = lightSensor();
    float rain = rainGauge();
    std::string windDirection = windDirectionSensor();
    float windSpeed = windSpeedSensor();
    HumiditySensorResponse humRes = humiditySensor();

    MeteoStationValues currStationValues(i2cRes, light, rain, windDirection, windSpeed, humRes);

    if(!(lastStationValues == currStationValues)) {
      std::string sendVal = "\nEnsoleillement : " + std::to_string(light) + 
                            "\nPluie : " + std::to_string(rain) + " mm" + 
                            "\nDirection du vent : " + windDirection.c_str() + 
                            "\nVitesse du vent : " + std::to_string(windSpeed) + " km/h" +
                            "\nPressure : " + std::to_string(i2cRes.pressure) + " Pa" + 
                            "\nTemperature : " + std::to_string(i2cRes.temperature) + " C" +
                            "\nHumidity : " + std::to_string(humRes.humidity) + " %RH" + 
                            "\nTemperature : " + std::to_string(humRes.temperature) + " C\n";
    
      pRemoteCharacteristic->writeValue(sendVal.c_str(), sendVal.length());
      lastStationValues = currStationValues;
    }
  }

  delay(500);
}
