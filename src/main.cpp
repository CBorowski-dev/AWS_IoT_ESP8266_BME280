#include <Arduino.h>
#include <ESP8266WiFi.h>
#include <PubSubClient.h>
#include <Adafruit_Sensor.h>
#include <Adafruit_BME280.h>
#include <ArduinoJson.h>
#include "private_data.h"

// AWS endpoint
const char* aws_endpoint = "akyvbbf6sysh7-ats.iot.eu-central-1.amazonaws.com";

// Client ID
const char* clientID = "ESP32_BME280_Thing";

// Topic we are going publish data to and to receive data from
const char* send_topic  = "ESP32_BME280_Thing/dataSend";
const char* receive_topic  = "ESP32_BME280_Thing/dataReceive";

// Create WiFiClientSecure instance
BearSSL::WiFiClientSecure secureClient;

// Set private key and certificates to use X.509 authentication
BearSSL::X509List clientCertList(certificatePemCrt);
BearSSL::PrivateKey clientPrivKey(privatePemKey);
BearSSL::X509List rootCert(caPemCrt);

// Adafruit_BME280 instance
Adafruit_BME280 bme280;

// Variables for storing BME280 data
float temperature;
float pressure;
float humidity;

// Declaration of callback function for receiving MQTT messages
void msgReceived(char* topic, byte* payload, unsigned int len);

// Create PubSubClient instance
// Using X.509 certificate based mutual authentication --> use 8883
// Using MQTT over WebSocket --> use 443 or 8443
PubSubClient pubSubClient(aws_endpoint, 8883, msgReceived, secureClient); 

unsigned long lastPublish;
int msgCount;

/**
 * @brief 
 * Connects with the defined HTTPS REST endpoint in AWS IoT
 */
void pubSubCheckConnect() {
  if (!pubSubClient.connected()) {
    Serial.print("PubSubClient connecting to: "); Serial.print(aws_endpoint);
    while (!pubSubClient.connected()) {
      Serial.print(".");
      if (pubSubClient.connect(clientID)) {
        Serial.println(" connected");
        pubSubClient.subscribe(receive_topic);
      } else {
        Serial.print("failed, rc=");
        Serial.println(pubSubClient.state());

        char buf[256];
        secureClient.getLastSSLError(buf,256);
        Serial.print("WiFiClientSecure SSL error: ");
        Serial.println(buf);
      }
    }
  }
  pubSubClient.loop();
}

/**
 * @brief 
 * PubSubClient callback function for processing received message
 * 
 * @param topic 
 * @param payload 
 * @param length 
 */
void msgReceived(char* topic, byte* payload, u_int32_t length) {
  Serial.print("Message received on "); Serial.print(topic); Serial.print(": ");
  for (u_int32_t i = 0; i < length; i++) {
    Serial.print((char)payload[i]);
  }
  Serial.println();
}

/**
 * @brief Set the Current Time object
 * 
 */
void setCurrentTime() {
  configTime(3 * 3600, 0, "pool.ntp.org", "time.nist.gov");

  Serial.print("Waiting for NTP time sync: ");
  time_t now = time(nullptr);
  while (now < 8 * 3600 * 2) {
    delay(500);
    Serial.print(".");
    now = time(nullptr);
  }
  Serial.println("");
  struct tm timeinfo;
  gmtime_r(&now, &timeinfo);
  Serial.print("Current time: "); Serial.print(asctime(&timeinfo));

  secureClient.setX509Time(now);
}

/**
 * @brief 
 * Read sensor data
 */
void readSensorData() {
  temperature = bme280.readTemperature();
  pressure = bme280.readPressure() / 100.0F;
  humidity = bme280.readHumidity();
}

/**
 * @brief 
 * Prepare sensor data for MQTT transmission and publish
 */
void publishSensorData() {
  /**
  {
    "ClientID": "ESP32_BME280_Thing",
    "BME280": {
      "Temperature": 21.7,
      "Humidity": 66.6,
      "Pressure": 988.6
    },
    "TempUnit": "C"
    "HumidityUnit": "g/m3"
    "PressureUnit": "hPa",
  }
  */

  // create JSON document
  StaticJsonDocument<200> doc;

  doc["ClientID"] = clientID;
  JsonObject nestedObject = doc.createNestedObject("BME280");
  nestedObject["Temperature"] = temperature;
  nestedObject["Humidity"] = humidity;
  nestedObject["Pressure"] = pressure;
  doc["TempUnit"] = "°C";
  doc["HumidityUnit"] = "%";
  doc["PressureUnit"] = "hPa";

  // convert JSON document to char array
  u_int16_t jsonSize = measureJson(doc);
  char buffer[jsonSize + 1] = "";
  buffer[jsonSize + 1] = '\n';
  serializeJson(doc, buffer, jsonSize);

  // publish data
  pubSubClient.publish(send_topic, buffer);
  Serial.print("Published: "); Serial.println(buffer);
}

/**
 * @brief 
 * Arduino framework setup() function
 */
void setup() {
  // Set ESP8266 to run at 160 Mhz and not the default 80 Mhz
  os_update_cpu_frequency(160);

  // Configure serial connection
  Serial.begin(115200);
  Serial.println("ESP8266 AWS IoT Example");
  Serial.print("Connecting to "); Serial.println(WIFI_SSID);
  
  // Connect to WiFi
  WiFi.begin(WIFI_SSID, WIFI_PASS);
  while (WiFi.status() != WL_CONNECTED) {
    delay(500);
    Serial.print(".");
  }
  Serial.print(", WiFi connected, IP address: "); Serial.println(WiFi.localIP());

  // Initialize BME280 sensor to address 0x76 
  int status = bme280.begin(0x76);
  if (!status) {
      Serial.println("BME280 sensor not available!");
      while (1);
  }

  // Get current time, otherwise certificates are flagged as expired
  setCurrentTime();

  // Set certificates
  secureClient.setClientRSACert(&clientCertList, &clientPrivKey);
  secureClient.setTrustAnchors(&rootCert);
}

/**
 * @brief 
 * Arduino framework loop() function
 */
void loop() {
  pubSubCheckConnect();
 
  if (millis() - lastPublish > 10000) {
    // Read sensor data
    readSensorData();
    // Send sensor data in JSON format to AWS IoT
    publishSensorData();
    lastPublish = millis();
  }
}
