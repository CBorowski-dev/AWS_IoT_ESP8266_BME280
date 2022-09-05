#include <Arduino.h>
#include <ESP8266WiFi.h>
#include <PubSubClient.h>
#include <Adafruit_Sensor.h>
#include <Adafruit_BMP280.h>
#include <ArduinoJson.h>
#include "private_data.h"

// Configuration for the timeserver
const char* ntpServer1 = "pool.ntp.org";    // primary server 
const char* ntpServer2 = "time.nist.gov";   // secondary server
const long  gmtOffset_sec = 3600;           // GMT offset (seconds)
const int   daylightOffset_sec = 3600;      // daylight offset (seconds)
const u_int16_t bufferSize = 400;           // buffer size used by the PubSubClient
time_t gmtRawTime;

unsigned long time_between_messages = 10000;          // time between MQTT messages in msec

// AWS endpoint
const char* aws_endpoint = "akyvbbf6sysh7-ats.iot.eu-central-1.amazonaws.com";

// Client ID
const char* clientID = "weatherstation2";

// Topic we are going publish data to and to receive data from
const char* send_topic  = "data/weatherapp/weatherstation2/send";
const char* receive_topic  = "cmd/weatherapp/weatherstation2/receive";

// Create WiFiClientSecure instance
BearSSL::WiFiClientSecure secureClient;

// Set private key and certificates to use X.509 authentication
BearSSL::X509List clientCertList(certificatePemCrt);
BearSSL::PrivateKey clientPrivKey(privatePemKey);
BearSSL::X509List rootCert(caPemCrt);

// Adafruit_BME280 instance
// Adafruit_BME280 bme280;
Adafruit_BMP280 bmp280;

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
boolean send_messages = true;

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
  char json_data[length] = "";

  Serial.print("Message received on "); Serial.print(topic); Serial.print(": ");
  for (u_int32_t i = 0; i < length; i++) {
    // Serial.print((char)payload[i]);
    json_data[i] = (char)payload[i];
  }
  Serial.println(json_data);

  // Commands:
  // {"cmd":"set_time","value":123456}
  // {"cmd":"set_status","value":"on"} or {"cmd":"set_status","value":"off"}

  // create JSON document
  StaticJsonDocument<50> doc;
  // Deserialize the JSON document
  DeserializationError error = deserializeJson(doc, json_data);
  // Test if parsing succeeds.
  if (error) {
    Serial.print(F("deserializeJson() failed: "));
    Serial.println(error.f_str());
    return;
  }

  // Fetch values.
  // Most of the time, you can rely on the implicit casts.
  // In other case, you can do doc["time"].as<long>();
  const char* command = doc["cmd"];
  if (strcmp("set_time", command) == 0) {
    long time = doc["value"].as<long>();
    // Serial.print("--> time: "); Serial.println(time);
    // Set time between messages to received value
    time_between_messages = time;
  } else if (strcmp("set_status", command) == 0) {
    const char* status = doc["value"];
    // Serial.print("--> status: "); Serial.println(status);
    send_messages = strcmp("on", status) == 0;
  }
}

/**
 * @brief Set the Current Time object
 * 
 */
void readCurrentTime() {
  Serial.println("Waiting for NTP time sync...");
  gmtRawTime = time(nullptr);
  while (gmtRawTime < 8 * 3600 * 2) {
    delay(500);
    gmtRawTime = time(nullptr);
  }
  Serial.print("Current time: "); Serial.print(asctime(localtime(&gmtRawTime)));
}

/**
 * @brief 
 * Read sensor data
 */
void readSensorData() {
  temperature = bmp280.readTemperature();
  pressure = bmp280.readPressure() / 100.0F;
  //humidity = bmp280.readHumidity();
}

/**
 * @brief 
 * Prepare sensor data for MQTT transmission and publish
 */
void publishSensorData() {
  /** Payload in JSON format
  {
    "ClientID": "weatherstation2",
    "Time": "Wed Mar 30 20:26:48 2022",
    "GPS": {
      "latitude": 51.764000,
      "longitude": 8.777043
    },
    "BME280": {
      "Temperature": 21.7,
      "Humidity": 66.6,
      "Pressure": 988.6
    },
    "TempUnit": "°C",
    "HumidityUnit": "%",
    "PressureUnit": "hPa"
  }
  */

  // create JSON document
  StaticJsonDocument<bufferSize> doc;
  doc["ClientID"] = clientID;
  // include time
  char t_buffer[26];
  readCurrentTime();
  strftime(t_buffer, 25, "%a %b %d %T %G", localtime(&gmtRawTime));
  doc["Time"] = t_buffer;
  // include GPS coordinate, here hard coded
  JsonObject GPS = doc.createNestedObject("GPS");
  GPS["latitude"] = 51.764000;
  GPS["longitude"] = 8.777043;
  // include BME280 sensor data
  JsonObject BME280 = doc.createNestedObject("BME280");
  BME280["Temperature"] = temperature;
  BME280["Humidity"] = humidity;
  BME280["Pressure"] = pressure;
  // include sensor data units
  doc["TempUnit"] = "°C";
  doc["HumidityUnit"] = "%";
  doc["PressureUnit"] = "hPa";

  // convert JSON document to char array
  u_int16_t jsonSize = measureJson(doc);
  // Serial.println(jsonSize);
  char buffer[jsonSize + 1] = "";
  buffer[jsonSize + 1] = '\n';
  serializeJson(doc, buffer, jsonSize);

  // publish data
  if (pubSubClient.publish(send_topic, buffer)) {
    Serial.print("==> Message published: "); Serial.println(buffer);
  } else {
    Serial.print("==> Message couldn't be published: "); Serial.println(buffer);
  }
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
  Serial.println("AWS IoT Example");
  Serial.print("Connecting to "); Serial.println(WIFI_SSID);
  
  // Connect to WiFi
  WiFi.begin(WIFI_SSID, WIFI_PASS);
  while (WiFi.status() != WL_CONNECTED) {
    delay(500);
    Serial.print(".");
  }
  Serial.print(", WiFi connected, IP address: "); Serial.println(WiFi.localIP());

  // Initialize BMP280 sensor to address 0x76 
  int status = bmp280.begin(0x76);
  if (!status) {
      Serial.println("bmp280 sensor not available!");
      while (1);
  }

  // Increase buffer size from default value 256 (see PubSubClient.h) to 400
  pubSubClient.setBufferSize(bufferSize);

  // Get current time, otherwise certificates are flagged as expired
  // For some reason configuring time with the local GMT offset and the daylight offset leads to no income payload on AWS ioT side?!
  // configTime(gmtOffset_sec, daylightOffset_sec, ntpServer1, ntpServer2);
  configTime(0, 0, ntpServer1, ntpServer2);
  readCurrentTime();
  secureClient.setX509Time(gmtRawTime);

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
 
  // Send sonsor data every 10 seconds
  if ((millis() - lastPublish) > time_between_messages) {
    if (send_messages) {
      // Read sensor data
      readSensorData();
      // Send sensor data in JSON format to AWS IoT
      publishSensorData();
    }
    lastPublish = millis();
  }
}
