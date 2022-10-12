#include <Arduino.h>
#include <ESP8266WiFi.h>
#include <PubSubClient.h>
#include <Adafruit_Sensor.h>
#include <Adafruit_BME280.h>
#include <Adafruit_BMP280.h>
#include <ArduinoJson.h>
#include "private_data.h"
#include <list>
#include <stdio.h>

// Configuration for the timeserver
const char* ntpServer1 = "pool.ntp.org";    // primary server 
const char* ntpServer2 = "time.nist.gov";   // secondary server
const long  gmtOffset_sec = 3600;           // GMT offset (seconds)
const int   daylightOffset_sec = 3600;      // daylight offset (seconds)
const u_int16_t bufferSize = 2048;           // buffer size used by the PubSubClient
time_t gmtRawTime;

unsigned long time_between_messages = 20000;          // time between MQTT messages in msec

std::string const SHADOW_TOPIC_PREFIX = "$aws/things/" + std::string(CLIENTID) + "/shadow";

// Topic we are going publish data to and to receive data from
std::string send_topic  = "data/weatherapp/" + std::string(CLIENTID) + "/send";
std::string shadow_update_topic = SHADOW_TOPIC_PREFIX + "/update";
std::string shadow_get_topic = SHADOW_TOPIC_PREFIX + "/get";

std::list<std::string> subscribe_topics_list = {
  SHADOW_TOPIC_PREFIX + "/update/accepted",
  SHADOW_TOPIC_PREFIX + "/update/rejected",
  SHADOW_TOPIC_PREFIX + "/update/delta",
  // SHADOW_TOPIC_PREFIX + "/update/documents",
  SHADOW_TOPIC_PREFIX + "/get/accepted",
  SHADOW_TOPIC_PREFIX + "/get/rejected"
};

// Create WiFiClientSecure instance
BearSSL::WiFiClientSecure secureClient;

// Set private key and certificates to use X.509 authentication
BearSSL::X509List clientCertList(certificatePemCrt);
BearSSL::PrivateKey clientPrivKey(privatePemKey);
BearSSL::X509List rootCert(caPemCrt);

// Adafruit_BME280 instance
Adafruit_BME280 bme280;
// Adafruit_BMP280 bme280;

// Variables for storing BME280 data
float temperature;
float pressure;
float humidity;

// Declaration of callback function for receiving MQTT messages
void msgReceived(char* topic, byte* payload, unsigned int len);

// Create PubSubClient instance
// Using X.509 certificate based mutual authentication --> use 8883
// Using MQTT over WebSocket --> use 443 or 8443
PubSubClient pubSubClient(AWS_IOT_ENDPOINT, 8883, msgReceived, secureClient); 

unsigned long lastPublish;
boolean send_messages = true;

/**
 * @brief 
 * Connects with the defined HTTPS REST endpoint in AWS IoT
 */
void pubSubCheckConnect() {
  if (!pubSubClient.connected()) {
    Serial.print("PubSubClient connecting to: "); Serial.println(AWS_IOT_ENDPOINT);
    while (!pubSubClient.connected()) {
      Serial.print(".");
      if (pubSubClient.connect(CLIENTID)) {
        Serial.println("Is connected");
        for(std::string mqtt_topic: subscribe_topics_list){
          Serial.printf("Subscribing to MQTT topic: %s\n", mqtt_topic.c_str());
          pubSubClient.subscribe(mqtt_topic.c_str());
        }
        break;
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
  Serial.println("-----------------------------------------------");
  Serial.println("## MQTT Callback ##");
  char json_data[length];

  Serial.printf("Message arrived on topic: %s (%u) \n", topic, length);
  for (u_int32_t i = 0; i < length; i++) {
    // Serial.print((char)payload[i]);
    json_data[i] = (char)payload[i];
  }
  Serial.printf("Message is: %s\n", json_data);
  // create JSON document
  StaticJsonDocument<bufferSize> doc;
  // Deserialize the JSON document
  DeserializationError error = deserializeJson(doc, json_data);
  // Test if parsing succeeds.
  if (error) {
    Serial.print(F("deserializeJson() failed: "));
    Serial.println(error.f_str());
    return;
  }

  // Commands:
  // {"state": {"desired": {"time": 80000}}} or {"state": {"desired": {"status": "on"}}} or {"state": {"desired": {"time": 80000, "status": "on"}}}
  if (std::string(topic) == (SHADOW_TOPIC_PREFIX + "/update/delta")) {
    // create JSON document
    StaticJsonDocument<bufferSize> sndJsonDoc;
    JsonObject reportedState = sndJsonDoc.createNestedObject("state");
    JsonObject reported = reportedState.createNestedObject("reported");
    // Fetch values.
    // Most of the time, you can rely on the implicit casts.
    // In other case, you can do doc["time"].as<long>();
    JsonObject state = doc["state"];
    if (state.containsKey("time")) {
      long time = state["time"].as<long>();
      Serial.print("--> time: "); Serial.println(time);
      // Set time between messages to received value
      time_between_messages = time;
      reported["time"] = time_between_messages;
    } 
    if (state.containsKey("status")) {
      const char* status = state["status"];
      Serial.print("--> status: "); Serial.println(status);
      send_messages = strcmp("on", status) == 0;
      reported["status"] = status;
    }
    
    if(!reported.isNull()){
      // convert JSON document to char array
      u_int16_t jsonSize = measureJson(sndJsonDoc);
      char buffer[jsonSize + 1] = "";
      buffer[jsonSize + 1] = '\n';
      serializeJson(sndJsonDoc, buffer, jsonSize);
      if (pubSubClient.publish(shadow_update_topic.c_str(), buffer)) {
        Serial.print("==> Message published: "); Serial.println(buffer);
        // pubSubClient.publish(shadow_get_topic.c_str(), "");
      } else {
        Serial.print("==> Message couldn't be published: "); Serial.println(buffer);
      }
    }
  } else if (std::string(topic) == (SHADOW_TOPIC_PREFIX + "/get/accepted")){
    JsonObject state = doc["state"];
    JsonObject desiredState = state["desired"];
    if (desiredState.containsKey("time")) {
      long time = desiredState["time"].as<long>();
      // Set time between messages to received value
      time_between_messages = time;
    } 
    if (desiredState.containsKey("status")) {
      const char* status = desiredState["status"];
      send_messages = strcmp("on", status) == 0;
    }
  } else if (std::string(topic) == (SHADOW_TOPIC_PREFIX + "/get/rejected")){
    std::string message = doc["message"];
    if (message == "No shadow exists with name: \'" + std::string(CLIENTID) + "\'") {
      Serial.println("Creating Device Shadow for the device");
      send_messages = true;

      StaticJsonDocument<bufferSize> sndJsonDoc;
      JsonObject state = sndJsonDoc.createNestedObject("state");
      JsonObject reported = state.createNestedObject("reported");
      JsonObject desired = state.createNestedObject("desired");
      reported["welcome"] = "aws-iot";
      reported["time"] = time_between_messages;
      reported["status"] = "on";
      desired["welcome"] = "aws-iot";
      desired["time"] = time_between_messages;
      desired["status"] = "on";
      // convert JSON document to char array
      u_int16_t jsonSize = measureJson(sndJsonDoc);
      char buffer[jsonSize + 1] = "";
      buffer[jsonSize + 1] = '\n';
      serializeJson(sndJsonDoc, buffer, jsonSize);
      if (pubSubClient.publish(shadow_update_topic.c_str(), buffer)) {
        Serial.print("==> Message published: "); Serial.println(buffer);
      } else {
        Serial.print("==> Message couldn't be published: "); Serial.println(buffer);
      }
    } 
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
  temperature = bme280.readTemperature();
  pressure = bme280.readPressure() / 100.0F;
  humidity = bme280.readHumidity();
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
  Serial.println("-----------------------------------------------");
  Serial.println("## MQTT Publish ##");

  // create JSON document
  StaticJsonDocument<bufferSize> doc;
  doc["ClientID"] = CLIENTID;
  // include time
  char t_buffer[26];
  readCurrentTime();
  strftime(t_buffer, 25, "%a %b %d %T %G", localtime(&gmtRawTime));
  doc["Time"] = t_buffer;
  // include GPS coordinate, here hard coded
  JsonObject GPS = doc.createNestedObject("GPS");
  GPS["latitude"] = 51.498348;
  GPS["longitude"] = 7.005091;
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
  if (pubSubClient.publish(send_topic.c_str(), buffer)) {
    Serial.print("==> Message published: "); Serial.println(buffer);
  } else {
    Serial.print("==> Message couldn't be published: "); Serial.println(buffer);
  }
}

void get_values_from_device_shadow(){
  Serial.println("-----------------------------------------------");
  Serial.println("Retrieve values from Device Shadow");
  if (pubSubClient.publish(shadow_get_topic.c_str(), "")) {
      Serial.println("==> Message published");
  } else {
    Serial.println("==> Message couldn't be published");
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

  // Initialize BME280 sensor to address 0x76 
  int status = bme280.begin(0x76);
  if (!status) {
      Serial.println("BME280 sensor not available!");
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
  pubSubCheckConnect();
  delay(1000);
  get_values_from_device_shadow();
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
