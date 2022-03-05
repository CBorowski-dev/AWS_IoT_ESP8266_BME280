#include <Arduino.h>
#include <ESP8266WiFi.h>
#include <PubSubClient.h>
#include "private_data.h"

// AWS endpoint
const char* aws_endpoint = "akyvbbf6sysh7-ats.iot.eu-central-1.amazonaws.com";

// Topic we are going publish data to
const char* aws_topic  = "$aws/things/ESP32_BME280_Thing/shadow/update";

// Set private key and certificates to use X.509 authentication
BearSSL::X509List client_crt(certificatePemCrt);
BearSSL::PrivateKey client_key(privatePemKey);
BearSSL::X509List rootCert(caPemCrt);

// Declaration of callback function for receiving MQTT messages
void msgReceived(char* topic, byte* payload, unsigned int len);

// Create WiFiClientSecure instance
WiFiClientSecure wiFiClient;

// Create PubSubClient instance
// Using X.509 certificate based mutual authentication --> use 8883
// Using MQTT over WebSocket --> use 443 or 8443
PubSubClient pubSubClient(aws_endpoint, 8883, msgReceived, wiFiClient); 

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
      if (pubSubClient.connect("1234")) {
        Serial.println(" connected");
        pubSubClient.subscribe("inTopic");
      } else {
        Serial.print("failed, rc=");
        Serial.println(pubSubClient.state());

        char buf[256];
        wiFiClient.getLastSSLError(buf,256);
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
}

/**
 * @brief 
 * Arduino framework setup() function
 */
void setup() {
  // Set ESP8266 to run at 160 Mhz and not the default 80 Mhz
  os_update_cpu_frequency(160);

  Serial.begin(115200);
  Serial.println("ESP8266 AWS IoT Example");

  Serial.print("Connecting to ");
  Serial.println(WIFI_SSID);
  
  WiFi.begin(WIFI_SSID, WIFI_PASS);
  while (WiFi.status() != WL_CONNECTED) {
    delay(500);
    Serial.print(".");
  }
  Serial.print(", WiFi connected, IP address: "); Serial.println(WiFi.localIP());

  // get current time, otherwise certificates are flagged as expired
  setCurrentTime();

  wiFiClient.setClientRSACert(&client_crt, &client_key);
  wiFiClient.setTrustAnchors(&rootCert);
}

/**
 * @brief 
 * Arduino framework loop() function
 */
void loop() {
  pubSubCheckConnect();

  if (millis() - lastPublish > 10000) {
    // message which will be send to AWS IoT via MQTT
    String msg = String("Hello from ESP8266: ") + ++msgCount;
    pubSubClient.publish("outTopic", msg.c_str());
    Serial.print("Published: "); Serial.println(msg);
    lastPublish = millis();
  }
}
