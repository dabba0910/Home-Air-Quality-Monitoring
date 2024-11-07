#include <WiFi.h>
#include <WebSocketsServer.h>
#include <Adafruit_Sensor.h>
#include <DHT.h>
#include <DHT_U.h>
#include <Wire.h>
#include "Adafruit_SGP30.h"
#include <PMS.h>

// Replace with your network credentials
const char* ssid = "Austin";
const char* password = "kissmefirst";

// DHT sensor setup
#define DHTPIN 4  // Pin connected to DHT22
#define DHTTYPE DHT22
DHT dht(DHTPIN, DHTTYPE);

// SGP30 sensor setup
Adafruit_SGP30 sgp;  // Create an instance of the SGP30 sensor

// PMSA003 sensor setup
#define PMS_RX_PIN 16  // RX2 pin for ESP32
#define PMS_TX_PIN 17  // TX2 pin for ESP32
HardwareSerial mySerial(2);  // Use Serial2 for PMSA003

// WebSocket server setup
WebSocketsServer webSocket = WebSocketsServer(81);

// Declare webSocketEvent function before setup()
void webSocketEvent(uint8_t num, WStype_t type, uint8_t * payload, size_t length);

void setup() {
  Serial.begin(9600);
  dht.begin();
  
  // Initialize SGP30 sensor
  if (!sgp.begin()) {
    Serial.println("SGP30 sensor not found!");
    while (1); // Stop if SGP30 is not found
  }
  Serial.println("SGP30 sensor found!");

  // Initialize PMSA003 sensor
  mySerial.begin(9600, SERIAL_8N1, PMS_RX_PIN, PMS_TX_PIN);
  Serial.println("PMSA003 sensor initialized");

  // Connect to Wi-Fi
  WiFi.begin(ssid, password);
  while (WiFi.status() != WL_CONNECTED) {
    delay(1000);
    Serial.println("Connecting to WiFi...");
  }
  Serial.println("Connected to WiFi");
  Serial.print("ESP32 IP Address: ");
  Serial.println(WiFi.localIP());

  // Start WebSocket server
  webSocket.begin();
  webSocket.onEvent(webSocketEvent);
}

void loop() {
  // Handle WebSocket communication
  webSocket.loop();
  
  // Read and send DHT sensor data
  readAndSendDHTData();
  
  // Read and send SGP30 sensor data
  readAndSendSGP30Data();
  
  // Read and send PMSA003 sensor data
  readAndSendPMSData();
  
  delay(2000);  // Send data every 2 seconds
}

void readAndSendDHTData() {
  float temperature = dht.readTemperature();
  float humidity = dht.readHumidity();
  if (!isnan(temperature) && !isnan(humidity)) {
    Serial.printf("Temperature: %.2f °C, Humidity: %.2f %%\n", temperature, humidity);
    String sensorData = "Temperature: " + String(temperature) + " °C, " +
                        "Humidity: " + String(humidity) + " %";
    webSocket.broadcastTXT(sensorData);
  } else {
    Serial.println("Failed to read from DHT sensor!");
  }
}

void readAndSendSGP30Data() {
  if (sgp.IAQmeasure()) {
    int eCO2 = sgp.eCO2;
    int TVOC = sgp.TVOC;
    Serial.printf("eCO2: %d ppm, TVOC: %d ppb\n", eCO2, TVOC);
    String sensorData = "eCO2: " + String(eCO2) + " ppm, " +
                        "TVOC: " + String(TVOC) + " ppb";
    webSocket.broadcastTXT(sensorData);
  } else {
    Serial.println("SGP30 measurement failed");
  }
}

void readAndSendPMSData() {
  static bool processed = false;  // Flag to ensure single processing
  
  if (!processed && mySerial.available() >= 32) {
    byte buffer[32];
    mySerial.readBytes(buffer, 32);
    
    if (buffer[0] == 0x42 && buffer[1] == 0x4D) {
      int pm1_0 = (buffer[10] << 8) | buffer[11];
      int pm2_5 = (buffer[12] << 8) | buffer[13];
      int pm10 = (buffer[14] << 8) | buffer[15];
      
      Serial.printf("PM1.0: %d µg/m³, PM2.5: %d µg/m³, PM10: %d µg/m³\n", pm1_0, pm2_5, pm10);
      String sensorData = "PM1.0: " + String(pm1_0) + " µg/m³, " +
                          "PM2.5: " + String(pm2_5) + " µg/m³, " +
                          "PM10: " + String(pm10) + " µg/m³";
      webSocket.broadcastTXT(sensorData);

      processed = true;  // Set flag to true after processing
    } else {
      Serial.println("Invalid data frame received. Clearing serial buffer.");
      while (mySerial.available()) {
        mySerial.read();
      }
    }
  }

  // Reset the flag after a delay to allow next processing
  static unsigned long lastTime = 0;
  unsigned long currentTime = millis();
  if (currentTime - lastTime >= 10000) {  // 10 seconds delay
    lastTime = currentTime;
    processed = false;
  }
}

void webSocketEvent(uint8_t num, WStype_t type, uint8_t * payload, size_t length) {
  if (type == WStype_CONNECTED) {
    Serial.println("WebSocket client connected");
  } else if (type == WStype_DISCONNECTED) {
    Serial.println("WebSocket client disconnected");
  } else if (type == WStype_TEXT) {
    Serial.printf("WebSocket received: %s\n", payload);
  }
}