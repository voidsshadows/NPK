// Include necessary libraries
#include <WiFi.h>
#include <WiFiClientSecure.h>
#include <PubSubClient.h>
#include <ModbusMaster.h>

// WiFi Credentials
const char* ssid = "Iphones";
const char* password = "12345679";

// MQTT Broker Details (HiveMQ)
const char* mqtt_server = "f9d2e52fdd6f43059ce2cd8d7f46a29d.s1.eu.hivemq.cloud";
const int mqtt_port = 8883;  // Secure MQTT Port
const char* mqtt_username = "soiler";
const char* mqtt_password = "Soil@123";

// RS485 Modbus Settings
#define RE_DE  2  // GPIO2 for RS485 DE/RE control
ModbusMaster modbus;

// WiFi & MQTT Clients
WiFiClientSecure espClient;
PubSubClient client(espClient);

// Connect to WiFi
void setupWiFi() {
    Serial.print("Connecting to WiFi");
    WiFi.begin(ssid, password);
    while (WiFi.status() != WL_CONNECTED) {
        delay(500);
        Serial.print(".");
    }
    Serial.println("\nWiFi Connected");
    Serial.print("IP Address: ");
    Serial.println(WiFi.localIP());
}

// Connect to MQTT
void reconnectMQTT() {
    while (!client.connected()) {
        Serial.print("Connecting to MQTT...");
        if (client.connect("ESP32_Client", mqtt_username, mqtt_password)) {
            Serial.println("Connected to MQTT Broker");
        } else {
            Serial.print("Failed, rc=");
            Serial.print(client.state());
            Serial.println(" Retrying in 5 seconds...");
            delay(5000);
        }
    }
}

// Read NPK Sensor Data
void readNPKSensor() {
    Serial.println("\nRequesting NPK sensor data...");
    modbus.begin(1, Serial2);  // Sensor ID 1, using Serial2
    digitalWrite(RE_DE, HIGH);  // Enable RS485
    
    uint8_t result = modbus.readHoldingRegisters(0x03, 3); // N, P, K values
    if (result == modbus.ku8MBSuccess) {
        int nitrogen = modbus.getResponseBuffer(0);
        int phosphorus = modbus.getResponseBuffer(1);
        int potassium = modbus.getResponseBuffer(2);

        Serial.printf("N: %d mg/kg, P: %d mg/kg, K: %d mg/kg\n", nitrogen, phosphorus, potassium);

        // Publish Data to MQTT
        String payload = "{ \"Nitrogen\": " + String(nitrogen) + 
                         ", \"Phosphorus\": " + String(phosphorus) +
                         ", \"Potassium\": " + String(potassium) + " }";
        if (client.publish("soil/data", payload.c_str())) {
            Serial.println("Published to MQTT successfully.");
        } else {
            Serial.println("Failed to publish to MQTT.");
        }
    } else {
        Serial.println("Failed to read from NPK sensor.");
        
        // Publish Error to MQTT if sensor read fails
        String errorPayload = "{ \"error\": \"Failed to read from NPK sensor\" }";
        if (client.publish("soil/error", errorPayload.c_str())) {
            Serial.println("Published error to MQTT.");
        } else {
            Serial.println("Failed to publish error to MQTT.");
        }
    }
    digitalWrite(RE_DE, LOW);  // Disable RS485
}

void setup() {
    Serial.begin(115200);
    Serial2.begin(9600, SERIAL_8N1, 16, 17); // RS485 on GPIO16 (RX) & GPIO17 (TX)
    pinMode(RE_DE, OUTPUT);
    
    setupWiFi();
    client.setServer(mqtt_server, mqtt_port);
    espClient.setInsecure();  // Required for HiveMQ TLS connections
}

void loop() {
    if (!client.connected()) reconnectMQTT();
    client.loop();

    readNPKSensor();
    delay(5000); // Read every 5 seconds
}
