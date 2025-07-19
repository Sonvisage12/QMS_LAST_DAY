#include <SPI.h>
#include <MFRC522.h>
#include <WiFi.h>
#include <PubSubClient.h>
#include <LittleFS.h>

#define RST_PIN         5
#define SS_PIN          4
#define GREEN_LED_PIN   15
#define RED_LED_PIN     2
#define BUZZER_PIN      27
#define WIFI_SSID       "YOUR_WIFI_SSID"
#define WIFI_PASSWORD   "YOUR_WIFI_PASSWORD"
#define MQTT_BROKER     "YOUR_BROKER_IP"
#define MQTT_PORT       1883
#define MQTT_TOPIC      "rfid/scan"

MFRC522 mfrc522(SS_PIN, RST_PIN);
WiFiClient espClient;
PubSubClient client(espClient);

void blinkLED(int pin) {
    digitalWrite(pin, HIGH);
    delay(500);
    digitalWrite(pin, LOW);
}

String getUIDString(byte *buffer, byte bufferSize) {
    String uid = "";
    for (byte i = 0; i < bufferSize; i++) {
        if (buffer[i] < 0x10) uid += "0";
        uid += String(buffer[i], HEX);
    }
    uid.toUpperCase();
    return uid;
}

void connectToWiFi() {
    WiFi.begin(WIFI_SSID, WIFI_PASSWORD);
    Serial.print("Connecting to WiFi...");
    while (WiFi.status() != WL_CONNECTED) {
        delay(500);
        Serial.print(".");
    }
    Serial.println(" Connected!");
}

void reconnectMQTT() {
    while (!client.connected()) {
        Serial.print("Connecting to MQTT...");
        if (client.connect("ESP32Client")) {
            Serial.println(" connected.");
            client.subscribe("rfid/response");
        } else {
            Serial.print(" failed, rc=");
            Serial.print(client.state());
            Serial.println(" try again in 5 seconds");
            delay(5000);
        }
    }
}

void callback(char* topic, byte* payload, unsigned int length) {
    String message;
    for (unsigned int i = 0; i < length; i++) {
        message += (char)payload[i];
    }

    Serial.print("Message received [");
    Serial.print(topic);
    Serial.print("]: ");
    Serial.println(message);

    if (message == "AUTHORIZED") {
        blinkLED(GREEN_LED_PIN);
    } else {
        blinkLED(RED_LED_PIN);
    }
}

void setup() {
    Serial.begin(115200);
    pinMode(GREEN_LED_PIN, OUTPUT);
    pinMode(RED_LED_PIN, OUTPUT);
    pinMode(BUZZER_PIN, OUTPUT);

    LittleFS.begin();
    SPI.begin();
    mfrc522.PCD_Init();

    connectToWiFi();
    client.setServer(MQTT_BROKER, MQTT_PORT);
    client.setCallback(callback);
}

void loop() {
    if (!client.connected()) {
        reconnectMQTT();
    }
    client.loop();

    if (mfrc522.PICC_IsNewCardPresent() && mfrc522.PICC_ReadCardSerial()) {
        String uid = getUIDString(mfrc522.uid.uidByte, mfrc522.uid.size);
        Serial.println("UID scanned: " + uid);
        client.publish(MQTT_TOPIC, uid.c_str());

        mfrc522.PICC_HaltA();
        mfrc522.PCD_StopCrypto1();
        delay(1000);
    }
}
