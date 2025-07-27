#include <ESP8266WiFi.h>
#include <PubSubClient.h>
#include <DHT.h>

#include "config.h"

//Wifi credentials
const char* ssid = WIFI_SSID;
const char* password = WIFI_PASSWORD;

//MQTT Broker configuration
const char* mqtt_broker = MQTT_BROKER;
const int mqtt_port = MQTT_PORT;
const char* mqtt_user = MQTT_USER;
const char* mqtt_pass = MQTT_PASS;

//Topics
const char* topics[] = {
  "home/controls/light",
  "home/controls/fan",
  "home/sensors/ldr/light",
  "home/sensors/dht/temperature",
  "home/sensors/dht/humidity",

};

const int topicsCount = sizeof(topics) / sizeof(topics[0]);

//Sensor configuration
#define DHTPIN
#define DHTTYPE DHT11
DHT dht(DHTPIN, DHTTYPE);

//Actuator configuration
#define LIGHT_PIN
#define FAN_PIN
#define MOTOR_PIN1
#define MOTOR_PIN2
#define SOUND_PIN
#define SERVO_PIN
//LDR light sensor is on A0 (no #define needed)

//MQTT Client
WiFiClient espClient;
PubSubClient client(espClient);

//Function prototypes
void setupWifi();
void mqttCallback(char* topic, byte* payload, unsigned int length);
void reconnectMQTT();
void setup();
void loop(); 

//Functions
void setup() {
    Serial.begin(115200);
    client.setServer(mqtt_broker, mqtt_port);
    client.setCallback(mqttCallback);
    setupWifi();

    //Initialize actuators
    pinMode(LIGHT_PIN, OUTPUT);
    pinMode(FAN_PIN, OUTPUT);
    pinMode(MOTOR_PIN1, OUTPUT);
    pinMode(MOTOR_PIN2, OUTPUT);
    pinMode(SOUND_PIN, OUTPUT);
    pinMode(SERVO_PIN, OUTPUT);

    //Initialize sensors
    dht.begin();
}

void loop() {
    if (!client.connected()) { //Check if MQTT client is connected
        reconnectMQTT();
    }
    
    client.loop(); //Process incoming messages

    if (millis() % 10000 == 0) { //Publish sensor data every 10 seconds
        float humidity = dht.readHumidity();
        float temperature = dht.readTemperature();
        int lightLevel = analogRead(A0);

        if (isnan(humidity) 
        || isnan(temperature)) {
            Serial.println("Failed to read from DHT sensor!");
            return;
        }

        String tempStr = String(temperature);
        String humidityStr = String(humidity);
        String lightStr = String(lightLevel);

        client.publish(topics[2], lightStr.c_str());
        client.publish(topics[3], temperatureStr.c_str());
        client.publish(topics[4], humidityStr.c_str());

        Serial.print("Published data - Temperature: " + tempStr);
        Serial.print(", Humidity: " + humidityStr);
        Serial.print(", Light Level: " + lightStr);

        Serial.println("Successfully published sensor data:");
    }

}

void setupWifi() { //wifi setup function
    WiFi.begin(ssid, password);
    while (WiFi.status() != WL_CONNECTED) {
        delay(500);
        Serial.print(".");
    }
    Serial.println("Connected to WiFi, IP Address: " + WiFi.localIP().toString());
    Serial.println("================================================================================");
}

void reconnectMQTT() { //reconnect to MQTT broker
    while (!client.connected()) {
        Serial.print("Attempting MQTT connection...");
        if (client.connect("ESP8266Client", mqtt_user, mqtt_pass)) {
            Serial.println("connected");

            //Subscribe to topics
            for (int i = 0; i < topicsCount; i++) {
                client.subscribe(topics[i]);
            }
            Serial.println("Subscribed to topics:");
            for (int i = 0; i < topicsCount; i++) {
                Serial.println(topics[i]);
            }
            Serial.println("================================================================================");
        } else {
            Serial.print("failed, rc=");
            Serial.print(client.state());
            delay(2000);
        }
    }
}

void mqttCallback(char* topic, byte* payload, unsigned int length) { //callback function for MQTT messages
    Serial.print("Message arrived [");
    Serial.print(topic);
    Serial.print("]: ");
    
    String messageTemp;
    for (int i = 0; i < length; i++) {
        messageTemp += (char)payload[i];
    }
    
    Serial.println(messageTemp);

    //Control actuators based on received messages
    if (String(topic) == topics[0]) { //Light control
        if (messageTemp == "ON") {
            digitalWrite(LIGHT_PIN, HIGH);
        } else if (messageTemp == "OFF") {
            digitalWrite(LIGHT_PIN, LOW);
        }

    } else if (String(topic) == topics[1]) { //Fan control
        if (messageTemp == "ON") {
            digitalWrite(FAN_PIN, HIGH);
        } else if (messageTemp == "OFF") {
            digitalWrite(FAN_PIN, LOW);
        }
    }

}