#include <ESP8266WiFi.h>
#include <PubSubClient.h>
#include <Wire.h>
#include <Adafruit_Sensor.h>

// Sensor libraries
#include <Adafruit_BH1750.h>
#include <DHT.h>
#include <Adafruit_APDS9960.h>
#include <Adafruit_BMP280.h>
#include <Adafruit_TCS34725.h>
#include <VL53L0X.h>
#include <MAX30100_PulseOximeter.h>

#include <Servo.h>

//Configurations
#include "config.h" // Define WIFI_SSID, WIFI_PASSWORD, MQTT_BROKER, MQTT_PORT, MQTT_USER, MQTT_PASS or remove this line if not using a config file

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
  "home/controls/dc_motor",
  "home/controls/servo",
  "home/sensors/bh1750/light",
  "home/sensors/dht/temperature",
  "home/sensors/dht/humidity",
  "home/sensors/sound",
  "home/sensors/gesture",
  "home/sensors/motion",
  "home/sensors/laser_distance",
  "home/sensors/heart_rate",
  "home/sensors/pressure",
  "home/sensors/color"
};

const int topicsCount = sizeof(topics) / sizeof(topics[0]);

// ------------------- Pin Definitions -------------------

// I2C pins (shared by many sensors)
#define I2C_SDA_PIN 4 // GPIO4 (D2)
#define I2C_SCL_PIN 5 // GPIO5 (D1)

// Sensors with dedicated pins
#define DHTPIN 2       // GPIO2 (D4) - DHT11 (Temperature & Humidity)
#define PIR_PIN 0      // GPIO0 (D3) - Motion PIR sensor (moved to avoid conflict with I2C)
#define SOUND_PIN A0   // Analog pin for sound sensor

// Actuators (Controls)
#define LIGHT_PIN 5    // GPIO5 (D1) - Light output
#define FAN_PIN 16     // GPIO16 (D0) - Fan output

#define DC_MOTOR_PIN1 12  // GPIO12 (D6) - DC motor control pin 1
#define DC_MOTOR_PIN2 13  // GPIO13 (D7) - DC motor control pin 2

#define SERVO_PIN 14   // GPIO14 (D5) - Servo motor control

// ------------------- Objects -------------------

//MQTT Client
WiFiClient espClient;
PubSubClient client(espClient);

Adafruit_BH1750 bh1750Sensor;
DHT dht(DHTPIN, DHT11);
Adafruit_APDS9960 gestureSensor;
Adafruit_BMP280 bmp;
Adafruit_TCS34725 colorSensor;
VL53L0X laserSensor;
MAX30100_PulseOximeter heartRateSensor;
Servo servo;


unsigned long lastSensorPublish = 0;
const unsigned long SENSOR_PUBLISH_INTERVAL = 10000;  // 10 seconds publish interval

float heartRate = 0; // variable to hold heart rate reading

// ------------------- Setups -------------------
void setupWifi();
void mqttCallback(char* topic, byte* payload, unsigned int length);
void reconnectMQTT();
void setup();
void loop(); 

// ------------------- Functions -------------------
void setup() {
    Serial.begin(115200);
    client.setServer(mqtt_broker, mqtt_port);
    client.setCallback(mqttCallback);
    setupWifi();

     // Initialize pins for Actuators
    pinMode(LIGHT_PIN, OUTPUT);
    pinMode(FAN_PIN, OUTPUT);

    pinMode(DC_MOTOR_PIN1, OUTPUT);
    pinMode(DC_MOTOR_PIN2, OUTPUT);
    stopDCMotor(); // ensure motor off on start

    pinMode(PIR_PIN, INPUT);
    pinMode(SOUND_PIN, INPUT);

    servo.attach(SERVO_PIN);
    servo.write(90); // neutral servo position

    // Initialize I2C bus explicitly with defined pins
    Wire.begin(I2C_SDA_PIN, I2C_SCL_PIN);

    // Initialize sensors
    if (!bh1750Sensor.begin(BH1750::CONTINUOUS_HIGH_RES_MODE)) {
        Serial.println("Failed to initialize BH1750");
    }

    dht.begin();

    if (!gestureSensor.begin()) {
        Serial.println("Failed to initialize APDS9960 Gesture sensor");
    } else {
        gestureSensor.enableGesture(true);
    }

    if (!bmp.begin(0x76)) {
        Serial.println("Failed to initialize BMP280");
    }

    if (!colorSensor.begin()) {
        Serial.println("Failed to initialize TCS34725");
    }

    laserSensor.setTimeout(500);
    if (!laserSensor.init()) {
        Serial.println("Failed to initialize VL53L0X");
    } else {
        laserSensor.startContinuous();
    }

    if (!heartRateSensor.begin()) {
        Serial.println("Failed to initialize MAX30100");
    } else {
        heartRateSensor.setIRLedCurrent(MAX30100_LED_CURR_7_6MA);
    }

    setupWifi();

    client.setServer(mqtt_broker, mqtt_port);
    client.setCallback(mqttCallback);

    Serial.println("Setup Complete");

}

// ------------------- WiFi and MQTT -------------------

void setupWifi() {
    Serial.print("Connecting to WiFi...");
    WiFi.begin(ssid, password);
    while (WiFi.status() != WL_CONNECTED) {
        delay(500);
        Serial.print(".");
    }
    Serial.println("connected");
    Serial.print("IP Address: ");
    Serial.println(WiFi.localIP());
}

void reconnectMQTT() {
  while (!client.connected()) {
    Serial.print("Attempting MQTT connection...");
    if (client.connect("ESP8266Client", mqtt_user, mqtt_pass)) {
      Serial.println("connected");
      for (int i = 0; i < topicsCount; i++) {
        client.subscribe(topics[i]);
      }
      Serial.println("Subscribed to MQTT topics");
    } else {
      Serial.print("failed, rc=");
      Serial.print(client.state());
      Serial.println("; retry in 5 sec");
      delay(5000);
    }
  }
}

// ------------------- MQTT Callback -------------------
void mqttCallback(char* topic, byte* payload, unsigned int length) {
  String message = "";
  for (unsigned int i = 0; i < length; i++)
    message += (char)payload[i];

  Serial.printf("MQTT message received [%s]: %s\n", topic, message.c_str());

  if (String(topic) == String(topics[0])) {  // Light control
    digitalWrite(LIGHT_PIN, message == "ON" ? HIGH : LOW);
  }
  else if (String(topic) == String(topics[1])) {  // Fan control
    digitalWrite(FAN_PIN, message == "ON" ? HIGH : LOW);
  }
  else if (String(topic) == String(topics[2])) {  // DC Motor control
    if (message == "stop") {
      stopDCMotor();
    } else if (message.startsWith("forward_speed_")) {
      int speed = message.substring(14).toInt();
      moveDCMotorForward(speed);
    } else if (message.startsWith("backward_speed_")) {
      int speed = message.substring(15).toInt();
      moveDCMotorBackward(speed);
    }
  }
  else if (String(topic) == String(topics[3])) {  // Servo control
    if (message.startsWith("angle_")) {
      int angle = message.substring(6).toInt();
      if (angle >= 0 && angle <= 180) {
        servo.write(angle);
        Serial.printf("Servo angle set to %d\n", angle);
      }
    }
  }
}

// ------------------- DC Motor Control -------------------
void moveDCMotorForward(int speed) {
  speed = constrain(speed, 0, 255);
  analogWrite(DC_MOTOR_PIN1, speed);  // PWM on pin1 for forward speed
  digitalWrite(DC_MOTOR_PIN2, LOW);
  Serial.printf("DC Motor forward at speed %d\n", speed);
}

void moveDCMotorBackward(int speed) {
  speed = constrain(speed, 0, 255);
  analogWrite(DC_MOTOR_PIN2, speed);  // PWM on pin2 for backward speed
  digitalWrite(DC_MOTOR_PIN1, LOW);
  Serial.printf("DC Motor backward at speed %d\n", speed);
}

void stopDCMotor() {
  digitalWrite(DC_MOTOR_PIN1, LOW);
  digitalWrite(DC_MOTOR_PIN2, LOW);
  Serial.println("DC Motor stopped");
}

void loop() {
  if (!client.connected()) reconnectMQTT();
  client.loop();

  heartRateSensor.update();

  if (millis() - lastSensorPublish >= SENSOR_PUBLISH_INTERVAL) {
    lastSensorPublish = millis();
    publishSensorData();
  }
}

// ------------------- Publishing Sensor Data -------------------

void publishSensorData() {
  float lux = bh1750Sensor.readLightLevel();

  float temperature = dht.readTemperature();
  float humidity = dht.readHumidity();

  String gestureStr = "none";
  if (gestureSensor.isGestureAvailable()) {
    switch (gestureSensor.readGesture()) {
      case APDS9960_UP: gestureStr = "up"; break;
      case APDS9960_DOWN: gestureStr = "down"; break;
      case APDS9960_LEFT: gestureStr = "left"; break;
      case APDS9960_RIGHT: gestureStr = "right"; break;
      case APDS9960_NEAR: gestureStr = "near"; break;
      case APDS9960_FAR: gestureStr = "far"; break;
      default: break;
    }
  }

  float pressure = bmp.readPressure() / 100.0F;

  uint16_t r, g, b, c;
  colorSensor.getRawData(&r, &g, &b, &c);
  float colorLux = colorSensor.calculateLux(r, g, b);
  float colorTemp = colorSensor.calculateColorTemperature(r, g, b);

  uint16_t distance = laserSensor.readRangeContinuousMillimeters();
  if (laserSensor.timeoutOccurred()) {
    Serial.println("Laser sensor timeout");
    distance = 0;
  }

  int soundLevel = analogRead(SOUND_PIN);

  bool motionDetected = digitalRead(PIR_PIN) == HIGH;

  heartRate = heartRateSensor.getHeartRate();

  if (client.connected()) {
    client.publish(topics[4], String(lux, 2).c_str());
    client.publish(topics[5], isnan(temperature) ? "NaN" : String(temperature, 2).c_str());
    client.publish(topics[6], isnan(humidity) ? "NaN" : String(humidity, 2).c_str());
    client.publish(topics[7], String(soundLevel).c_str());
    client.publish(topics[8], gestureStr.c_str());
    client.publish(topics[9], motionDetected ? "true" : "false");
    client.publish(topics[10], String(distance).c_str());
    client.publish(topics[11], String(heartRate, 2).c_str());
    client.publish(topics[12], String(pressure, 2).c_str());
    // Color sensor JSON payload
    String colorPayload = "{\"r\":" + String(r) + ",\"g\":" + String(g) + ",\"b\":" + String(b) + 
                          ",\"c\":" + String(c) + ",\"lux\":" + String(colorLux, 2) + 
                          ",\"temp\":" + String(colorTemp, 2) + "}";
    client.publish(topics[13], colorPayload.c_str());
  }

  Serial.println("Published sensor data:");
  Serial.printf("Light Lux: %.2f\nTemp: %.2f °C\nHumidity: %.2f %%\n", lux, temperature, humidity);
  Serial.printf("Gesture: %s\nMotion: %s\n", gestureStr.c_str(), motionDetected ? "Yes" : "No");
  Serial.printf("Laser Distance: %d mm\nHeart Rate: %.2f bpm\n", distance, heartRate);
  Serial.printf("Pressure: %.2f hPa\nSound Level: %d\n", pressure, soundLevel);
  Serial.printf("Color(R,G,B,C): %d, %d, %d, %d\nColor Lux: %.2f\nColor Temp: %.2f K\n", r, g, b, c, colorLux, colorTemp);
}