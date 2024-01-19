#include <WiFi.h>
#include <PubSubClient.h>

// Replace with your WiFi credentials
const char* ssid = "Sandun Meesara Xperia XZ2";
const char* password = "Sandun6754";

// Replace with your MQTT broker details
const char* mqtt_server = "20.197.23.81";
const int mqtt_port = 1883; // Default MQTT port
const char* mqtt_user = "azurevm01";
const char* mqtt_password = "azurevm01passwd";

// Replace with your sensor topic
const char* sensor_topic = "ESP";

WiFiClient espClient;
PubSubClient client(espClient);


void setup() {
  Serial.begin(115200);

  WiFi.begin(ssid, password);
  while (WiFi.status() != WL_CONNECTED) {
    delay(500);
    Serial.print(".");
  }
  Serial.println("");
  Serial.println("WiFi connected");

  client.setServer(mqtt_server, mqtt_port);
  client.setCallback(callback); // Optional for receiving messages
}

void reconnect() {
  while (!client.connected()) {
    if (client.connect("ESP32Client", mqtt_user, mqtt_password)) {
      Serial.println("MQTT connected");
    } else {
      Serial.print("failed, rc=");
      Serial.print(client.state());
      Serial.println(" try again in 5 seconds");
      delay(5000);
    }
  }
}

void loop() {
  if (!client.connected()) {
    reconnect();
  }
  client.loop();

  // Publish the message to the MQTT topic
  for(int i=0;i<1000;i++){
    String payload = String(i);
    client.publish(sensor_topic, payload.c_str());
    delay(1000); // Update interval (adjust as needed)
  }
  
}

// Optional callback function for receiving MQTT messages
void callback(char* topic, byte* payload, unsigned int length) {
  Serial.print("Message arrived [");
  Serial.print(topic);
  Serial.print("] ");
  for (int i = 0; i < length; i++) {
    Serial.print((char)payload[i]);
  }
  Serial.println();
}
