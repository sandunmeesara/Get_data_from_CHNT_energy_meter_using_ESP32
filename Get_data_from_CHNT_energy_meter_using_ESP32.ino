#include <WiFi.h>
#include <PubSubClient.h>
#include <ModbusRTUMaster.h>
#include <ArduinoJson.h>
#include <ESPmDNS.h>
#include <WiFiUdp.h>
#include <ArduinoOTA.h>

// Replace with your WiFi credentials
const char* ssid = "ENG";
const char* password = "123456789#";

// Replace with your MQTT broker details
const char* mqtt_server = "192.168.1.50";
const int mqtt_port = 1883; // Default MQTT port
const char* mqtt_user = "Mosq_Admin";
const char* mqtt_password = "iot@MPLmqtt24";

// Replace with your sensor topic
const char* sensor_topic = "54K-2";
const int Int_Threshold = 1700;

//Pin define for max 485 module
const int8_t rxPin = 16;
const int8_t txPin = 17;
const uint8_t dePin = 4;

//Object creation
WiFiServer telnetServer(23);
WiFiClient telnetClient;
WiFiClient espClient;
PubSubClient client(espClient);
ModbusRTUMaster modbus(Serial2, dePin); // serial Serial interface, driver enable pin for rs-485 (optional)

//Variables and Constants for Modbus communication
uint16_t holdingRegisters[2];
uint32_t total,x;
int dataAddress;
uint16_t UrAt,IrAt;
float floatResult;
int count_for_reboot = 0;

//Variables and Constants for Sensor
const int sensorPin = 5; // Pin connected to the proximity sensor
volatile unsigned long previousMillis = 0;
volatile unsigned long elapsedTime = 0;
volatile unsigned long int_previousMillis = 0;
volatile float int_elapsedTime = 0;
unsigned long previousMillis_delay = 0;
int rpm = 0;
int interruptCounter = 0;
volatile bool firstInterrupt = true;

// Function prototypes
void modbusTask(void* parameter);
void interruptTask(void* parameter);
void IRAM_ATTR handleInterrupt();

void setup() {

  pinMode(2,OUTPUT); // Onboard Blue LED for MQTT connected indication
  Serial.begin(115200); // For debugging purposes
  SetupOTA(sensor_topic,"IOT@mpl");//Replace this password with your OTA password
  telnetServer.begin(); // For remote debugging purposes
  modbus.begin(9600, SERIAL_8E1, rxPin, txPin);
  client.setServer(mqtt_server, mqtt_port);
  
  //Tasks Section

  xTaskCreatePinnedToCore(
    modbusTask,        // Function to run on core 0
    "ModbusTask",      // Task name
    10000,             // Stack size (bytes)
    NULL,              // Parameter to pass to the task
    1,                 // Priority (1 is default)
    NULL,              // Task handle
    0);                // Core to run the task (0 is core 0)

  xTaskCreatePinnedToCore(
    interruptTask,     // Function to run on core 1
    "InterruptTask",   // Task name
    10000,             // Stack size (bytes)
    NULL,              // Parameter to pass to the task
    1,                 // Priority (1 is default)
    NULL,              // Task handle
    1);                // Core to run the task (1 is core 1)

}

// Local Functions for Tasks

void Reconnect() {

  while (!client.connected()) {
    if (client.connect(sensor_topic, mqtt_user, mqtt_password)) {
      Serial.println("MQTT connected");
      telnetClient.println("MQTT connected");
      count_for_reboot = 0;
      digitalWrite(2,HIGH);
    } else {
      digitalWrite(2,LOW);
      Serial.print("failed, rc=");
      telnetClient.print("failed, rc=");
      Serial.print(client.state());
      telnetClient.print(client.state());
      Serial.println(" try again in 1 seconds");
      telnetClient.println(" try again in 1 seconds");

      count_for_reboot += 1;
      if(count_for_reboot > 2){
        telnetClient.println("Rebooting...");
        ESP.restart();
      }
      vTaskDelay(pdMS_TO_TICKS(1000));
    }
  }
}


void IRAM_ATTR handleInterrupt() {
  //increament counter
  //interruptCounter += 1;
  unsigned long currentMillis_delay = millis(); // Get the current time
  long x_time = currentMillis_delay - previousMillis_delay;
  if (x_time > Int_Threshold) {
    interruptCounter += 1;
    previousMillis_delay = currentMillis_delay; // Save the time of the first interrupt
  }

  unsigned long int_currentMillis = millis(); // Get the current time
  if (firstInterrupt) {
    int_previousMillis = int_currentMillis; // Save the time of the first interrupt
    firstInterrupt = false;
  } else {
    int_elapsedTime = (int_currentMillis - int_previousMillis)/1000.00; // Calculate the time difference between interrupts
    int_previousMillis = int_currentMillis; // Save the time of the second interrupt for the next calculation
  }

}

uint16_t readIntData(int dataAddress){
  modbus.readHoldingRegisters(1, dataAddress, holdingRegisters,1);
  x = holdingRegisters[0];
  return x;
}

float readFloatData(int dataAddress){

  modbus.readHoldingRegisters(1, dataAddress, holdingRegisters,2);
  total = ((uint32_t)holdingRegisters[0]<<16) | holdingRegisters[1];
  String hexString = String(total, HEX);
  float floatResult = hexToFloat(hexString);
  return floatResult;
}

String decimalToHex(String decimalString) {
  long decimalValue = atol(decimalString.c_str()); // Convert decimal string to long
  char hexBuffer[9]; // Buffer for storing hexadecimal representation (8 characters + null terminator)
  snprintf(hexBuffer, sizeof(hexBuffer), "%08lX", decimalValue); // Convert long to hexadecimal string
  return String(hexBuffer);
}

bool isValidHex(String hexString) {
  for (char c : hexString) {
    if (!isHexadecimalDigit(c)) {
      return false;
    }
  }
  return hexString.length() > 0;
}

bool isHexadecimalDigit(char c) {
  return (c >= '0' && c <= '9') || (c >= 'A' && c <= 'F') || (c >= 'a' && c <= 'f');
}

float hexToFloat(String hexString) {
  unsigned long hexInt = strtoul(hexString.c_str(), NULL, 16); // Convert hexadecimal string to unsigned long
  float result;
  memcpy(&result, &hexInt, sizeof(result)); // Copy the raw bytes into a float variable
  return result;
}

void SetupOTA(const char* OTA_Hostname,const char* OTA_Password) {

  Serial.println("MPL-Sensor-Node is Booting...");
  WiFi.mode(WIFI_STA);
  WiFi.begin(ssid, password);
  while (WiFi.waitForConnectResult() != WL_CONNECTED) {
    Serial.println("Connection Failed! Rebooting...");
    vTaskDelay(pdMS_TO_TICKS(1000));
    ESP.restart();
  }

  // Hostname defaults to esp32-[MAC]
  ArduinoOTA.setHostname(OTA_Hostname);

  // No authentication by default
  ArduinoOTA.setPassword(OTA_Password);
  
  ArduinoOTA
    .onStart([]() {
      String type;
      if (ArduinoOTA.getCommand() == U_FLASH)
        type = "sketch";
      else // U_SPIFFS
        type = "filesystem";

      // NOTE: if updating SPIFFS this would be the place to unmount SPIFFS using SPIFFS.end()
      Serial.println("Start updating " + type);
    })
    .onEnd([]() {
      Serial.println("\nEnd");
    })
    .onProgress([](unsigned int progress, unsigned int total) {
      Serial.printf("Progress: %u%%\r", (progress / (total / 100)));
    })
    .onError([](ota_error_t error) {
      Serial.printf("Error[%u]: ", error);
      if (error == OTA_AUTH_ERROR) Serial.println("Auth Failed");
      else if (error == OTA_BEGIN_ERROR) Serial.println("Begin Failed");
      else if (error == OTA_CONNECT_ERROR) Serial.println("Connect Failed");
      else if (error == OTA_RECEIVE_ERROR) Serial.println("Receive Failed");
      else if (error == OTA_END_ERROR) Serial.println("End Failed");
    });

  ArduinoOTA.begin();

  Serial.println("Ready");
  Serial.print("IP address: ");
  Serial.println(WiFi.localIP());
}


// Task Functions

void modbusTask(void* parameter) {

  //Establishing MQTT Connection
  if (!client.connected()) {
    Reconnect();
  }
  client.loop();


  for (;;) {

  ArduinoOTA.handle();

  // Create a JSON objects for each data categories
  StaticJsonDocument<350> jsonDoc1;
  StaticJsonDocument<200> jsonDoc2;

  //print Current Transformer Rate(IrAt)
  dataAddress = 0x06;
  IrAt = readIntData(dataAddress);

  //print Voltage Transformer Rate(UrAt)
  dataAddress = 0x07;
  UrAt = readIntData(dataAddress);

  //print Three Phase Phase voltage(Ua)
  dataAddress = 0x2006;
  floatResult = readFloatData(dataAddress) * 0.1;
  Serial.println("Ua : " + String(floatResult) + "v");
  jsonDoc1["Ua"] = floatResult;

  //print Three Phase Phase voltage(Ub)
  dataAddress = 0x2008;
  floatResult = readFloatData(dataAddress) * 0.1;
  Serial.println("Ub : " + String(floatResult) + "v");
  jsonDoc1["Ub"] = floatResult;

  //print Three Phase Phase voltage(Uc)
  dataAddress = 0x200A;
  floatResult = readFloatData(dataAddress) * 0.1;
  Serial.println("Uc : " + String(floatResult) + "v");
  jsonDoc1["Uc"] = floatResult;

  //print Three Phase Current(Ia)
  dataAddress = 0x200C;
  floatResult = readFloatData(dataAddress) * IrAt * 0.001;
  Serial.println("Ia : " + String(floatResult) + "A");
  jsonDoc1["Ia"] = floatResult;

  //print Three Phase Current(Ib)
  dataAddress = 0x200E;
  floatResult = readFloatData(dataAddress) * IrAt * 0.001;
  Serial.println("Ib : " + String(floatResult) + "A");
  jsonDoc1["Ib"] = floatResult;

  //print Three Phase Current(Ic)
  dataAddress = 0x2010;
  floatResult = readFloatData(dataAddress) * IrAt * 0.001;
  Serial.println("Ic : " + String(floatResult) + "A");
  jsonDoc1["Ic"] = floatResult;

  //print Combined Active Power(Pt)
  dataAddress = 0x2012;
  floatResult = readFloatData(dataAddress) * IrAt * 0.1;
  Serial.println("Pt : " + String(floatResult) + "W");
  jsonDoc1["Pt"] = floatResult;

  //print A Phase active power(Pa)
  dataAddress = 0x2014;
  floatResult = readFloatData(dataAddress) * IrAt * 0.1;
  Serial.println("Pa : " + String(floatResult) + "W");
  jsonDoc1["Pa"] = floatResult;

  //print B Phase active power(Pb)
  dataAddress = 0x2016;
  floatResult = readFloatData(dataAddress) * IrAt * 0.1;
  Serial.println("Pb : " + String(floatResult) + "W");
  jsonDoc1["Pb"] = floatResult;

  //print C Phase active power(Pc)
  dataAddress = 0x2018;
  floatResult = readFloatData(dataAddress) * IrAt * 0.1;
  Serial.println("Pc : " + String(floatResult) + "W");
  jsonDoc1["Pc"] = floatResult;

  //print Combined Power Factor(PFt)
  dataAddress = 0x202A;
  floatResult = readFloatData(dataAddress) * 0.001;
  Serial.println("PFt : " + String(floatResult));
  jsonDoc2["PFt"] = floatResult;

  //print A Phase Power Factor(PFa)
  dataAddress = 0x202C;
  floatResult = readFloatData(dataAddress) * 0.001;
  Serial.println("PFa : " + String(floatResult));
  jsonDoc1["PFa"] = floatResult;

  //print B Phase Power Factor(PFb)
  dataAddress = 0x202E;
  floatResult = readFloatData(dataAddress) * 0.001;
  Serial.println("PFb : " + String(floatResult));
  jsonDoc1["PFb"] = floatResult;

  //print C Phase Power Factor(PFc)
  dataAddress = 0x2030;
  floatResult = readFloatData(dataAddress) * 0.001;
  Serial.println("PFc : " + String(floatResult));
  jsonDoc1["PFc"] = floatResult;

  //print Frequency
  dataAddress = 0x2044;
  floatResult = readFloatData(dataAddress) * 0.01;
  Serial.println("Frequency : " + String(floatResult) + "Hz");
  jsonDoc2["Freq"] = floatResult;

  //print Forward Total Active Energy - ImpEp
  dataAddress = 0x101E;
  floatResult = readFloatData(dataAddress) * UrAt * 0.1 * IrAt;
  Serial.println("Forward total active energy(ImpEp)  : " + String(floatResult) + "kWh");
  jsonDoc2["ImpEp"] = floatResult;

  //Serial.println(interruptCounter);
  jsonDoc1["Cycle_time(s)"] = int_elapsedTime;  

  //Serial.println(interruptCounter);
  jsonDoc1["RPM"] = rpm;  

  // Serialize the JSON objects to a strings
  char jsonString1[350];
  serializeJson(jsonDoc1, jsonString1);
  char jsonString2[200];
  serializeJson(jsonDoc2, jsonString2);

  // Publish the JSON string to a MQTT topic
  
  client.publish(sensor_topic, jsonString1,true);
  client.publish(sensor_topic, jsonString2,true);
  vTaskDelay(pdMS_TO_TICKS(500));
  
  //client.publish(sensor_topic, jsonString, true);  // Set retained flag to true
  Serial.println("Message sent to MQTT");
  
  //------------------------------------------------------------------------------------------

  //For Debugging
  Serial.println("--------Debugging-----------");

  if (client.publish(sensor_topic, jsonString1)) {
    Serial.println("Message1 sent to MQTT successfully");
    telnetClient.println("Message1 sent to MQTT successfully");
  } else {
    Serial.println("Failed to publish message1 to MQTT");
    telnetClient.println("Failed to publish message1 to MQTT");
  }

  if (client.publish(sensor_topic, jsonString1)) {
    Serial.println("Message2 sent to MQTT successfully");
    telnetClient.println("Message2 sent to MQTT successfully");
  } else {
    Serial.println("Failed to publish message2 to MQTT");
    telnetClient.println("Failed to publish message2 to MQTT");

    count_for_reboot += 1;
    if(count_for_reboot > 2){
      telnetClient.println("Rebooting...");
      ESP.restart();
    }

  }

  Serial.print("MQTT Connection State: ");
  telnetClient.println("MQTT Connection State: ");
  Serial.println(client.state());
  telnetClient.println(client.state());

  Serial.print("Wi-Fi Status: ");
  telnetClient.println("Wi-Fi Status: ");
  Serial.println(WiFi.status());
  telnetClient.println(WiFi.status());

  Serial.println("JSON content: " + String(jsonString1));
  telnetClient.println("JSON content: " + String(jsonString1));
  Serial.println("JSON content: " + String(jsonString2));
  telnetClient.println("JSON content: " + String(jsonString2));

  Serial.println("---------------------------------------");
  telnetClient.println("---------------------------------------");
  
  }
}


void interruptTask(void* parameter) {

  unsigned long currentMillis = millis(); // Get the current time
  previousMillis = currentMillis;
  
  pinMode(sensorPin, INPUT_PULLUP); // Set the sensor pin as input with internal pull-up resistor
  attachInterrupt(digitalPinToInterrupt(sensorPin), handleInterrupt, RISING); // Attach interrupt to the sensor pin

  for (;;) {
    //Establishing Telnet Server
    if (telnetServer.hasClient()) {
      if (!telnetClient || !telnetClient.connected()) {
        if (telnetClient) telnetClient.stop();
        telnetClient = telnetServer.available();
      }
    }

    currentMillis = millis(); // Get the current time
    elapsedTime = (currentMillis - previousMillis);
    if(elapsedTime==60000){
          rpm = interruptCounter;//no of cycles per minute
          interruptCounter = 0;
          previousMillis = currentMillis; // Save the time of the second interrupt for the next calculation
    }

  }

}

void loop() {
  
}
