#include <WiFi.h>
#include <PubSubClient.h>
#include <ModbusRTUMaster.h>
#include <ArduinoJson.h>
//#include "OTA.h"

// Replace with your WiFi credentials
const char* ssid = "Sandun Meesara Xperia XZ2";
const char* password = "Sandun6755";

// Replace with your MQTT broker details
const char* mqtt_server = "35.187.228.48";
const int mqtt_port = 1883; // Default MQTT port
const char* mqtt_user = "sandun";
const char* mqtt_password = "Sandun2000";
//const char* mqtt_user = "azurevm01";
//const char* mqtt_password = "azurevm01passwd";

// Replace with your sensor topic
const char* sensor_topic = "Machine_01";

WiFiClient espClient;
PubSubClient client(espClient);

//pin define for max 485 module
const int8_t rxPin = 16;
const int8_t txPin = 17;
const uint8_t dePin = 4;

ModbusRTUMaster modbus(Serial2, dePin); // serial Serial interface, driver enable pin for rs-485 (optional)

uint16_t holdingRegisters[2];
uint32_t total,x;
int dataAddress;
uint16_t UrAt,IrAt;
float floatResult;

// Proximity sensor settings
const int sensorPin = 27; // Pin connected to the proximity sensor
int interruptCounter = 0;
//QueueHandle_t interruptQueue; // Queue to share interrupt counts between cores

// Function prototypes
void modbusTask(void* parameter);
void interruptTask(void* parameter);
void IRAM_ATTR handleInterrupt();


void setup() {

  Serial.begin(115200); // For debugging
  modbus.begin(9600, SERIAL_8N1, rxPin, txPin);
  //ArduinoOTA.setHostname("54-K_ESP32");
  //setupOTA();

  WiFi.begin(ssid, password);
  while (WiFi.status() != WL_CONNECTED) {
    delay(500);
    Serial.print(".");
  }
  Serial.println("");
  Serial.println("WiFi connected");

  client.setServer(mqtt_server, mqtt_port);
  client.setCallback(callback); // Optional for receiving messages

  //interruptQueue = xQueueCreate(1, sizeof(int)); // Create a queue for interrupt counts, with a capacity of 1 integer

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
  //ArduinoOTA.handle();

  if (!client.connected()) {
    reconnect();
  }
  client.loop();

}

void modbusTask(void* parameter) {
  // Your Modbus and MQTT code here
  for (;;) {
    // Create a JSON objects for each data categories
  StaticJsonDocument<200> jsonDoc1;
  //JsonObject Energy_Meter_Data = jsonDoc1.createNestedObject("Energy_Meter_Data");
  //JsonObject Voltage_Data = jsonDoc1.createNestedObject("Voltage_Data");
  //JsonObject Current_Data = jsonDoc1.createNestedObject("Current_Data");

  StaticJsonDocument<200> jsonDoc2;
  //JsonObject Power_Data = jsonDoc2.createNestedObject("Power_Data");

  StaticJsonDocument<200> jsonDoc3;
  //JsonObject Power_Secondary_Data = jsonDoc3.createNestedObject("Power_Secondary_Data");

  
  //print Software version
  dataAddress = 0x00;
  Serial.println("Software version : " + String(readIntData(dataAddress)));
  //jsonDoc1["Soft_Ver."] = readIntData(dataAddress);
 
  //print Programming Code
  dataAddress = 0x01;
  Serial.println("Programming Code : " + String(readIntData(dataAddress)));
  //jsonDoc1["Pro_Code"] = readIntData(dataAddress);

  //print Network Selection
  dataAddress = 0x03;
  Serial.println("Network Selection : " + String(readIntData(dataAddress)));
  //jsonDoc1["Net_Select"] = readIntData(dataAddress);
  
  //print Current Transformer Rate(IrAt)
  dataAddress = 0x06;
  IrAt = readIntData(dataAddress);
  Serial.println("Current Transformer Rate(IrAt) : " + String(IrAt));
  jsonDoc1["IrAt"] = IrAt;
  

  //print Voltage Transformer Rate(UrAt)
  dataAddress = 0x07;
  UrAt = readIntData(dataAddress);
  Serial.println("Voltage Transformer Rate(UrAt) : " + String(UrAt));
  jsonDoc1["UrAt"] = UrAt;

  //print Rotating Display Time(s)
  dataAddress = 0x0A;
  Serial.println("Rotating Display Time(s) : " + String(readIntData(dataAddress)));

  //print Backlight Time Control(s)
  dataAddress = 0x0B;
  Serial.println("Backlight Time Control(s) : " + String(readIntData(dataAddress)));

  //print Protocol Switching
  dataAddress = 0x2C;
  Serial.println("Protocol Switching : " + String(readIntData(dataAddress)));
  //jsonDoc1["Protocol"] = readIntData(dataAddress);

  //print Communication Baud Rate
  dataAddress = 0x2D;
  Serial.println("Communication Baud Rate : " + String(readIntData(dataAddress)));
  //jsonDoc1["Baud_Rate"] = readIntData(dataAddress);

  //print Communication Address
  dataAddress = 0x2E;
  Serial.println("Communication Address : " + String(readIntData(dataAddress)));
  //jsonDoc1["D_Id"] = readIntData(dataAddress);


  //print Three Phase line voltage(Uab)
  dataAddress = 0x2000;
  floatResult = readFloatData(dataAddress) * 0.1;
  Serial.println("Uab : " + String(floatResult) + "v");
  jsonDoc1["Uab"] = floatResult;

  //print Three Phase line voltage(Ubc)
  dataAddress = 0x2002;
  floatResult = readFloatData(dataAddress) * 0.1;
  Serial.println("Ubc : " + String(floatResult) + "v");
  jsonDoc1["Ubc"] = floatResult;

  //print Three Phase line voltage(Uca)
  dataAddress = 0x2004;
  floatResult = readFloatData(dataAddress) * 0.1;
  Serial.println("Uca : " + String(floatResult) + "v");
  jsonDoc1["Uca"] = floatResult;

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
  jsonDoc2["Pt"] = floatResult;

  //print A Phase active power(Pa)
  dataAddress = 0x2014;
  floatResult = readFloatData(dataAddress) * IrAt * 0.1;
  Serial.println("Pa : " + String(floatResult) + "W");
  jsonDoc2["Pa"] = floatResult;

  //print B Phase active power(Pb)
  dataAddress = 0x2016;
  floatResult = readFloatData(dataAddress) * IrAt * 0.1;
  Serial.println("Pb : " + String(floatResult) + "W");
  jsonDoc2["Pb"] = floatResult;

  //print C Phase active power(Pc)
  dataAddress = 0x2018;
  floatResult = readFloatData(dataAddress) * IrAt * 0.1;
  Serial.println("Pc : " + String(floatResult) + "W");
  jsonDoc2["Pc"] = floatResult;

  //print Combined Reactive Power(Qt)
  dataAddress = 0x201A;
  floatResult = readFloatData(dataAddress) * IrAt * 0.1;
  Serial.println("Qt : " + String(floatResult) + "var");
  jsonDoc2["Qt"] = floatResult;

  //print A Phase Reactive Power(Qa)
  dataAddress = 0x201C;
  floatResult = readFloatData(dataAddress) * IrAt * 0.1;
  Serial.println("Qa : " + String(floatResult) + "var");
  jsonDoc2["Qa"] = floatResult;

  //print B Phase Reactive Power(Qb)
  dataAddress = 0x201E;
  floatResult = readFloatData(dataAddress) * IrAt * 0.1;
  Serial.println("Qb : " + String(floatResult) + "var");
  jsonDoc2["Qb"] = floatResult;

  //print C Phase Reactive Power(Qc)
  dataAddress = 0x2020;
  floatResult = readFloatData(dataAddress) * IrAt * 0.1;
  Serial.println("Qc : " + String(floatResult) + "var");
  jsonDoc2["Qc"] = floatResult;

  //print Combined Power Factor(PFt)
  dataAddress = 0x202A;
  floatResult = readFloatData(dataAddress) * 0.001;
  Serial.println("PFt : " + String(floatResult));
  jsonDoc2["PFt"] = floatResult;

  //print A Phase Power Factor(PFa)
  dataAddress = 0x202C;
  floatResult = readFloatData(dataAddress) * 0.001;
  Serial.println("PFa : " + String(floatResult));
  jsonDoc2["PFa"] = floatResult;

  //print B Phase Power Factor(PFb)
  dataAddress = 0x202E;
  floatResult = readFloatData(dataAddress) * 0.001;
  Serial.println("PFb : " + String(floatResult));
  jsonDoc3["PFb"] = floatResult;

  //print C Phase Power Factor(PFc)
  dataAddress = 0x2030;
  floatResult = readFloatData(dataAddress) * 0.001;
  Serial.println("PFc : " + String(floatResult));
  jsonDoc3["PFc"] = floatResult;

  //print Frequency
  dataAddress = 0x2044;
  floatResult = readFloatData(dataAddress) * 0.01;
  Serial.println("Frequency : " + String(floatResult) + "Hz");
  jsonDoc3["Freq"] = floatResult;

  //print Forward Total Active Energy - ImpEp
  dataAddress = 0x101E;
  floatResult = readFloatData(dataAddress) * UrAt * 0.1 * IrAt;
  Serial.println("Forward total active energy(ImpEp)  : " + String(floatResult) + "kWh");
  jsonDoc3["ImpEp"] = floatResult;

  //print Reverse Total Active Energy - ExpEp
  dataAddress = 0x1028;
  floatResult = readFloatData(dataAddress) * IrAt;
  Serial.println("Reverse total active energy(ExpEp)  : " + String(floatResult) + "kWh");
  jsonDoc3["ExpEp"] = floatResult;

  //print Total reactive energy of the first quadrant 
  dataAddress = 0x1032;
  floatResult = readFloatData(dataAddress) * IrAt;
  Serial.println("Total reactive energy of the first quadrant : " + String(floatResult) + "kvarh");
  jsonDoc3["1Q"] = floatResult;

  //print Total reactive energy of the second quadrant 
  dataAddress = 0x103C;
  floatResult = readFloatData(dataAddress) * IrAt;
  Serial.println("Total reactive energy of the second quadrant : " + String(floatResult) + "kvarh");
  jsonDoc3["2Q"] = floatResult;

  //print Total reactive energy of the third quadrant 
  dataAddress = 0x1046;
  floatResult = readFloatData(dataAddress) * IrAt;
  Serial.println("Total reactive energy of the third quadrant : " + String(floatResult) + "kvarh");
  jsonDoc3["3Q"] = floatResult;
  
  //print Total reactive energy of the fourth quadrant 
  dataAddress = 0x1050;
  floatResult = readFloatData(dataAddress) * IrAt;
  Serial.println("Total reactive energy of the fourth quadrant : " + String(floatResult) + "kvarh");
  jsonDoc3["4Q"] = floatResult;

  //Serial.println(interruptCounter);
  jsonDoc3["Count"] = interruptCounter; 

  // Serialize the JSON objects to a strings
  char jsonString1[200];
  serializeJson(jsonDoc1, jsonString1);

  char jsonString2[200];
  serializeJson(jsonDoc2, jsonString2);

  char jsonString3[200];
  serializeJson(jsonDoc3, jsonString3);

  // Publish the JSON string to a MQTT topic
  client.publish(sensor_topic, jsonString1);
  delay(350);
  client.publish(sensor_topic, jsonString2);
  delay(350);
  client.publish(sensor_topic, jsonString3);
  delay(350);
  
  Serial.println("Message sent to MQTT");
  
  //------------------------------------------------------------------------------------------

  //for debugging
  Serial.println("--------Debugging-----------");

  if (client.publish(sensor_topic, jsonString1)) {
    Serial.println("Message1 sent to MQTT successfully");
  } else {
    Serial.println("Failed to publish message to MQTT");
  }

  if (client.publish(sensor_topic, jsonString2)) {
    Serial.println("Message2 sent to MQTT successfully");
  } else {
    Serial.println("Failed to publish message to MQTT");
  }

  if (client.publish(sensor_topic, jsonString3)) {
    Serial.println("Message2 sent to MQTT successfully");
  } else {
    Serial.println("Failed to publish message to MQTT");
  }

  Serial.print("MQTT Connection State: ");
  Serial.println(client.state());

  Serial.print("Wi-Fi Status: ");
  Serial.println(WiFi.status());

  //client.publish(sensor_topic, jsonString, true);  // Set retained flag to true

  Serial.println("JSON content: " + String(jsonString1));
  Serial.println("JSON content: " + String(jsonString2));
  Serial.println("JSON content: " + String(jsonString3));
  // Check available stack space
  Serial.println("Available stack space: " + String(uxTaskGetStackHighWaterMark(NULL)));

  //------------------------------------------------------------------------------------------

  Serial.println("---------------------------------------");
  delay(1000);
  
  }
}

void IRAM_ATTR handleInterrupt() {
  interruptCounter++; // Increment the counter on each interrupt
  //xQueueSendFromISR(interruptQueue, &interruptCounter, NULL); // Send interrupt count to the queue
}

void interruptTask(void* parameter) {
  pinMode(sensorPin, INPUT_PULLUP); // Set the sensor pin as input with internal pull-up resistor
  attachInterrupt(digitalPinToInterrupt(sensorPin), handleInterrupt, RISING); // Attach interrupt to the sensor pin

  //int receivedCount = 0;
  for (;;) {
    //Serial.println(interruptCounter);
    vTaskDelay(pdMS_TO_TICKS(1000)); // Delay for 1000 milliseconds
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
