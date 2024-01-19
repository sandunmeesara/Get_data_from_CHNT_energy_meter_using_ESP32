#include <WiFi.h>
#include <PubSubClient.h>
#include <ModbusRTUMaster.h>
#include <ArduinoJson.h>

// Replace with your WiFi credentials
const char* ssid = "S9+";
const char* password = "Sachin1237";

// Replace with your MQTT broker details
const char* mqtt_server = "20.197.23.81";
const int mqtt_port = 1883; // Default MQTT port
const char* mqtt_user = "azurevm01";
const char* mqtt_password = "azurevm01passwd";

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
float floatResult;


void setup() {

  Serial.begin(115200); // For debugging
  modbus.begin(9600, SERIAL_8N1, rxPin, txPin);

  WiFi.begin(ssid, password);
  while (WiFi.status() != WL_CONNECTED) {
    delay(500);
    Serial.print(".");
  }
  Serial.println("");
  Serial.println("WiFi connected");

  client.setServer(mqtt_server, mqtt_port);
  //client.setSocketTimeout(60);  // Set the timeout to 10 seconds (adjust as needed)
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

  // Create a JSON objects for each data categories
  StaticJsonDocument<200> jsonDoc1;
  JsonObject Energy_Meter_Data = jsonDoc1.createNestedObject("Energy_Meter_Data");
  JsonObject Voltage_Data = jsonDoc1.createNestedObject("Voltage_Data");
  JsonObject Current_Data = jsonDoc1.createNestedObject("Current_Data");

  StaticJsonDocument<200> jsonDoc2;
  JsonObject Power_Data = jsonDoc2.createNestedObject("Power_Data");

  StaticJsonDocument<200> jsonDoc3;
  JsonObject Power_Secondary_Data = jsonDoc3.createNestedObject("Power_Secondary_Data");

  
  //print Software version
  dataAddress = 0x00;
  Serial.println("Software version : " + String(readIntData(dataAddress)));
  //Energy_Meter_Data["Soft_Ver."] = readIntData(dataAddress);
 
  //print Programming Code
  dataAddress = 0x01;
  Serial.println("Programming Code : " + String(readIntData(dataAddress)));
  //Energy_Meter_Data["Pro_Code"] = readIntData(dataAddress);

  //print Network Selection
  dataAddress = 0x03;
  Serial.println("Network Selection : " + String(readIntData(dataAddress)));
  //Energy_Meter_Data["Net_Select"] = readIntData(dataAddress);
  
  //print Current Transformer Rate(IrAt)
  dataAddress = 0x06;
  Serial.println("Current Transformer Rate(IrAt) : " + String(readIntData(dataAddress)));
  Energy_Meter_Data["IrAt"] = readIntData(dataAddress);
  

  //print Voltage Transformer Rate(UrAt)
  dataAddress = 0x07;
  Serial.println("Voltage Transformer Rate(UrAt) : " + String(readIntData(dataAddress)));
  Energy_Meter_Data["UrAt"] = readIntData(dataAddress);

  //print Rotating Display Time(s)
  dataAddress = 0x0A;
  Serial.println("Rotating Display Time(s) : " + String(readIntData(dataAddress)));

  //print Backlight Time Control(s)
  dataAddress = 0x0B;
  Serial.println("Backlight Time Control(s) : " + String(readIntData(dataAddress)));

  //print Protocol Switching
  dataAddress = 0x2C;
  Serial.println("Protocol Switching : " + String(readIntData(dataAddress)));
  //Energy_Meter_Data["Protocol"] = readIntData(dataAddress);

  //print Communication Baud Rate
  dataAddress = 0x2D;
  Serial.println("Communication Baud Rate : " + String(readIntData(dataAddress)));
  //Energy_Meter_Data["Baud_Rate"] = readIntData(dataAddress);

  //print Communication Address
  dataAddress = 0x2E;
  Serial.println("Communication Address : " + String(readIntData(dataAddress)));
  //Energy_Meter_Data["D_Id"] = readIntData(dataAddress);


  //print Three Phase line voltage(Uab)
  dataAddress = 0x2000;
  floatResult = readFloatData(dataAddress) * 0.1;
  Serial.println("Uab : " + String(floatResult) + "v");
  Voltage_Data["Uab"] = String(floatResult);

  //print Three Phase line voltage(Ubc)
  dataAddress = 0x2002;
  floatResult = readFloatData(dataAddress) * 0.1;
  Serial.println("Ubc : " + String(floatResult) + "v");
  Voltage_Data["Ubc"] = String(floatResult);

  //print Three Phase line voltage(Uca)
  dataAddress = 0x2004;
  floatResult = readFloatData(dataAddress) * 0.1;
  Serial.println("Uca : " + String(floatResult) + "v");
  Voltage_Data["Uca"] = String(floatResult);

  //print Three Phase Phase voltage(Ua)
  dataAddress = 0x2006;
  floatResult = readFloatData(dataAddress) * 0.1;
  Serial.println("Ua : " + String(floatResult) + "v");
  Voltage_Data["Ua"] = String(floatResult);

  //print Three Phase Phase voltage(Ub)
  dataAddress = 0x2008;
  floatResult = readFloatData(dataAddress) * 0.1;
  Serial.println("Ub : " + String(floatResult) + "v");
  Voltage_Data["Ub"] = String(floatResult);

  //print Three Phase Phase voltage(Uc)
  dataAddress = 0x200A;
  floatResult = readFloatData(dataAddress) * 0.1;
  Serial.println("Uc : " + String(floatResult) + "v");
  Voltage_Data["Uc"] = String(floatResult);

  //print Three Phase Current(Ia)
  dataAddress = 0x200C;
  floatResult = readFloatData(dataAddress) * 0.001;
  Serial.println("Ia : " + String(floatResult) + "A");
  Current_Data["Ia"] = String(floatResult);

  //print Three Phase Current(Ib)
  dataAddress = 0x200E;
  floatResult = readFloatData(dataAddress) * 0.001;
  Serial.println("Ib : " + String(floatResult) + "A");
  Current_Data["Ib"] = String(floatResult);

  //print Three Phase Current(Ic)
  dataAddress = 0x2010;
  floatResult = readFloatData(dataAddress) * 0.001;
  Serial.println("Ic : " + String(floatResult) + "A");
  Current_Data["Ic"] = String(floatResult);

  //print Combined Active Power(Pt)
  dataAddress = 0x2012;
  floatResult = readFloatData(dataAddress) * 0.1;
  Serial.println("Pt : " + String(floatResult) + "W");
  Power_Data["Pt"] = String(floatResult);

  //print A Phase active power(Pa)
  dataAddress = 0x2014;
  floatResult = readFloatData(dataAddress) * 0.1;
  Serial.println("Pa : " + String(floatResult) + "W");
  Power_Data["Pa"] = String(floatResult);

  //print B Phase active power(Pb)
  dataAddress = 0x2016;
  floatResult = readFloatData(dataAddress) * 0.1;
  Serial.println("Pb : " + String(floatResult) + "W");
  Power_Data["Pb"] = String(floatResult);

  //print C Phase active power(Pc)
  dataAddress = 0x2018;
  floatResult = readFloatData(dataAddress) * 0.1;
  Serial.println("Pc : " + String(floatResult) + "W");
  Power_Data["Pc"] = String(floatResult);

  //print Combined Reactive Power(Qt)
  dataAddress = 0x201A;
  floatResult = readFloatData(dataAddress) * 0.1;
  Serial.println("Qt : " + String(floatResult) + "var");
  Power_Data["Qt"] = String(floatResult);

  //print A Phase Reactive Power(Qa)
  dataAddress = 0x201C;
  floatResult = readFloatData(dataAddress) * 0.1;
  Serial.println("Qa : " + String(floatResult) + "var");
  Power_Data["Qa"] = String(floatResult);

  //print B Phase Reactive Power(Qb)
  dataAddress = 0x201E;
  floatResult = readFloatData(dataAddress) * 0.1;
  Serial.println("Qb : " + String(floatResult) + "var");
  Power_Data["Qb"] = String(floatResult);

  //print C Phase Reactive Power(Qc)
  dataAddress = 0x2020;
  floatResult = readFloatData(dataAddress) * 0.1;
  Serial.println("Qc : " + String(floatResult) + "var");
  Power_Data["Qc"] = String(floatResult);

  //print Combined Power Factor(PFt)
  dataAddress = 0x202A;
  floatResult = readFloatData(dataAddress) * 0.001;
  Serial.println("PFt : " + String(floatResult));
  Power_Data["PFt"] = String(floatResult);

  //print A Phase Power Factor(PFa)
  dataAddress = 0x202C;
  floatResult = readFloatData(dataAddress) * 0.001;
  Serial.println("PFa : " + String(floatResult));
  Power_Data["PFa"] = String(floatResult);

  //print B Phase Power Factor(PFb)
  dataAddress = 0x202E;
  floatResult = readFloatData(dataAddress) * 0.001;
  Serial.println("PFb : " + String(floatResult));
  Power_Data["PDb"] = String(floatResult);

  //print C Phase Power Factor(PFc)
  dataAddress = 0x2030;
  floatResult = readFloatData(dataAddress) * 0.001;
  Serial.println("PFc : " + String(floatResult));
  Power_Data["PFc"] = String(floatResult);

  //print Frequency
  dataAddress = 0x2044;
  floatResult = readFloatData(dataAddress) * 0.01;
  Serial.println("Frequency : " + String(floatResult) + "Hz");
  Power_Data["Freq."] = String(floatResult);

  //print Forward Total Active Energy 
  dataAddress = 0x101E;
  floatResult = readFloatData(dataAddress);
  Serial.println("Forward total active energy  : " + String(floatResult) + "kWh");
  Power_Secondary_Data["Forw_Act_Ene."] = String(floatResult);

  //print Reverse Total Active Energy 
  dataAddress = 0x1028;
  floatResult = readFloatData(dataAddress);
  Serial.println("Reverse total active energy  : " + String(floatResult) + "kWh");
  Power_Secondary_Data["Reve_Act_Ene."] = String(floatResult);

  //print Total reactive energy of the first quadrant 
  dataAddress = 0x1032;
  floatResult = readFloatData(dataAddress);
  Serial.println("Total reactive energy of the first quadrant : " + String(floatResult) + "kvarh");
  Power_Secondary_Data["1Q"] = String(floatResult);

  //print Total reactive energy of the second quadrant 
  dataAddress = 0x103C;
  floatResult = readFloatData(dataAddress);
  Serial.println("Total reactive energy of the second quadrant : " + String(floatResult) + "kvarh");
  Power_Secondary_Data["2Q"] = String(floatResult);

  //print Total reactive energy of the third quadrant 
  dataAddress = 0x1046;
  floatResult = readFloatData(dataAddress);
  Serial.println("Total reactive energy of the third quadrant : " + String(floatResult) + "kvarh");
  Power_Secondary_Data["3Q"] = String(floatResult);
  
  //print Total reactive energy of the fourth quadrant 
  dataAddress = 0x1050;
  floatResult = readFloatData(dataAddress);
  Serial.println("Total reactive energy of the fourth quadrant : " + String(floatResult) + "kvarh");
  Power_Secondary_Data["4Q"] = String(floatResult);


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
  delay(350);
  
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
