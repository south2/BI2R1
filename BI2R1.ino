// bodyinteraction IOT project
// IOT sex toy prototype - control via browser and MQTT
// last working version, V17 and v18 not working properly
#include <ESP8266WiFi.h>
#include <WiFiClient.h>
#include <ESP8266WebServer.h>
#include <ESP8266mDNS.h>
#include <ESP8266HTTPUpdateServer.h>
#include <Wire.h>
#include <PubSubClient.h>
#include <ArduinoJson.h>
#include "WEMOS_Motor.h"

ADC_MODE(ADC_VCC);

#include <DNSServer.h>            //Local DNS Server used for redirecting all requests to the configuration portal
//Local WebServer used to serve the configuration portal
#include <WiFiManager.h>



#define    MPU9250_ADDRESS            0x68
#define    MAG_ADDRESS                0x0C

#define    GYRO_FULL_SCALE_250_DPS    0x00
#define    GYRO_FULL_SCALE_500_DPS    0x08
#define    GYRO_FULL_SCALE_1000_DPS   0x10
#define    GYRO_FULL_SCALE_2000_DPS   0x18

#define    ACC_FULL_SCALE_2_G        0x00
#define    ACC_FULL_SCALE_4_G        0x08
#define    ACC_FULL_SCALE_8_G        0x10
#define    ACC_FULL_SCALE_16_G       0x18

#define Motor_Shield_Address 0x30

const char* ota_host = "bi-esp8266-webupdate";
const char* mqtt_server = "m12.cloudmqtt.com";
uint16_t mqtt_port = 15376;
const char* mqtt_user = "nvcuumkf";
const char* mqtt_password = "C-X6glwisHOP";
const char* device_name = "bi2-one";
const int chip_id = ESP.getChipId();
String id = device_name + String(chip_id);

const char* module = "wemosmini";
//const char* module = "NodeMCU";

int ledPin = 0;   // NodeMCU pad D3 = GPIO 0
int motorPin = 13; // NodeMCU pad D7 = GPIO 13
double sinusValue = 0;

// define constants for four different vibration modes
const int offMode = 0;
const int maxMode = 1;
const int sinusMode = 2;
const int motionMode = 3;
const int constantMode = 4;
const int listenMode = 5;
const int listenAndMotionMode = 6;

int motorMode = offMode; //current mode
int motionVector = 0; //current fusioned motion
// Acceleration in x,y and z direction at t(ime)=1 and time=0
// Geroscop data
int16_t ax, ay, az, ax1, ay1, az1, gx, gy, gz, gx1, gy1, gz1;
int valueMotor; //vibrator motor speed 0-1023

// Init Wemos mini motor shield
//Motor M1(0x30,_MOTOR_B, 1000);//Motor A

// OTA service
ESP8266WebServer httpServer(80);
ESP8266HTTPUpdateServer httpUpdater;

// MQTT service
//WiFiServer server(80);
WiFiClient espclient;
PubSubClient mqttclient(espclient);
char msg[512];
char outMsg[512];

bool requesting = false; // is there a request eg button pressed on webpage

// data structure for JSON object
StaticJsonBuffer<500> jsonBuffer;
JsonObject& root = jsonBuffer.createObject();
//timing of mqtt messages
long now, now2 = 0;
long lastMsg, lastI2Csend = 0;
int value = 0;
int valueLED = LOW;

// This function read Nbytes bytes from I2C device at address Address.
// Put read bytes starting at register Register in the Data array.
void I2Cread(uint8_t Address, uint8_t Register, uint8_t Nbytes, uint8_t* Data)
{
  // Set register address
  Wire.beginTransmission(Address);
  Wire.write(Register);
  Wire.endTransmission();

  // Read Nbytes
  Wire.requestFrom(Address, Nbytes);
  uint8_t index = 0;
  while (Wire.available())
    Data[index++] = Wire.read();
}

// Write a byte (Data) in device (Address) at register (Register)
void I2CwriteByte(uint8_t Address, uint8_t Register, uint8_t Data)
{
  // Set register address
  Wire.beginTransmission(Address);
  Wire.write(Register);
  Wire.write(Data);
  Wire.endTransmission();
}

void setMotorSpeed(float s) { // s is motor speed in % (100=100%=max speed)
  //Serial.print(" in setMotorSpeed   ");

  if (module == "wemosmini") {
    //M1.setmotor( _CW, (float) s/3);
    s = s / 3.5;
    delay(100);
    Wire.beginTransmission(Motor_Shield_Address);
    Wire.write(0 | (byte)0x10); // 0 is motor A, 1 is motor B
    Wire.write(_CW);
    uint16_t _pwm_val = uint16_t(s * 100);

    if (_pwm_val > 10000) {
      _pwm_val = 10000;
    }
    Wire.write((byte)(_pwm_val >> 8));
    Wire.write((byte)_pwm_val);
    Wire.endTransmission();     // stop transmitting
    delay(100);
    Serial.print("set motor speed: ");
    Serial.println(s * 100);
    //Serial.print(" ---   ");
  }
  if (module == "NodeMCU") {
    analogWrite(motorPin, (s * 10.24) - 1 );
  }

}

// Return the response /generate webpage
void generateWebpage() {
  char temp[2000];
  //<meta http-equiv='refresh' content='5'/>\  hinter head ENTFERNT!!!!
  snprintf(temp, 2000,
           "<html>\
<head>\
<title>ESP8266 bodyinteraction.com</title>\
<meta name=\"viewport\" content=\"width=device-width, initial-scale=1\">\
<link rel=\"stylesheet\" href=\"http://www.w3schools.com/lib/w3.css\">\
<style>\
  body { background-color: #cccccc; font-family: Arial, Helvetica, Sans-Serif; Color: #000088; }\
</style>\
<body>Led pin is now: %01d<br>\
Motor pin is now: %01d<br>\
Motion vector is now: %01d<br>\
VCC is now: %01d<br><br>\
<div class=\"w3-container w3-layout-col\">\
  <a href=\"LED=ON\"><input class=\"w3-btn w3-blue w3-border\" type=\"button\" value=\"LED on\"></a>\
  <a href=\"LED=OFF\"><input class=\"w3-btn w3-blue w3-border\" type=\"button\" value=\"LED off\"></a><br><br>\
  <a href=\"MOTOR=OFF\"><input class=\"w3-btn w3-indigo w3-border\" type=\"button\" value=\" Motor Off \"></a>\
  <a href=\"MOTOR=MAX\"><input class=\"w3-btn w3-indigo w3-border\" type=\"button\" value=\" Max Vib \"></a>\
  <a href=\"MOTOR=SINUS\"><input class=\"w3-btn w3-indigo w3-border\" type=\"button\" value=\" Sinus Vib\"></a>\
  <a href=\"MOTOR=MOTION\"><input class=\"w3-btn w3-indigo w3-border\" type=\"button\" value=\" Motion Vib\"></a>\
  <a href=\"MOTOR=LISTEN\"><input class=\"w3-btn w3-indigo w3-border\" type=\"button\" value=\" Listen Vib\"></a>\
  <a href=\"MOTOR=LISTENMOTION\"><input class=\"w3-btn w3-indigo w3-border\" type=\"button\" value=\" Listen and Motion Vib\"></a><br><br>\
  <a href=\"SOFTRESET\"><input class=\"w3-btn w3-red w3-border\" type=\"button\" value=\"Restart\"></a><br>\
</div>\
</body>\
</html>",
           valueLED, valueMotor, motionVector, ESP.getVcc());
  httpServer.send ( 200, "text/html", temp );
}



// function callback is executed when a Mqtt message comes in
// - prints mqtt message
// - parse JSON file
// - execute commands
// to do read sensor data of other bis and adjust speed
void mqttCallback(char* topic, byte* payload, unsigned int length) {
  Serial.print("Message arrived [");
  Serial.print(topic);

  // Unterscheidung nach topic einfügen
  Serial.print("] ");
  char s[length];
  for (int i = 0; i < length; i++) {
    Serial.print((char)payload[i]);
    s[i] = payload[i];
  }
  StaticJsonBuffer<500> jsonBuffer;
  JsonObject& root = jsonBuffer.parseObject(s);
  if (!root.success()) {
    Serial.println("parseObject() failed");
  }
  String messageType = root["messageType"]; //sensor, execute , message

  Serial.print("messageType: ");
  Serial.println(messageType);


  // print message
  if (messageType == "message") {
    String message  = root["message"];
    Serial.print("Incoming message: ");
    Serial.println(message);
  }
  // message type execute
  if (messageType == "execute") {
    String targetDeviceID  = root["targetDeviceID"];
    //if (id==targetDeviceID) {
    String actuator  = root["actuator"];
    int actuatorValue  = root["actuatorValue"];
    String actuatorMode  = root["actuatorMode"];
    Serial.print("actuator: ");
    Serial.println(actuator);
    Serial.print("actuatorValue: ");
    Serial.println(actuatorValue);
    // LED commands
    if (actuator == "LED" && actuatorValue == 1) {
      Serial.println("LED on received");
      digitalWrite(ledPin, HIGH);
      digitalWrite(BUILTIN_LED, LOW);
      valueLED = HIGH;
    }
    if (actuator == "LED" && actuatorValue == 0) {
      Serial.println("LED off received");
      digitalWrite(ledPin, LOW);
      digitalWrite(BUILTIN_LED, HIGH);
      valueLED = LOW;
    }
    // set modes commands
    if (actuator == "motor1" && actuatorMode == "off") {
      setMotorSpeed(0);
      //analogWrite(motorPin, 0);
      valueMotor = 0;
      motorMode = offMode;
    }
    if (actuator == "motor1" && actuatorMode == "sinus") {
      motorMode = sinusMode;
    }
    if (actuator == "motor1" && actuatorMode == "motion") {
      motorMode = motionMode;
    }
    if (actuator == "motor1" && actuatorMode == "listen") {
      motorMode = listenMode;
    }
    if (actuator == "motor1" && actuatorMode == "listenmotion") {
      motorMode = listenAndMotionMode;
    }
    // set motor to max speed
    if (actuator == "motor1" && actuatorMode == "max") {
      motorMode = maxMode;
      valueMotor = 100;
      setMotorSpeed(valueMotor);
      //analogWrite(motorPin,valueMotor);
    }
    // set motor to fixed speed
    if (actuator == "motor1" && actuatorMode == "constant") {
      motorMode = constantMode;
      if (actuatorValue > 100) {
        actuatorValue = 100;
      }
      if (actuatorValue < 0) {
        actuatorValue = 0;
      }
      valueMotor = actuatorValue;
      setMotorSpeed(valueMotor);
      //analogWrite(motorPin,valueMotor);
    }
  }

  if (messageType == "sensor" ) {
    String sensor  = root["sensor"];
    int incomingFusionedData  = root["fusionedData"];
    String deviceName = root["deviceName"]; Serial.print(deviceName);
    String deviceChipId = root["chipId"]; Serial.print(deviceChipId);Serial.print(id);
    if (  id == deviceName + deviceChipId) {Serial.println(" receiving own data");}
    if ( sensor == "mps9250" && id != deviceName + deviceChipId) {
      // incoming sensor data, adjust motor speed when in listen or motion&listen mode
      Serial.print("  incoming sensor data from" );
      Serial.println(deviceName+deviceChipId);
      /*
        if (motorMode==motionMode) {
        Serial.print(" - adjusting speed ");Serial.println(id);
        if (motionVector <=2000) {valueMotor=valueMotor-2;}
        if (motionVector > 2000 && motionVector < 10000) {valueMotor=valueMotor+5;}
        if (motionVector >= 10000) {valueMotor=valueMotor+10;}
        if (valueMotor<50) {valueMotor=50;} //values must be above 500 otherwise the motor is off
        if (valueMotor>100) {valueMotor=100;} //
        setMotorSpeed(valueMotor);
        //analogWrite(motorPin, valueMotor); // set motor speed
        }
      */
      if (motorMode == listenMode) {
        if (incomingFusionedData > 2500) {
          valueMotor = valueMotor + 5;
        }
        if (incomingFusionedData <= 2000) {
          valueMotor = valueMotor - 2;
        }
        if (incomingFusionedData > 2000 && motionVector < 10000) {
          valueMotor = valueMotor + 5;
        }
        if (incomingFusionedData >= 10000) {
          valueMotor = valueMotor + 10;
        }
        if (valueMotor < 50) {
          valueMotor = 50; //values must be above 500 otherwise the motor is off
        }
        if (valueMotor > 100) {
          valueMotor = 100; //
        }
        setMotorSpeed(valueMotor);
      }

      // listen and motion mode uses incoming motion data and own motion data to calculate new valueMotor
      if (motorMode == listenAndMotionMode) {
        int d = motionVector - incomingFusionedData;
        Serial.print("incoming d:");Serial.println(incomingFusionedData);
        if (abs(d) > 1000) {
          valueMotor = valueMotor + 5;
        }
        if (abs(d) < 1000) {
          valueMotor = valueMotor - 5;
        }
        if (valueMotor < 50) {
          valueMotor = 50; //values must be above 500 otherwise the motor is off
        }
        if (valueMotor > 100) {
          valueMotor = 100; //
        }
        setMotorSpeed(valueMotor);
      }
    }
  }
  generateWebpage();
}


// connect to mqtt server
void reconnect() {
  //while (!mqttclient.connected()) {
  if (!mqttclient.connected()) {
    Serial.print("Attempting MQTT connection...");
    // Create a random client ID
    String clientId = id; // "ESP8266Client-";
    //clientId += String(random(0xffff), HEX);
    // Attempt to connect
    if (mqttclient.connect(clientId.c_str(), mqtt_user, mqtt_password)) {
      Serial.println("connected");
      mqttclient.subscribe("BIcomm");
      
    } else {
      Serial.print("failed, rc=");
      Serial.print(mqttclient.state());
      Serial.println(" try again in 5 seconds");
      delay(5000);
    }
  }
}



void setup() {
  Serial.begin(115200);
  delay(10);

  Wire.begin();
  // connect MPU9265 via i2c bus
  // NodeMCU D1 = GPIO5 connected to MCU9265 SCL
  // NodeMCU D2 = GPIO4 connected to MCU9265 SDA
  //Wire.pins(5,4);
  /*
    Serial.println("module=");
    if (module == "NodeMCU") {Wire.pins(5,4); Serial.println("NodeMCU");}
    if (module == "wemosmini") {Wire.pins(2,0); Serial.println("wemosmini");}
  */
  // init motor shield



  

  WiFiManager wifiManager;
  //wifiManager.resetSettings();
  wifiManager.autoConnect();

  /* Connect to WiFi network
  Serial.println();
  Serial.print("Connecting to ");
  Serial.println(ssid);

  WiFi.mode(WIFI_AP_STA);
  WiFi.begin(ssid, password);

  while (WiFi.status() != WL_CONNECTED) {
    delay(500);
    Serial.print(".");
  }
  Serial.println("");
  Serial.println("WiFi connected");
*/
  Serial.println("V30");
  Serial.print("id="); Serial.println(id);
  // Start the server
  //server.begin();
  //Serial.println("Server started");

  // Print the IP address
  Serial.print("Use this URL to connect: ");
  Serial.print("http://");
  Serial.print(WiFi.localIP());
  Serial.println("/");

  // init mqtt client
  mqttclient.setServer(mqtt_server, mqtt_port);
  mqttclient.setCallback(mqttCallback);

  // OTA service
  MDNS.begin(ota_host);
  httpUpdater.setup(&httpServer);
  httpServer.begin();
  MDNS.addService("http", "tcp", 80);
  Serial.printf("HTTPUpdateServer ready! Open http://%s.local/update in your browser\n", ota_host);

  if (MDNS.begin("esp8266")) {
    Serial.println("MDNS responder started");
  }
  //init webserver callback
  httpServer.on("/", generateWebpage);

  // LED on button pressed
  httpServer.on("/LED=ON", []() {
    requesting = true;
    digitalWrite(ledPin, HIGH);
    valueLED = HIGH;
    root["LEDstatus"] = valueLED;
    generateWebpage();
    digitalWrite(BUILTIN_LED, LOW);
  });
  // LED off button pressed
  httpServer.on("/LED=OFF", []() {
    requesting = true;
    digitalWrite(ledPin, LOW);
    valueLED = LOW;
    root["LEDstatus"] = valueLED;
    generateWebpage();
    digitalWrite(BUILTIN_LED, HIGH);
  });
  // set motor to maximum speed button pressed
  httpServer.on("/MOTOR=MAX", []() {
    requesting = true;
    setMotorSpeed(100);
    //analogWrite(motorPin, 1023);
    valueMotor = 100;
    motorMode = maxMode;
    root["motor1mode"] = motorMode;
    root["motor1speed"] = valueMotor;
    generateWebpage();
  });
  // set motor off button pressed
  httpServer.on("/MOTOR=OFF", []() {
    requesting = true;
    setMotorSpeed(0);
    //analogWrite(motorPin, 0);
    valueMotor = 0;
    motorMode = offMode;
    root["motor1mode"] = motorMode;
    root["motor1speed"] = valueMotor;
    generateWebpage();
  });
  // motor amplitude controlled like a sinus curve
  httpServer.on("/MOTOR=SINUS", []() {
    requesting = true;
    motorMode = sinusMode;
    root["motor1mode"] = motorMode;
    root["motor1speed"] = valueMotor;
    generateWebpage();
  });
  // motor speed is adjusted by movements (classical "body interaction" interaction pattern)
  httpServer.on("/MOTOR=MOTION", []() {
    requesting = true;
    motorMode = motionMode;
    valueMotor = 50;
    root["motor1mode"] = motorMode;
    root["motor1speed"] = valueMotor;
    generateWebpage();
  });
  // motor speed is adjusted by movements (classical "body interaction" interaction pattern)
  httpServer.on("/MOTOR=LISTEN", []() {
    requesting = true;
    motorMode = listenMode;
    valueMotor = 50;
    root["motor1mode"] = motorMode;
    root["motor1speed"] = valueMotor;
    generateWebpage();
  });
  // motor speed is adjusted by movements (classical "body interaction" interaction pattern)
  httpServer.on("/MOTOR=LISTENMOTION", []() {
    requesting = true;
    motorMode = listenAndMotionMode;
    valueMotor = 50;
    root["motor1mode"] = motorMode;
    root["motor1speed"] = valueMotor;
    generateWebpage();
  });
  httpServer.on("/SOFTRESET", []() {
    ESP.reset();
    generateWebpage();
  });
  // MPU9250 gyro, accelerometer
  // Configure gyroscope range
  I2CwriteByte(MPU9250_ADDRESS, 27, GYRO_FULL_SCALE_2000_DPS);
  // Configure accelerometers range
  I2CwriteByte(MPU9250_ADDRESS, 28, ACC_FULL_SCALE_16_G);
  // Set by pass mode for the magnetometers
  //I2CwriteByte(MPU9250_ADDRESS,0x37,0x02);

  // Request first magnetometer single measurement
  //I2CwriteByte(MAG_ADDRESS,0x0A,0x01);

  if (module == "NodeMCU") {
    // init LED pin
    pinMode(ledPin, OUTPUT);
    digitalWrite(ledPin, LOW);
    // init motor pin
    pinMode(motorPin, OUTPUT);
    setMotorSpeed(0);
    //analogWrite(motorPin, 0);
  }
  if (module == "wemosmini") {
    pinMode(BUILTIN_LED, OUTPUT);
  }

  Wire.beginTransmission(Motor_Shield_Address);
  uint32_t freq = 1000;
  Wire.write(((byte)(freq >> 16)) & (byte)0x0f);
  Wire.write((byte)(freq >> 16));
  Wire.write((byte)(freq >> 8));
  Wire.write((byte)freq);
  Wire.endTransmission();     // stop transmitting
  delay(100);
}

void loop() {
  // OTA service
  httpServer.handleClient();

  if (!mqttclient.connected()) {
    reconnect();
  }
  mqttclient.loop();

  // Read accelerometer and gyroscope
  uint8_t Buf[14];
  I2Cread(MPU9250_ADDRESS, 0x3B, 14, Buf);

  // Create 16 bits values from 8 bits data

  // Accelerometer
  ax = -(Buf[0] << 8 | Buf[1]);
  ay = -(Buf[2] << 8 | Buf[3]);
  az = Buf[4] << 8 | Buf[5];

  // Gyroscope
  gx = -(Buf[8] << 8 | Buf[9]);
  gy = -(Buf[10] << 8 | Buf[11]);
  gz = Buf[12] << 8 | Buf[13];

  int tmp = sqrt(pow(ax - ax1, 2) + pow(ay - ay1, 2) + pow(az - az1, 2)); //calculate motion vector

  if (motionVector < tmp) motionVector = tmp; // save highest value between MQTT message intervals

  // save values
  ax1 = ax;
  ay1 = ay;
  az1 = az;
  gx1 = gx;
  gy1 = gy;
  gz1 = gz;

  // when in "motionMode" the vibration motor is controlled by motion
  if (motorMode == motionMode) {
    // adjust vibration motor speed
    // if motion vector > 5000 raise speed by 25
    // otherwise lover speed by 10
    // adjust these constants to your needs

    if (motionVector <= 2000) {
      valueMotor = valueMotor - 2;
    }
    if (motionVector > 2000 && motionVector < 10000) {
      valueMotor = valueMotor + 5;
    }
    if (motionVector >= 10000) {
      valueMotor = valueMotor + 10;
    }
    if (valueMotor < 50) {
      valueMotor = 50; //values must be above 500 otherwise the motor is off
    }
    if (valueMotor > 100) {
      valueMotor = 100; // values higher than 1023 are not supported
    }
    setMotorSpeed(valueMotor);
    //analogWrite(motorPin, valueMotor); // set motor speed

    Serial.print("motionVector: ");
    Serial.print(motionVector);
    Serial.print(", valueMotor: ");
    Serial.println(valueMotor);
  }

  delay(200); // delay between acceleration measurements

  // change vibration motor speed according to a sinus curve
  if (motorMode == sinusMode) {
    sinusValue = sinusValue + .1;
    //delay(20);
    int sinTmp = ((sin(sinusValue) + 1) * .5 * (100 - 50)) + 50;
    setMotorSpeed(sinTmp);
    //analogWrite(motorPin, sinTmp);
    valueMotor = sinTmp;
    Serial.print(", sin: ");
    Serial.println(sinTmp);
  }

  //StaticJsonBuffer<500> jsonBuffer;
  //JsonObject& root = jsonBuffer.createObject();
  root["deviceName"] = device_name;
  root["chipId"] = chip_id;
  root["messageType"] = "sensor"; // execute, message, sensor
  // execute - send command to other device
  // root["targetDeviceID"] = "xxxxx"; //for execute and message message types
  // root["actuator"] = "motor1";
  // root["actuatorValue"]="222";
  // root["actuatorMode"] = "sinus";
  // root["command"] = none;
  // root["commandParameter1"] ="";

  // message - send message to targetDeviceID
  // root["message"] = "hello world";

  //sensor - for chip-ing sensor data
  root["sensor"] = "mps9250";
  // root["time"] = none;
  root["fusionedData"] = motionVector;
  root["messageNumber"] = 0;
  // example for raw data
  // JsonArray& rawdata = root.createNestedArray("rawdata"); // x,y,z,roll, pitch better??
  // rawdata.add(0, 0);  // ax
  // rawdata.add(0, 0);  // ay
  // rawdata.add(0, 0);  // az
  // rawdata.add(0, 0);  // gx
  // rawdata.add(0, 0);  // gx
  // rawdata.add(0, 0);  // gx
  // rawdata.add(0, 0);  // mx
  // rawdata.add(0, 0);  // mx
  // rawdata.add(0, 0);  //mx
  root["LEDstatus"] = valueLED;
  root["motor1mode"] = motorMode;
  root["motor1speed"] = valueMotor;



  now2 = millis();
  if (now2 - lastI2Csend > 1000) { // publish motor speed as mqtt message every 5 seconds
    setMotorSpeed(valueMotor); // try to avoid crash of i2c bus
    lastI2Csend = now2;
  }

  if (motorMode == maxMode || motorMode == sinusMode || motorMode == motionMode || motorMode == constantMode) {
    now = millis();
    if (now - lastMsg > 1000) { //
      if (!mqttclient.connected()) {
        reconnect();
      }
      lastMsg = now;
      ++value;
      root["messageNumber"] = value;
      // publish data as MQTT message in JSON format
      root.printTo(outMsg, sizeof(outMsg));
      snprintf (msg, 1000, "%s", outMsg);
      mqttclient.publish("BIcomm", msg);
      //Serial.print("Publish message every 10 sec: ");
      Serial.println(msg);
      //motionVector = 0; //reset motion data
    }
  }

  //generateWebpage(espclient);

  // send outMessage to Mqtt server after a request from web portal
  if (requesting) {
    requesting = false;
    if (!mqttclient.connected()) {
      reconnect();
    }
    ++value;

    root["messageNumber"] = value;
    //root["motor1mode"] = motorMode;
    //root["motor1speed"] = valueMotor;

    // publish data as MQTT message in JSON format
    root.printTo(outMsg, sizeof(outMsg));
    snprintf (msg, 1000, "%s", outMsg);
    mqttclient.publish("BIcomm", msg);

    Serial.print("Publish message: ");
    Serial.println(msg);
  }
}
