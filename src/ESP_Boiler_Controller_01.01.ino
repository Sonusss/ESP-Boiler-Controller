

// Board ESP-01
// OTA enabled
// LWT implemented
// Added MQTT reboot command
// Based on LC_Tech 4 relays board

const char* VERS = "ESP_Boiler_Controller_01.01";
const bool debug = false;
// if true, relay control through serial is inhibited and serial port is used as debug interface

//#include <Arduino.h>
#include <PubSubClient.h>
#include <ArduinoJson.h>
#include <ESP8266WiFi.h>
#include "uptime.h"
#include <ArduinoOTA.h>
#include <OneWire.h> 
#include <DallasTemperature.h>
#include <EEPROM.h>
#include <ezButton.h>
#include <string.h>


/***************************
 * Begin Settings
 **************************/


// WIFI
const char* WIFI_SSID = "YourWIFI";
const char* WIFI_PWD = "Yourpassword";

// MQTT
const char* MQTT_SERVER = "10.10.0.221";
const int16_t MQTT_PORT = 1883;
const char* MQTT_USER = "mqttuser";
const char* MQTT_PASSWD = "mqttpassword";
#define MQTT_MAX_TRANSFER_SIZE 1024


const char* MQTT_SUBSCRIBE_TOPIC = "oil_burner/cmd";
const char* MQTT_PUBLISH_TOPIC = "oil_burner/state";
byte MQTT_WILL_QOS = 0;
const char* MQTT_WILL_TOPIC = "oil_burner/lwt";
const char* MQTT_WILL_MESSAGE = "Offline";
boolean MQTT_WILL_RETAIN = false;

// OTA
const char* OTA_PWD = "OTApassword";


const char* Sensor_obj = "sensors";
const char* Status_obj = "status";
const char* Event_obj = "event";
const char* Settings_obj = "settings";

int lp_counter = 0;

// SENSORS

int outlet_probe_idx = 0;  // furnace temp or water outlet temp
int return_probe_idx = 0;  // furnace return water temp
int boiler_probe_idx = 0;  // hot water tank temp
int inside_probe_idx = 0;  // inside temp sensor
int outside_probe_idx = 0; //outside temp sensor
float heating_temp = 0;
float return_temp = 0;
float boiler_temp = 0;
float inside_temp = 0;
float outside_temp = 0;

// PARAMETERS

byte heating = 0;
byte heating_preset_temp = 70;
byte heating_high_limit = 90;
byte heating_low_limit = 50;
byte heating_hysteresis = 2;  //10th of degree
byte heating_temp_factor = 100;
byte heating_temp_bias = 10;
byte adaptive_heating = 0;
float heating_target_temp = 20.5;
byte burner_min_time = 3;


byte boiler = 0;
boolean boiler_active = false;
byte boiler_priority = 0;
byte boiler_preset_temp = 60;
byte boiler_heating_temp = 75;
byte boiler_high_limit = 90;
byte boiler_low_limit = 55;
byte boiler_temp_factor = 100;
byte boiler_temp_bias = 5;

// RELAYS
bool  relay1 = false;
bool  c_burner = false;
bool  burner = false;
bool  h_pump = false;
bool  b_pump = false;
byte  burner_timer = 0; 


// INPUTS
ezButton Dinput1(4);  // create ezButton object that attach to pin 4;
bool flame = false;

// SETUP

char mqtt_mesg[100];
byte MsgRL1On[] = {0xA0, 0x01, 0x01, 0xA2}; // ouvre le relais #1
byte MsgRL1Off[] = {0xA0, 0x01, 0x00, 0xA1}; // ferme le relais #1
byte MsgRL2On[] = {0xA0, 0x02, 0x01, 0xA3}; // ouvre le relais #2
byte MsgRL2Off[] = {0xA0, 0x02, 0x00, 0xA2}; // ferme le relais #2
byte MsgRL3On[] = {0xA0, 0x03, 0x01, 0xA4}; // ouvre le relais #3
byte MsgRL3Off[] = {0xA0, 0x03, 0x00, 0xA3}; // ferme le relais #3
byte MsgRL4On[] = {0xA0, 0x04, 0x01, 0xA5}; // ouvre le relais #4
byte MsgRL4Off[] = {0xA0, 0x04, 0x00, 0xA4}; // ferme le relais #4


String comnd = "";


// Setup a oneWire instance to communicate with any OneWire devices  
// (not just Maxim/Dallas temperature ICs) 
#define ONE_WIRE_BUS 2 
OneWire oneWire(ONE_WIRE_BUS); 
/********************************************************************/
// Pass our oneWire reference to Dallas Temperature. 
DallasTemperature sensors(&oneWire);

DeviceAddress heaterThermometer, boilerThermometer;

/***************************
 * End Settings
 **************************/



time_t now;

WiFiClient espClient;
PubSubClient client(espClient);
unsigned long lastMsg = 0;
#define MSG_BUFFER_SIZE  (100)
char msg[MSG_BUFFER_SIZE];
int value = 0;

os_timer_t myTimer;
int tickOccured = 0;

// start of timerCallback
void timerCallback(void *pArg) {
      tickOccured ++;
} // End of timerCallback


String get_uptime()
{
  uptime::calculateUptime();
  
String T_hours;
String T_Minutes;
String T_Seconds;

  //Serial.print("\n");
  if (uptime::getHours()<10){
     T_hours = "0" + String(uptime::getHours());
  }else{
     T_hours = String(uptime::getHours());
  }

  if (uptime::getMinutes()<10){
     T_Minutes = "0" + String(uptime::getMinutes());
  }else{
     T_Minutes = String(uptime::getMinutes());
  }
    
  if (uptime::getSeconds()<10){
     T_Seconds = "0" + String(uptime::getSeconds());
  }else{
     T_Seconds = String(uptime::getSeconds());
  }

 //restart every day
//  if (uptime::getDays()>1){
    //resetFunc(); //call reset 
//    ESP.restart();
//  }
    
    
  String G_uptime = String(uptime::getDays()) + "T" + T_hours + ":" + T_Minutes + ":" + T_Seconds;
  return G_uptime;
}

void callback_MQTT(char* topic, byte* payload, unsigned int length) {
  if (debug) {
    Serial.print("MQTT Receive <- [");
    Serial.print(topic);
    Serial.print("] ");
    for (int i = 0; i < length; i++) {
      Serial.print((char)payload[i]);
    }
    Serial.println();
  }
  String recv_payload = String(( char *) payload);
  StaticJsonDocument<1024> doc0;
  DeserializationError err = deserializeJson(doc0, payload);

  JsonVariant answer1 = doc0["relay1"];
  JsonVariant answer2 = doc0["burner"];
  JsonVariant answer3 = doc0["h_pump"];
  JsonVariant answer4 = doc0["b_pump"];
  JsonVariant Joutlet_probe_idx = doc0["outlet_probe_idx"];
  JsonVariant Jreturn_probe_idx = doc0["return_probe_idx"];
  JsonVariant Jboiler_probe_idx = doc0["boiler_probe_idx"];
  JsonVariant Jinside_probe_idx = doc0["inside_probe_idx"];
  JsonVariant Joutside_probe_idx = doc0["outside_probe_idx"];
  JsonVariant Jheating = doc0["heating"];
  JsonVariant Jheating_preset_temp = doc0["heating_preset_temp"];
  JsonVariant Jheating_high_limit = doc0["heating_high_limit"];
  JsonVariant Jheating_low_limit = doc0["heating_low_limit"];
  JsonVariant Jheating_hysteresis = doc0["heating_hysteresis"];
  JsonVariant Jheating_temp_factor = doc0["heating_temp_factor"];
  JsonVariant Jheating_temp_bias = doc0["heating_temp_bias"];
  JsonVariant Jadaptive_heating = doc0["adaptive_heating"];
  JsonVariant Jheating_target_temp = doc0["heating_target_temp"];
  JsonVariant Jboiler = doc0["boiler"];
  JsonVariant Jboiler_priority = doc0["boiler_priority"];
  JsonVariant Jboiler_preset_temp = doc0["boiler_preset_temp"];
  JsonVariant Jboiler_heating_temp = doc0["boiler_heating_temp"];
  JsonVariant Jboiler_high_limit = doc0["boiler_high_limit"];
  JsonVariant Jboiler_low_limit = doc0["boiler_low_limit"];
  JsonVariant Jboiler_temp_factor = doc0["boiler_temp_factor"];
  JsonVariant Jboiler_temp_bias = doc0["boiler_temp_bias"];
  JsonVariant Jburner_min_time = doc0["burner_min_time"];
  JsonVariant Jsave = doc0["save"];
  JsonVariant Jwipe = doc0["wipe"];
  JsonVariant Jinit = doc0["init"];
  JsonVariant Jreboot = doc0["reboot"];
  JsonVariant Jsettings = doc0["settings"];
  JsonVariant Jprobe = doc0["probe"];
  JsonVariant Jheating_temp = doc0["heating_temp"];
  JsonVariant Jreturn_temp = doc0["return_temp"];  
  JsonVariant Jboiler_temp = doc0["boiler_temp"];

 
  if (!Joutlet_probe_idx.isNull()) { outlet_probe_idx = Joutlet_probe_idx; };
  if (!Jreturn_probe_idx.isNull()) { return_probe_idx = Jreturn_probe_idx; };
  if (!Jboiler_probe_idx.isNull()) { boiler_probe_idx = Jboiler_probe_idx; };
  if (!Jinside_probe_idx.isNull()) { inside_probe_idx = Jinside_probe_idx; };
  if (!Joutside_probe_idx.isNull()) { outside_probe_idx = Joutside_probe_idx; };
  if (!Jheating.isNull()) { 
       heating = Jheating; 
       //publish_MQTT_event("heating", String(heating));
        publish_MQTT_stat();         
       };
  if (!Jheating_preset_temp.isNull()) { heating_preset_temp = Jheating_preset_temp; };
  if (!Jheating_high_limit.isNull()) { heating_high_limit = Jheating_high_limit; };
  if (!Jheating_low_limit.isNull()) { heating_low_limit = Jheating_low_limit; };
  if (!Jheating_hysteresis.isNull()) { heating_hysteresis = Jheating_hysteresis; };
  if (!Jheating_temp_factor.isNull()) { heating_temp_factor = Jheating_temp_factor; };
  if (!Jheating_temp_bias.isNull()) { heating_temp_factor = Jheating_temp_bias; };
  if (!Jadaptive_heating.isNull()) { adaptive_heating = Jadaptive_heating; };
  if (!Jheating_target_temp.isNull()) { heating_target_temp = Jheating_target_temp; };
  if (!Jboiler.isNull()) { 
       boiler = Jboiler;
       //publish_MQTT_event("boiler", String(boiler));       
        publish_MQTT_stat();         
       };
  if (!Jboiler_priority.isNull()) { boiler_priority = Jboiler_priority; };
  if (!Jboiler_preset_temp.isNull()) { boiler_preset_temp = Jboiler_preset_temp; };
  if (!Jboiler_heating_temp.isNull()) { boiler_heating_temp = Jboiler_heating_temp; };
  if (!Jboiler_high_limit.isNull()) { boiler_high_limit = Jboiler_high_limit; };
  if (!Jboiler_low_limit.isNull()) { boiler_low_limit = Jboiler_low_limit; };
  if (!Jboiler_temp_factor.isNull()) { boiler_temp_factor = Jboiler_temp_factor; };
  if (!Jboiler_temp_bias.isNull()) { boiler_temp_bias = Jboiler_temp_bias; };
  if (!Jburner_min_time.isNull()) { burner_min_time = Jburner_min_time; };
//  if (!Jsave.isNull()) { save_setup(); };  //some commands takes too long and conflict with loop()
  if (!Jsave.isNull()) { comnd = "save_setup"; };
// if (!Jwipe.isNull()) { wipe_setup(); };
  if (!Jwipe.isNull()) { comnd = "wipe_setup"; };
//  if (!Jinit.isNull()) { init_setup(); };
  if (!Jinit.isNull()) { comnd = "init_setup"; };
//  if (!Jreboot.isNull()) { ESP.restart(); };
  if (!Jreboot.isNull()) { comnd = "ESP.restart"; };
//  if (!Jsettings.isNull()) { publish_MQTT_settings(); };
  if (!Jsettings.isNull()) { comnd = "publish_MQTT_settings"; };
//  if (!Jprobe.isNull()) { publish_MQTT_sensors(); };
  if (!Jprobe.isNull()) { comnd = "publish_MQTT_sensors"; };

  if (!Jheating_temp.isNull()) { heating_temp = Jheating_temp; };
  if (!Jreturn_temp.isNull()) { return_temp = Jreturn_temp; };
  if (!Jboiler_temp.isNull()) { boiler_temp = Jboiler_temp; };
    
// PARAMETERS
//  Serial.print("-");  
//  Serial.print(answer.as<float>());    
//  Serial.println("-"); 
         if (answer1 == 0){
            relay(1,"off");
         }
         if (answer1 == 1){
            relay(1,"on");
         }
         if (answer2 == 0){
            relay(2,"off");
         }
         if (answer2 == 1){
            relay(2,"on");
         }
         if (answer3 == 0){
            relay(3,"off");
         }
         if (answer3 == 1){
            relay(3,"on");
         }
         if (answer4 == 0){
            relay(4,"off");
         }
         if (answer4 == 1){
            relay(4,"on");
         }

  //Parse succeeded?
  if (err && debug) {
    Serial.print(F("deserializeJson() returned "));
    Serial.println(err.c_str());
    Serial.println();
  }

}

void reconnect_MQTT() {
  // Loop until we're reconnected
  while (!client.connected()) {
    if (debug){
      Serial.print("Attempting MQTT connection...");
    }
    // Attempt to connect
    if (client.connect("Oil_Burner",MQTT_USER, MQTT_PASSWD, MQTT_WILL_TOPIC, MQTT_WILL_QOS, MQTT_WILL_RETAIN, MQTT_WILL_MESSAGE)) {
    if (debug){
        Serial.println("connected");
    }
      // Once connected, publish an announcement...
      if (client.subscribe(MQTT_SUBSCRIBE_TOPIC)) {
        if (debug){
          Serial.println(" - Subscribed");
        }
      }         
    } else {
    if (debug){      
      Serial.print("failed, rc=");
      Serial.print(client.state());
    }
   }
  }
}



void publish_MQTT_stat() {
  StaticJsonDocument<400> doc1;
  JsonObject status0 = doc1.createNestedObject(Status_obj);
  status0["heating"] = String(heating);
  status0["boiler"] = String(boiler);
  status0["relay1"] = String(relay1);
  status0["burner"] = String(burner);
  status0["h_pump"] = String(h_pump);
  status0["b_pump"] = String(b_pump);
  status0["flame"] = String(flame);
  status0["heating_temp"] = String(heating_temp,1);
  status0["return_temp"] = String(return_temp,1);
  status0["boiler_temp"] = String(boiler_temp,1); 
  status0["inside_temp"] = String(inside_temp,1);
  status0["outside_temp"] = String(outside_temp,1);         
  status0["Version"] = VERS;
  status0["Uptime"] = get_uptime();
  status0["IPAddress"] = WiFi.localIP().toString();
  status0["MACAddress"] = WiFi.macAddress();
  status0["SSID"] = WiFi.SSID();
  status0["RSSI"] = WiFi.RSSI();
   char mqtt_mesg[400];
  serializeJson(doc1, mqtt_mesg);
    if (debug){
      Serial.print("MQTT Publish -> ["); 
      Serial.print(MQTT_PUBLISH_TOPIC);
      Serial.print("] ");       
      Serial.println(mqtt_mesg);
    }
    if (client.connected()) {
      client.publish(MQTT_PUBLISH_TOPIC, mqtt_mesg );
      // ... and resubscribe
      //client.subscribe(MQTT_SUBSCRIBE_TOPIC);
    } else {
    if (debug){      
      Serial.print("failed, rc=");
      Serial.print(client.state());
    }
  }
}



void publish_MQTT_lwt() {
    if (debug){
      Serial.print("MQTT Publish -> ["); 
      Serial.print(MQTT_WILL_TOPIC);
      Serial.print("] ");       
      Serial.println("Online");
    }
    if (client.connected()) {
      client.publish(MQTT_WILL_TOPIC, "Online" );
      // ... and resubscribe
      //client.subscribe(MQTT_SUBSCRIBE_TOPIC);
    } else {
    if (debug){
      Serial.print("failed, rc=");
      Serial.print(client.state());
    }
  }
}


void publish_MQTT_event(String event, String state) {
  StaticJsonDocument<50> doc2;
  JsonObject event0= doc2.createNestedObject(Event_obj);
  event0[event] = state;
  char mqtt_mesg[50];
  serializeJson(doc2, mqtt_mesg);
    if (debug){  
      Serial.print("MQTT Publish -> ["); 
      Serial.print(MQTT_PUBLISH_TOPIC);
      Serial.print("] ");       
      Serial.println(mqtt_mesg);
      Serial.println(client.state());
    }      
    if (client.connected()) {
      client.publish(MQTT_PUBLISH_TOPIC, mqtt_mesg );
      // ... and resubscribe
      //client.subscribe(MQTT_SUBSCRIBE_TOPIC);
    } else {
    if (debug){
      Serial.print("failed, rc=");
      Serial.print(client.state());
    }
  }
}


void publish_MQTT_sensors() {
  String temp0=String(sensors.getTempCByIndex(0),1);
  String temp1=String(sensors.getTempCByIndex(1),1);
  String temp2=String(sensors.getTempCByIndex(2),1);
  String temp3=String(sensors.getTempCByIndex(3),1);
  String temp4=String(sensors.getTempCByIndex(4),1);
  StaticJsonDocument<100> doc3;
  JsonObject sensor0 = doc3.createNestedObject(Sensor_obj);
  sensor0["T_probe0"] = temp0;
  sensor0["T_probe1"] = temp1;
  sensor0["T_probe2"] = temp2;
  sensor0["T_probe3"] = temp3;
  sensor0["T_probe4"] = temp4;
  
  char mqtt_mesg[100];
  serializeJson(doc3, mqtt_mesg);
    if (debug){
      Serial.print("MQTT Publish -> ["); 
      Serial.print(MQTT_PUBLISH_TOPIC);
      Serial.print("] ");       
      Serial.println(mqtt_mesg);
      Serial.println(client.state());
    }
    if (client.connected()) {
      client.publish(MQTT_PUBLISH_TOPIC, mqtt_mesg );
      // ... and resubscribe
      //client.subscribe(MQTT_SUBSCRIBE_TOPIC);
    } else {
    if (debug){
      Serial.print("failed, rc=");
      Serial.print(client.state());
    }
  }
}


void publish_MQTT_settings() {
  StaticJsonDocument<1024> doc4;
  JsonObject settings0 = doc4.createNestedObject(Settings_obj);
  settings0["outlet_probe_idx"] = outlet_probe_idx;
  settings0["return_probe_idx"] = return_probe_idx;
  settings0["boiler_probe_idx"] = boiler_probe_idx;
  settings0["inside_probe_idx"] = inside_probe_idx;
  settings0["outside_probe_idx"] = outside_probe_idx;
  settings0["heating"] = heating;
  settings0["heating_preset_temp"] = heating_preset_temp;
  settings0["heating_high_limit"] = heating_high_limit;
  settings0["heating_low_limit"] = heating_low_limit;
  settings0["heating_hysteresis"] = heating_hysteresis;
  settings0["heating_temp_factor"] = heating_temp_factor;
  settings0["adaptive_heating"] = adaptive_heating;
  settings0["heating_target_temp"] = heating_target_temp;
  settings0["boiler"] = boiler;
  settings0["boiler_priority"] = boiler_priority;
  settings0["boiler_preset_temp"] = boiler_preset_temp;
  settings0["boiler_heating_temp"] = boiler_heating_temp;
  settings0["boiler_high_limit"] = boiler_high_limit;
  settings0["boiler_low_limit"] = boiler_low_limit;
  settings0["boiler_temp_factor"] = boiler_temp_factor;
  settings0["boiler_temp_bias"] = boiler_temp_bias;
  settings0["burner_min_time"] = burner_min_time;  
  char mqtt_mesg[1024];
  serializeJson(doc4, mqtt_mesg);
    if (debug){
      Serial.print("MQTT Publish -> ["); 
      Serial.print(MQTT_PUBLISH_TOPIC);
      Serial.print("] ");       
      Serial.println(mqtt_mesg);
    }
    if (client.connected()) {
      client.publish(MQTT_PUBLISH_TOPIC, mqtt_mesg );
       //... and resubscribe
      client.subscribe(MQTT_SUBSCRIBE_TOPIC);
    } else {
    if (debug){
      Serial.print("failed, rc=");
      Serial.print(client.state());
    }
  }
}


void relay(int relay, char state[3]) {

  if (relay == 1) {
    if (state == "on") {
      if (!debug){
        Serial.write(MsgRL1On , sizeof(MsgRL1On));
      }
        relay1 = true;
        publish_MQTT_stat();          
    }
    if (state == "off") {
      if (!debug){
        Serial.write(MsgRL1Off , sizeof(MsgRL1Off));
      }
        relay1 = false;
        publish_MQTT_stat();          
    }
  }
  if (relay == 2) {
    if (state == "on") {
      if (!debug){
        Serial.write(MsgRL2On , sizeof(MsgRL2On));
      }
        burner = true;
        publish_MQTT_stat();         
    }
    if (state == "off") {
      if (!debug){
        Serial.write(MsgRL2Off, sizeof(MsgRL2Off));
      }
        burner = false;
        publish_MQTT_stat();  
    }
  }
  if (relay == 3) {
    if (state == "on") {
      if (!debug){
        Serial.write(MsgRL3On , sizeof(MsgRL3On));
      }
        h_pump = true;
        publish_MQTT_stat();        
    }
    if (state == "off") {
      if (!debug){
        Serial.write(MsgRL3Off , sizeof(MsgRL3Off));
      }       
        h_pump = false;
        publish_MQTT_stat(); 
    }
  }
  if (relay == 4) {
    if (state == "on") {
      if (!debug){
        Serial.write(MsgRL4On , sizeof(MsgRL4On));
      }
        b_pump = true;
        publish_MQTT_stat();  
    }
    if (state == "off") {
      if (!debug){
        Serial.write(MsgRL4Off , sizeof(MsgRL4Off));
      }
        b_pump = false;
        publish_MQTT_stat();  
    }
  }  
}

void get_temp(){
//  sensors.requestTemperatures(); // Send the command to get temperature readings 
//  delay(1000);
  heating_temp = float(sensors.getTempCByIndex(outlet_probe_idx));
  return_temp = float(sensors.getTempCByIndex(return_probe_idx));
  boiler_temp = float(sensors.getTempCByIndex(boiler_probe_idx));
  inside_temp = float(sensors.getTempCByIndex(inside_probe_idx));
  outside_temp = float(sensors.getTempCByIndex(outside_probe_idx));
}

void init_setup(){
  outlet_probe_idx = 0;
  return_probe_idx = 1;
  boiler_probe_idx = 2;
  inside_probe_idx = 3;
  outside_probe_idx = 4;
  heating = 0;                //Heating mode on/off  (0/1)
  heating_preset_temp = 70;   //Normal heating water temp 
  heating_high_limit = 90;    //Safety upper limit
  heating_low_limit = 50;     //Temperature when firing the flame
  heating_hysteresis = 2;     //not used (future use)
  heating_temp_factor = 100;  //not used (future use)
  heating_temp_bias = 10;     //not used (future use)
  adaptive_heating = 0;       //not used (future use)
  heating_target_temp = 20.5; //not used (future use)
  burner_min_time = 1;        //minimal flame time to avoid scattering
  boiler = 0;                 //Hot water generator on/off (0/1)
  boiler_priority = 1;        //Priority mode over heating
  boiler_preset_temp = 60;    //Target temp when generation hot water
  boiler_heating_temp = 75;   //Target temp of circulating water (minimum 5 degrees over boiler_preset_temp)
  boiler_high_limit = 90;     //Safety upper limit
  boiler_low_limit = 40;      //hot water generation is triggered when going below this temperature
  boiler_temp_factor = 100;   //not used
  boiler_temp_bias = 5;       //Circulating water should be warmer of x degrees in order to turn circulating pump on (efficiency security) 
}


void save_setup() {
  int addr = 0;
  EEPROM.write(addr, outlet_probe_idx);
  addr++;
  EEPROM.write(addr, return_probe_idx);
  addr++;
  EEPROM.write(addr, boiler_probe_idx);
  addr++;
  EEPROM.write(addr, inside_probe_idx);
  addr++;
  EEPROM.write(addr, outside_probe_idx);
  addr++;
  EEPROM.write(addr, heating);
  addr++;
  EEPROM.write(addr, heating_preset_temp);
  addr++;
  EEPROM.write(addr, heating_high_limit);
  addr++;
  EEPROM.write(addr, heating_low_limit);
  addr++;
  EEPROM.write(addr, heating_hysteresis);
  addr++;
  EEPROM.write(addr, heating_temp_factor);
  addr++;
  EEPROM.write(addr, adaptive_heating);
  addr++;
  EEPROM.put(addr, heating_target_temp);
  addr++;
  addr++;  
  addr++;
  addr++;  
  EEPROM.write(addr, boiler);
  addr++;
  EEPROM.write(addr, boiler_priority);
  addr++;
  EEPROM.write(addr, boiler_preset_temp);
  addr++;
  EEPROM.write(addr, boiler_heating_temp);
  addr++;
  EEPROM.write(addr, boiler_high_limit);
  addr++;
  EEPROM.write(addr, boiler_low_limit);
  addr++;
  EEPROM.write(addr, boiler_temp_factor);
  addr++;
  EEPROM.write(addr, boiler_temp_bias);
  addr++;
  EEPROM.write(addr, burner_min_time);
  EEPROM.commit();    //Store data to EEPROM
}

void read_setup(){
  int addr = 0;
  outlet_probe_idx=EEPROM.read(addr);
  addr++;
  return_probe_idx=EEPROM.read(addr);
  addr++;
  boiler_probe_idx=EEPROM.read(addr);
  addr++;
  inside_probe_idx=EEPROM.read(addr);
  addr++;
  outside_probe_idx=EEPROM.read(addr);
  addr++;
  heating=EEPROM.read(addr);
  addr++;
  heating_preset_temp=EEPROM.read(addr);
  addr++;
  heating_high_limit=EEPROM.read(addr);
  addr++;
  heating_low_limit=EEPROM.read(addr);
  addr++;
  heating_hysteresis=EEPROM.read(addr);
  addr++;
  heating_temp_factor=EEPROM.read(addr);
  addr++;
  adaptive_heating=EEPROM.read(addr);
  addr++;
  float target_temp;
  EEPROM.get(addr,target_temp);
  heating_target_temp = target_temp;
  addr++;
  addr++;
  addr++;
  addr++;
  boiler=EEPROM.read(addr);
  addr++;
  boiler_priority=EEPROM.read(addr);
  addr++;
  boiler_preset_temp=EEPROM.read(addr);
  addr++;
  boiler_heating_temp=EEPROM.read(addr);
  addr++;
  boiler_high_limit=EEPROM.read(addr);
  addr++;
  boiler_low_limit=EEPROM.read(addr);
  addr++;
  boiler_temp_factor=EEPROM.read(addr);
  addr++;
  boiler_temp_bias=EEPROM.read(addr);
  addr++;
  burner_min_time=EEPROM.read(addr);
}

void wipe_setup(){
  int addr = 0;
  while (addr < 512) {
    EEPROM.write(addr, 0);
    addr++;
  }
  EEPROM.commit();    //Store data to EEPROM
}

//void writeIntIntoEEPROM(int address, int number)
//{ 
//  EEPROM.write(address, number >> 8);
//  EEPROM.write(address + 1, number & 0xFF);
//}
//
//
//int readIntFromEEPROM(int address)
//{
//  byte byte1 = EEPROM.read(address);
//  byte byte2 = EEPROM.read(address + 1);
//
//  return (byte1 << 8) + byte2;
//}


void heating_rules(){
  if (heating == 1) {
    if ((boiler_priority == 1 && boiler_active == false) || (boiler_priority == 0)){
      if (heating_temp < heating_low_limit + 3.0 && burner == false) {
        //relay(2, "on");
        c_burner = true;
      }
      if (heating_temp >= heating_preset_temp && boiler_active == false && burner == true) {
        //relay(2, "off");
        c_burner = false;
      } 
      if (heating_temp >= heating_low_limit  && h_pump == false) {
        relay(3, "on");
      }
      if (heating_temp < heating_low_limit  && h_pump == true) {
        relay(3, "off");
      }
    }else{
      if (h_pump == true){
        relay(3, "off");
      } 
    }
  }
  if (heating == 0) {
    if (h_pump == true){
      relay(3, "off");
    }
//    if (burner == true ){
//      relay(2, "off");
//    }
  }
}


void boiler_rules(){
  if (boiler == 1) {
    if (boiler_temp < boiler_low_limit) {
      boiler_active = true;
    }
    if (boiler_temp >= boiler_preset_temp) {
      boiler_active = false;
    } 
    if (heating_temp < boiler_heating_temp && boiler_active == true && burner == false) {
      //relay(2,"on");
      c_burner = true;
    }
    if (heating_temp >= boiler_heating_temp && boiler_active == true && burner == true) {
      //relay(2,"off");
      c_burner = false;
    }    
    if (boiler_temp < heating_temp && heating_temp > boiler_temp + float(boiler_temp_bias) && boiler_temp < boiler_preset_temp && boiler_active == true && b_pump == false) {
      relay(4, "on");
    }
  }
  if (boiler == 0) {
    if (b_pump == true){
      relay(4, "off");
      boiler_active = false;      
    }
  }
  if (boiler_temp >= boiler_preset_temp  && b_pump == true) {
      relay(4, "off");
      relay(2, "off");
      c_burner = false;
      burner_timer = 0;      
      boiler_active = false;
    }
  if (boiler_temp > heating_temp && b_pump == true) {
      relay(4, "off");
    }
}


void setup() {
  
  Serial.begin(115200);
  //Serial.begin(9600);

  if (!debug){
  
    Serial.write(MsgRL1Off , sizeof(MsgRL1Off));
    delay(10);
    Serial.write(MsgRL2Off , sizeof(MsgRL2Off));
    delay(10);
    Serial.write(MsgRL3Off , sizeof(MsgRL3Off));
    delay(10);
    Serial.write(MsgRL4Off , sizeof(MsgRL4Off));
  }

//  Serial.write(MsgRL1On , sizeof(MsgRL1On));
//  delay(10);
//  Serial.write(MsgRL2On , sizeof(MsgRL2On));
//  delay(10);
//  Serial.write(MsgRL3On , sizeof(MsgRL3On));
//  delay(10);
//  Serial.write(MsgRL4On , sizeof(MsgRL4On));
  
  client.setServer(MQTT_SERVER, MQTT_PORT);
  client.setCallback(callback_MQTT);


  WiFi.begin(WIFI_SSID, WIFI_PWD);

  int counter = 0;
  while (WiFi.status() != WL_CONNECTED) {
    delay(500);
      if (debug){
        Serial.print(".");
      }    

    counter++;
  }

  EEPROM.begin(512);  //Initialize EEPROM

     os_timer_setfn(&myTimer, timerCallback, NULL);
     os_timer_arm(&myTimer, 1000, true);
     tickOccured = 0;

  // Port defaults to 8266
  // ArduinoOTA.setPort(8266);

  // Hostname defaults to esp8266-[ChipID]
  ArduinoOTA.setHostname(VERS);

  // No authentication by default
  ArduinoOTA.setPassword(OTA_PWD);

//  ArduinoOTA.onStart([]() {
//    Serial.println("OTA Start");
//  });
//  ArduinoOTA.onEnd([]() {
//    Serial.println("\n OTA End");
//  });

  ArduinoOTA.onProgress([](unsigned int progress, unsigned int total) {
    if (debug){
      Serial.printf("Progress: %u%%\r", (progress / (total / 100)));
    }
  });
  ArduinoOTA.onError([](ota_error_t error) {
    if (debug){    
      Serial.printf("Error[%u]: ", error);
      if (error == OTA_AUTH_ERROR) Serial.println("Auth Failed");
      else if (error == OTA_BEGIN_ERROR) Serial.println("Begin Failed");
      else if (error == OTA_CONNECT_ERROR) Serial.println("Connect Failed");
      else if (error == OTA_RECEIVE_ERROR) Serial.println("Receive Failed");
      else if (error == OTA_END_ERROR) Serial.println("End Failed");
    }
  });
  ArduinoOTA.begin();
  if (debug){
    Serial.println("OTA Ready");
    Serial.print("OTA IP address: ");
    Serial.println(WiFi.localIP());
    Serial.println("MQTT buffer: " + String(client.getBufferSize())); 
    Serial.println("");
    Serial.print("Software version: ");
    Serial.println(VERS);
    Serial.println();
  }
   reconnect_MQTT();

   sensors.begin(); 
   read_setup();
   sensors.requestTemperatures(); // Send the command to get temperature readings 
   publish_MQTT_lwt(); 
   delay(1000);


//   get_temp();      

   publish_MQTT_sensors();
   delay(1000);
   publish_MQTT_settings();   
//   pinMode(InputPin1, INPUT_PULLUP);
   Dinput1.setDebounceTime(50); // set debounce time to 50 milliseconds
}

void loop() {
static int32_t lastime1 = millis(); 
static int32_t lastime2 = millis(); 
static int32_t lastime3 = millis(); 
delay(1);

  Dinput1.loop(); // MUST call the loop() function first

  if(Dinput1.isPressed()) {
        //publish_MQTT_event("flame", "1");
        publish_MQTT_stat();
        flame = true;
  }
  if(Dinput1.isReleased()) {
        //publish_MQTT_event("flame", "0");
        flame = false;
        publish_MQTT_stat();
  }


//    lp_counter0++;
//    if (lp_counter0 > MQTT_PUBLISH_TIME) {
//      lp_counter0 = 0;
//      publish_MQTT_lwt(); 
//      publish_MQTT_stat();
//    }

    if (comnd == "publish_MQTT_settings"){
        comnd = "";
        publish_MQTT_settings();
    }

    if (comnd == "save_setup"){
        comnd = "";
        save_setup();
    }

    if (comnd == "wipe_setup"){
        comnd = "";
        wipe_setup();
    }

    if (comnd == "init_setup"){
        comnd = "";
        init_setup();
    }

    if (comnd == "publish_MQTT_sensors"){
        comnd = "";
        publish_MQTT_sensors();
    }

    if (comnd == "ESP.restart"){
        comnd = "";
        ESP.restart();
    }

    heating_rules();
    boiler_rules();
    if (heating == 0 && boiler == 0){
       if (burner == true){
         relay(2, "off");
       }
    }
    if (c_burner == true && burner == false){
      relay(2, "on");
      burner_timer = burner_min_time; 
    }else if (c_burner == false && burner == true && burner_timer == 0){
      relay(2, "off");
    }
// OverTemp security
    if (heating_temp > heating_high_limit || heating_temp > boiler_high_limit) {
      if (burner == true){
        relay(2, "off");
        c_burner = false;
        publish_MQTT_event("alarm", "overtemp");        
      }
    }
// Burner security
    if (heating == 0 && boiler_active == false) {
      if (burner == true){
        relay(2, "off");
        c_burner = false;
      }
    }
    if((millis() - lastime1) >= 60000) {  // 1 minute loop)
       lastime1 = millis();
       if (burner_timer > 0) {
         burner_timer--;
       }
       publish_MQTT_lwt(); 
       publish_MQTT_stat();       
       publish_MQTT_sensors();
       publish_MQTT_settings();
    }    
    //sensors.requestTemperatures(); // Send the command to get temperature readings 
    if((millis() - lastime2) >= 6000) {  // 6 seconds loop)
         lastime2 = millis();
//       get_temp();
         publish_MQTT_event("get_sensors", String(lp_counter));
         lp_counter++;
    }
    if((millis() - lastime3) >= 1000) {  // 1 second loop)
         lastime3 = millis();
      if (!client.connected()) {
        reconnect_MQTT();
      }
      client.loop();
      ArduinoOTA.handle();
      yield();
    }
 }
