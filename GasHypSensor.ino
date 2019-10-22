#include "DHT.h"
#include <OneWire.h>
#include <DallasTemperature.h>
#include <WiFi.h>
#include <PubSubClient.h>
#include "ArduinoJson.h"


// Update these with values suitable for your network.
const char* ssid = "free";
const char* password = "freefree";
const char* mqtt_server = "157.245.102.35";
#define mqtt_port 1883
#define MQTT_USER "dinothunder"
#define MQTT_PASSWORD ""
#define MQTT_SERIAL_PUBLISH_CH "v1/devices/me/telemetry"
#define MQTT_SERIAL_RECEIVER_CH "v1/devices/me/telemetry"

#define DHTPIN 33     
#define DHTTYPE DHT22 
#define MQ8 14
#define MQ9 35
#define MQ135 32
#define MG_PIN 34

#define RL_VALUE 10    //define the load resistance on the board, in kilo ohms
#define RO_CLEAN_AIR_FACTOR 9.21  //RO_CLEAR_AIR_FACTOR=(Sensor resistance in clean air)/RO
#define CALIBARAION_SAMPLE_TIMES 50   //define how many samples you are going to take in the calibration phase
#define CALIBRATION_SAMPLE_INTERVAL 500   //define the time interal(in milisecond) between each samples in the cablibration phase
#define READ_SAMPLE_INTERVAL 50    //define how many samples you are going to take in normal operation
#define READ_SAMPLE_TIMES 5     //define the time interal(in milisecond) between each samples in normal operation


#define GAS_H2 0
#define MG_PIN  34
#define DC_GAIN 8.5
#define READ_SAMPLE_INTERVAL1 50
#define READ_SAMPLE_TIMES1 5
#define ZERO_POINT_VOLTAGE 0.220
#define REACTION_VOLTGAE 0.020

float CO2Curve[3] = {2.602,ZERO_POINT_VOLTAGE,(REACTION_VOLTGAE/(2.602-3))};   

DHT dht(DHTPIN, DHTTYPE);
float H2Curve[3] = {2.3, 0.93,-1.44};
float Ro;

DynamicJsonDocument doc(1024);

WiFiClient wifiClient;

  char charBuf[50];

PubSubClient client(wifiClient);

void setup_wifi() {
    delay(10);
    // We start by connecting to a WiFi network
    Serial.println();
    Serial.print("Connecting to ");
    Serial.println(ssid);
    WiFi.begin(ssid, password);
    while (WiFi.status() != WL_CONNECTED) {
      delay(500);
      Serial.print(".");
    }
    randomSeed(micros());
    Serial.println("");
    Serial.println("WiFi connected");
    Serial.println("IP address: ");
    Serial.println(WiFi.localIP());
}

void reconnect() {
  // Loop until we're reconnected
  while (!client.connected()) {
    Serial.print("Attempting MQTT connection...");
    // Create a random client ID
    String clientId = "ESP32Client-";
    clientId += String(random(0xffff), HEX);
    // Attempt to connect
    if (client.connect(clientId.c_str(),MQTT_USER,MQTT_PASSWORD)) {
      Serial.println("connected");
      //Once connected, publish an announcement...
      Serial.println("Parsing start: ");
 
       //char JSONMessage[] = " {\"temp1\": \"111\", }"; //Original message
         //StaticJsonDocument<256> doc;
        //JsonObject& parsed = doc.parseObject(JSONMessage); //Parse message
        ///JsonObject root;
        //root["co"]=111;
        //Serial.println(root["co"]);
//        JsonObject& parsed = deserializeJson(doc, JSONMessage);


 
      client.publish("v1/devices/me/telemetry","{\"co\":111}");
      // ... and resubscribe
      client.subscribe(MQTT_SERIAL_RECEIVER_CH);
    } else {
      Serial.print("failed, rc=");
      Serial.print(client.state());
      Serial.println(" try again in 5 seconds");
      // Wait 5 seconds before retrying
      delay(5000);
    }
  }
}

void callback(char* topic, byte *payload, unsigned int length) {
    Serial.println("-------new message from broker-----");
    Serial.print("channel:");
    Serial.println(topic);
    Serial.print("data:");  
    Serial.write(payload, length);
    Serial.println();
}

void setup() {
  Serial.begin(115200);
  Serial.setTimeout(500);// Set time out for 
  setup_wifi();
  client.setServer(mqtt_server, mqtt_port);
  client.setCallback(callback);
  reconnect();
  dht.begin();
  reconnect();
  Ro = MQCalibration(MQ8);
}

void publishSerialData(char *serialData){
  if (!client.connected()) {
    reconnect();
  }

  //client.publish(MQTT_SERIAL_PUBLISH_CH, serialData);
}
void loop() {
   client.loop();
   /*if (Serial.available() > 0) {
     char mun[501];
     memset(mun,0, 501);
     Serial.readBytesUntil( '\n',mun,500);
    // publishSerialData("{temp1:111}");
     
   }*/
   
  // String a ="{\"co\":";
// String b = a+String(random(1,100));
 //Serial.println(b);
 
 //String c = b+"}";
 
  //c.toCharArray(charBuf,50);
  //client.publish("v1/devices/me/telemetry",charBuf);
  doc["h2"] = Mq_8();
  doc["co2"] = Mg_811();
  doc["co"] = Mq_9();
  doc["ap"] = Mq_135();
  serializeJson(doc,charBuf);
  client.publish("v1/devices/me/telemetry",charBuf);

  
  //delay(1000);
  Serial.println("Sent");
 }

struct ab{
  float t;
  float h;
};

 
struct ab temp_hum() {
  struct ab th;
  th.h = dht.readHumidity();
  // Read temperature as Celsius (the default)
  th.t = dht.readTemperature();
  //Serial.println(th.t);
  //Serial.println(th.h);
  return th;
}

 float Mq_8()
{
  //Serial.print("H2:"); 
  float x = MQGetGasPercentage(MQRead(MQ8)/Ro,GAS_H2);
   //Serial.println(" ppm " );
  //
   delay(1000);
   return x;
}

 float Mg_811()
 {
  int percentage;
    float volts;
    volts = MGRead(MG_PIN);
    percentage = MGGetPercentage(volts,CO2Curve);
    //Serial.print("CO2: ");
    if (percentage == -1) {
        //Serial.print(percentage);
        //
        delay(1000);
    } else {
        //Serial.print(percentage);
        //Serial.print( " ppm" );
        //
        delay(1000);
    }  
    return percentage;
    //Serial.print("\n");
    delay(200);
 }
 
float MQResistanceCalculation(int raw_adc)
{
  return ( ((float)RL_VALUE*(1023-raw_adc)/raw_adc));
}
 
float MQCalibration(int mq8)
{
  int i;
  float val=0;
 
  for (i=0;i<CALIBARAION_SAMPLE_TIMES;i++) {            //take multiple samples
    val += MQResistanceCalculation(analogRead(mq8));
    delay(CALIBRATION_SAMPLE_INTERVAL);
  }
  val = val/CALIBARAION_SAMPLE_TIMES;                   //calculate the average value
 
  val = val/RO_CLEAN_AIR_FACTOR;                        //divided by RO_CLEAN_AIR_FACTOR yields the Ro 
                                                        //according to the chart in the datasheet 
 
  return val; 
}

float MQRead(int mq8)
{
  int i;
  float rs=0;
 
  for (i=0;i<READ_SAMPLE_TIMES;i++) {
    rs += MQResistanceCalculation(analogRead(mq8));
    delay(READ_SAMPLE_INTERVAL);
  }
 
  rs = rs/READ_SAMPLE_TIMES;
 
  return rs;  
}
 
int MQGetGasPercentage(float rs_ro_ratio, int gas_id)
{
  if ( gas_id == GAS_H2) {
     return MQGetPercentage(rs_ro_ratio,H2Curve);
  }  
  return 0;
}
 
int  MQGetPercentage(float rs_ro_ratio, float *pcurve)
{
  return (pow(10,( ((log(rs_ro_ratio)-pcurve[1])/pcurve[2]) + pcurve[0])));
}

float MGRead(int mg_pin)
{
    int i;
    float v=0;
 
    for (i=0;i<READ_SAMPLE_TIMES1;i++) {
        v += analogRead(mg_pin);
        delay(READ_SAMPLE_INTERVAL1);
    }
    v = (v/READ_SAMPLE_TIMES1) *5/1024 ;
    return v;  
}
 
int  MGGetPercentage(float volts, float *pcurve)
{
   if ((volts/DC_GAIN )>=ZERO_POINT_VOLTAGE) {
      return -1;
   } else { 
      return pow(10, ((volts/DC_GAIN)-pcurve[1])/pcurve[2]+pcurve[0]);
   }
}

float Mq_9()
{
    float sensor_volt;
    float RS_gas; // Get value of RS in a GAS
    float ratio; // Get ratio RS_GAS/RS_air 
    int sensorValue = analogRead(MQ9);
    sensor_volt=(float)sensorValue/1024*5.0;
    RS_gas = (5.0-sensor_volt)/sensor_volt; // omit *RL
    ratio = RS_gas/(-0.06);  // ratio = RS/R0
    //Serial.print("CO_voltage: ");
    //Serial.println(sensor_volt);
    //
   delay(1000);
   return sensor_volt;

}

float Mq_135()
{
  float y = analogRead(MQ135);
  //Serial.println(y);
  //
   delay(1000);
   return y;
}
 
