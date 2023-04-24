#include <WiFi.h>
#include <Adafruit_MQTT.h>
#include <Adafruit_MQTT_Client.h>
#include <DHT.h>

byte rpin = 25;
byte gpin = 26;
byte bpin = 27;
byte rchannel = 0;
byte gchannel = 1;
byte bchannel = 2;
byte resolution = 8;
int frequency = 5000;
byte rval , gval , bval = 0;
byte dht_pin = 4;
#define dht_type DHT11
DHT dht(dht_pin , dht_type);

const char ssid[] = "ssid";
const char password[] = "password";

#define IO_USERNAME  "AshutoshSwamy"
#define IO_KEY       "aio_ULRn98e0dxarXa1ya0WadQSTPsTp"
#define IO_BROKER    "io.adafruit.com"
#define IO_PORT       1883

WiFiClient wificlient;
Adafruit_MQTT_Client mqtt(&wificlient , IO_BROKER , IO_PORT , IO_USERNAME , IO_KEY);

Adafruit_MQTT_Subscribe red = Adafruit_MQTT_Subscribe(&mqtt , IO_USERNAME"/feeds/redvalue");
Adafruit_MQTT_Subscribe green = Adafruit_MQTT_Subscribe(&mqtt , IO_USERNAME"/feeds/greenvalue");
Adafruit_MQTT_Subscribe blue = Adafruit_MQTT_Subscribe(&mqtt , IO_USERNAME"/feeds/bluevalue");
Adafruit_MQTT_Publish dp = Adafruit_MQTT_Publish(&mqtt , IO_USERNAME"/feeds/dew");
Adafruit_MQTT_Publish tc = Adafruit_MQTT_Publish(&mqtt , IO_USERNAME"/feeds/temperature celcius");
Adafruit_MQTT_Publish tf = Adafruit_MQTT_Publish(&mqtt , IO_USERNAME"/feeds/temperature fahrenheit");
Adafruit_MQTT_Publish tk = Adafruit_MQTT_Publish(&mqtt , IO_USERNAME"/feeds/temperature kelvin");
Adafruit_MQTT_Publish h = Adafruit_MQTT_Publish(&mqtt , IO_USERNAME"/feeds/humidity");

void setup(){
  Serial.begin(115200);
  Serial.print("Connecting with : ");
  Serial.println(ssid);
  WiFi.begin(ssid , password);
  while (WiFi.status() != WL_CONNECTED)
  {
    Serial.print(".");
    delay(500);
  }
  Serial.println();
  Serial.println("Connected !");
  Serial.print("IP assigned by AP : ");
  Serial.println(WiFi.localIP());
  Serial.println();

  ledcSetup(rchannel , frequency , resolution);
  ledcSetup(gchannel , frequency , resolution);
  ledcSetup(bchannel , frequency , resolution);

  ledcAttachPin(rpin , rchannel);
  ledcAttachPin(gpin , gchannel);
  ledcAttachPin(bpin , bchannel);

  dht.begin();

  mqtt.subscribe(&red);
  mqtt.subscribe(&green);
  mqtt.subscribe(&blue);
}

void loop(){
  mqttconnect();

  float tempc = dht.readTemperature();
  float tempf = dht.readTemperature(true);
  float tempk = tempc + 273.15;
  float humidity = dht.readHumidity();
  float dew_point = (tempc - (100 - humidity) / 5); 

  if (isnan(tempc)  ||  isnan(tempf)  ||  isnan(humidity)){
    Serial.println("Sensor not working!");
    delay(1000);
    return;
  }

  String val = String(tempc) + " *C" + "\t" + String(tempf) + " *F" + "\t" + String(tempk) + " *K" + "\t" + 
               String(humidity) + " %RH" + "\t" + String(dew_point) + " *C";
  Serial.println(val);

  if (!tc.publish(tempc)  ||  !tf.publish(tempf)  ||  !tk.publish(tempk)  ||  !dp.publish(dew_point)  ||  !h.publish(humidity)){
    Serial.println("Can't publish!");
  }
  
  Adafruit_MQTT_Subscribe *subscription;
  while (true){
    subscription = mqtt.readSubscription(5000); 
    if (subscription  ==  0)  {
      Serial.println("Can't catch feed");
      break;
    } else {
      if (subscription  ==  &red){
        String temp = (char *)red.lastread;

        rval = temp.toInt();
        makecolor(rval , gval , bval);
      } else if (subscription  ==  &green){
        String temp = (char *)green.lastread;

        gval = temp.toInt();
        makecolor(rval , gval , bval);
      } else if (subscription  ==  &blue){
        String temp = (char *)blue.lastread;

        bval = temp.toInt();
        makecolor(rval , gval , bval);
      }
    }
  }

  delay(7000);
}

void mqttconnect(){
  if (mqtt.connected()) return;

  else
  {
    while (true)
    {
      int connection = mqtt.connect(); 
      if (connection  ==  0){
        Serial.println("Connected to IO");
        break;
      }
      else{
        Serial.println("Can't Connect");
        mqtt.disconnect();
        Serial.println(mqtt.connectErrorString(connection));
        delay(5000);
      }
    }
  }
}

void makecolor(byte r , byte g , byte b){
  Serial.print("RED : ");
  Serial.print(r);
  Serial.print('\t');
  Serial.print("GREEN : ");
  Serial.print(g);
  Serial.print('\t');
  Serial.print("BLUE : ");
  Serial.println(b);

  ledcWrite(rchannel , r);
  ledcWrite(gchannel , g);
  ledcWrite(bchannel , b);
}