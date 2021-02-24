/*
Author : VAIBHAV
*/
#include <ESP8266WiFi.h>
#include <PubSubClient.h>

// Connect to the WiFi
const char* ssid = "";                            //type your ssid!!!!!!!!!!!!!!!!!!!!!
const char* password = "123123123";                //type your password!!!!!!!!!!!!!!!!!!!!!
const char* mqtt_server = "broker.hivemq.com";                 //!!!!!!!!!!!!!!!!!!!!!

String str = "";
WiFiClient AGV_FireBird;
PubSubClient client(AGV_FireBird);

void setup()
{
  Serial.begin(9600);
  delay(10);
  WiFi.mode(WIFI_STA);
  WiFi.begin(ssid, password);
  while (WiFi.status() != WL_CONNECTED) 
    {
      delay(500);
    }
 client.setServer(mqtt_server, 1883);
 client.setCallback(callback);
 str.reserve(10);

}
 

 
void callback(char* topic, byte* payload, unsigned int length) 
{
 
   delay(100);
   for (int i=0;i<length;i++) 
      {
        char receivedChar = (char)payload[i];
        Serial.write(receivedChar);
      }
}
 
 
void reconnect() 
{
   while (!client.connected()) 
     {
    
     if (client.connect("AGV_FireBird_")) 
         {
          client.subscribe("path_tx");
         } 
     else 
        {
        delay(3000);
        }
     }
}

char buff[10];



void loop()
{
  
 if (!client.connected()) 
   {
     reconnect();
   }
 
 client.loop();
 
 while(Serial.available()>0)
   {
    char c= (char)Serial.read();
    str += c;
    if(str.length()>=3)
      {
        str.toCharArray(buff,10);
        client.publish("feedback",buff);
        str = "";
      }

 }

}
