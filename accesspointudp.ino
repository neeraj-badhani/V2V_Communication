/*
    Wireless Serial using UDP ESP8266
    Hardware: NodeMCU
    Circuits4you.com
    2018
    Master Board creates Access Point
*/
#include <ESP8266WiFi.h>
#include <WiFiUdp.h>

const char *ssid = "ECU_tx";
const char *pass = "password123"; 

unsigned int localPort = 2000; // local port to listen for UDP packets

IPAddress ServerIP(192,168,4,1);
IPAddress ClientIP(192,168,4,2);

// A UDP instance to let us send and receive packets over UDP
WiFiUDP udp;

char packetBuffer[20];   //Where we get the UDP data
//=======================================================================
//                Setup
//=======================================================================
void setup()
{
    Serial.begin(128000);
    Serial.println();
    WiFi.softAP(ssid, pass);    //Create Access point

    //Start UDP
    Serial.println("Starting UDP");
    udp.begin(localPort);
    Serial.print("Local port: ");
    Serial.println(udp.localPort());
}
//======================================================================
//                MAIN LOOP
//======================================================================
void loop()
{
    int cb = udp.parsePacket();
    if (!cb) 
    {
      //If serial data is recived send it to UDP
      if(Serial.available()>0)
        {
        udp.beginPacket(ClientIP, 2000);
        //Send UDP requests are to port 2000
        
        char a[1];
        a[0]=char(Serial.read()); //Serial Byte Read
        udp.write(a,1); //Send one byte to ESP8266 
        udp.endPacket();
    }
    else {
      // We've received a UDP packet, send it to serial
      udp.read(packetBuffer, 1); // read the packet into the buffer, we are reading only one byte
      Serial.print(packetBuffer);
      delay(20);
    }
}
}
//======================================================================
