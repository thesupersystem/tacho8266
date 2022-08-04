
#include <ESP8266WiFi.h>
#include <WiFiUdp.h>
#include <OSCMessage.h>

const char* ssid     = "TheB3st$$1";         // The SSID (name) of the Wi-Fi network you want to connect to
const char* password = "l0ck0N3$$1";     // The password of the Wi-Fi network

WiFiUDP Udp;                                // A UDP instance to let us send and receive packets over UDP
const IPAddress outIp(192,168,1,202);        // remote IP of your computer
const unsigned int outPort = 9999;          // remote port to receive OSC
const unsigned int localPort = 8888;        // local port to listen for OSC packets (actually not used for sending)

unsigned long lastflash;
int RPM;
void ICACHE_RAM_ATTR sens() {
  RPM++;
}


void setup() {
    Serial.begin(115200);

    // IR Infrared sensor
attachInterrupt(5,sens,RISING); //SENSOR: GPIO0 (NodeMCU - D3)

    // Connect to WiFi network
    Serial.println();
    Serial.println();
    Serial.print("Connecting to ");
    Serial.println(ssid);
    WiFi.begin(ssid, password);

    while (WiFi.status() != WL_CONNECTED) {
        delay(500);
        Serial.print(".");
    }
    Serial.println("");

    Serial.println("WiFi connected");
    Serial.println("IP address: ");
    Serial.println(WiFi.localIP());

    Serial.println("Starting UDP");
    Udp.begin(localPort);
    Serial.print("Local port: ");
    Serial.println(Udp.localPort());

}

void loop() {
    OSCMessage msg("/test");
  //  msg.add("hello, osc.");
    msg.add(RPM);
    Udp.beginPacket(outIp, outPort);
    msg.send(Udp);
    Udp.endPacket();
    msg.empty();
    delay(500);

      noInterrupts();
  Serial.print(RPM);
  Serial.print(" RPM sek / ");
  Serial.print(RPM*60);
  Serial.println("RPM min. "); 
  RPM=0;
  interrupts(); 
  delay(100);  //1 sek.
}
