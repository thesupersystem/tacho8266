//RPMOSC sends rpm input into a server via OSC. Users can connect device to server via a web portal


#include <Arduino.h>
#ifdef ESP32
  #include <WiFi.h>
  //#include <AsyncTCP.h>
#else
  #include <ESP8266WiFi.h>
  #include <ESPAsyncTCP.h>
#endif
#include <ESPAsyncWebServer.h>
#include <WiFiUdp.h>
#include <OSCMessage.h>

///RPM///

float value=0;
float rev=0;
int rpm;
int oldtime=0;
int newtime;

void isr() //interrupt service routine
{
rev++;
}


///SERVER///

AsyncWebServer server(80);

// REPLACE WITH YOUR NETWORK CREDENTIALS
const char* ssid = "TheB3st$$1";
const char* password = "l0ck0N3$$1";

// WiFi network name and password:
const char * networkName = "TheB3st$$1";
const char * networkPswd = "l0ck0N3$$1";


const char* PARAM_INPUT_1 = "input1";
String inputMessage_1;


      // HTML web page to handle 3 input fields (input1, input2, input3)
          const char index_html[] PROGMEM = R"rawliteral(
          <!DOCTYPE HTML><html><head>
          <title>ESP Input Form</title>
          <meta name="viewport" content="width=device-width, initial-scale=1">
          </head><body>
          <h1>Unreal OSC Camera Controller</h1>
               <p> Key In IP Address of OSC Receiver </p>
               <form action="/get">
               input1: 
               <input type="text" maxlength="3" name="input1">
               <input type="text" maxlength="3" name="input2">
               <input type="text" maxlength="3" name="input3">
               <input type="text" maxlength="3" name="input4">
                <br>
                <br />
                 port:
                <input type="text" maxlength="4" name="input5">
                <br>
                <br />
                <input type="submit" value="Submit">
                </form><br>
                  <br>
               <p> Left Controller </p>
        <p> Joystick : /joy1 (2 outputs) </p>
        
        
            </body></html>)rawliteral";


//Are we currently connected?
boolean connected = false;


//The udp library class
WiFiUDP udp;


void notFound(AsyncWebServerRequest *request) {
  request->send(404, "text/plain", "Not found");
}





void setup() {
  Serial.begin(115200);
  connectToWiFi(networkName, networkPswd);
  attachInterrupt(digitalPinToInterrupt(18),isr,RISING); //attaching the interrupt

}

void loop() {
  RPMcal();

}

/// SERVER TINGS ///

void connectToWiFi(const char * ssid, const char * pwd){
  Serial.println("Connecting to WiFi network: " + String(ssid));

  // delete old config
  WiFi.disconnect(true);
  //register event handler
  WiFi.onEvent(WiFiEvent);
  
  //Initiate connection
  WiFi.begin(ssid, pwd);

  Serial.println("Waiting for WIFI connection...");
}

//wifi event handler
void WiFiEvent(WiFiEvent_t event){
    switch(event) {
      case SYSTEM_EVENT_STA_GOT_IP:
          //When connected set 
          Serial.print("WiFi connected! IP address: ");
          Serial.println(WiFi.localIP());  
       
          connected = true;
          ServerRun();
          break;
      case SYSTEM_EVENT_STA_DISCONNECTED:
          Serial.println("WiFi lost connection");
          connected = false;
          break;
    }
}

void ServerRun(){
    // Send web page with input fields to client
  server.on("/", HTTP_GET, [](AsyncWebServerRequest *request){
    request->send_P(200, "text/html", index_html);
  });

  // Send a GET request to <ESP_IP>/get?input1=<inputMessage>
  server.on("/get", HTTP_GET, [] (AsyncWebServerRequest *request) {
    

        
    // GET input1 value on <ESP_IP>/get?input1=<inputMessage>
    if (request->hasParam(PARAM_INPUT_1)) {
      inputMessage_1 = request->getParam(PARAM_INPUT_1)->value();
      inputParam_1 = PARAM_INPUT_1;
      inputMessage_2 = request->getParam(PARAM_INPUT_2)->value();
      inputParam_2 = PARAM_INPUT_2;
      inputMessage_3 = request->getParam(PARAM_INPUT_3)->value();
      inputParam_3 = PARAM_INPUT_3;
      inputMessage_4 = request->getParam(PARAM_INPUT_4)->value();
      inputParam_4 = PARAM_INPUT_4;
      inputMessage_5 = request->getParam(PARAM_INPUT_5)->value();
      inputParam_5 = PARAM_INPUT_5;      
    }

    else {
      inputMessage_1 = "No message sent";
      inputParam_1 = "none";

       inputMessage_2 = "No message sent";
      inputParam_2 = "none";

       inputMessage_3 = "No message sent";
      inputParam_3 = "none";

      inputMessage_4 = "No message sent";
      inputParam_4 = "none";
      inputMessage_5 = "No message sent";
      inputParam_5 = "none";      
    }
    Serial.println(inputMessage_1);
        
request->send(200, "text/html", "HTTP GET request sent to your ESP on input field (" 
                                     + inputParam_1 + ") with value: " + inputMessage_1 +
                                     "<br><a href=\"/\">Return to Home Page</a>");
Serial.println(inputMessage_2);
            
request->send(200, "text/html", "HTTP GET request sent to your ESP on input field (" 
                                     + inputParam_2 + ") with value: " + inputMessage_2 +
                                     "<br><a href=\"/\">Return to Home Page</a>");

Serial.println(inputMessage_3);

request->send(200, "text/html", "HTTP GET request sent to your ESP on input field (" 
                                     + inputParam_3 + ") with value: " + inputMessage_3 +
                                     "<br><a href=\"/\">Return to Home Page</a>");

Serial.println(inputMessage_4);
        
request->send(200, "text/html", "HTTP GET request sent to your ESP on input field (" 
                                     + inputParam_4 + ") with value: " + inputMessage_4 +
                                     "<br><a href=\"/\">Return to Home Page</a>");
Serial.println(inputMessage_5);
            
request->send(200, "text/html", "HTTP GET request sent to your ESP on input field (" 
                                     + inputParam_5 + ") with value: " + inputMessage_5 +
                                     "<br><a href=\"/\">Return to Home Page</a>");

 int ip1 = inputMessage_1.toInt();
 int ip2 = inputMessage_2.toInt();
 int ip3 = inputMessage_3.toInt();
 int ip4 = inputMessage_4.toInt();
 int portNumber = inputMessage_5.toInt();


// a network broadcast address
 const IPAddress udpAddress(ip1, ip2, ip3, ip4);
 const int udpPort = portNumber;


   udp.begin(WiFi.localIP(),udpPort);

 if(connected){

     OSCMessage msg("/UDP successfully Connected!");
//initializes the UDP state
          //This initializes the transfer buffer
          
    udp.beginPacket(udpAddress, udpPort);
    msg.send(udp);
   udp.endPacket();
    msg.empty();

  }

  });
  server.onNotFound(notFound);
  server.begin();
  
  }

///RPM THINGS ///

void RPMcal(){
    
    int ip1 = inputMessage_1.toInt();
    int ip2 = inputMessage_2.toInt();
    int ip3 = inputMessage_3.toInt();
    int ip4 = inputMessage_4.toInt();
    int portNumber = inputMessage_5.toInt();


    // a network broadcast address
    const IPAddress udpAddress(ip1, ip2, ip3, ip4);
    const int udpPort = portNumber;


   udp.begin(WiFi.localIP(),udpPort);

    delay(1000);
    detachInterrupt(0); //detaches the interrupt
    newtime=millis()-oldtime; //finds the time 
    int wings= 3; // no of wings of rotating object, for disc object use 1 with white tape on one side
    int RPMnew = rev/wings; //here we used fan which has 3 wings
    rpm=(RPMnew/newtime)*60000; //calculates rpm
    oldtime=millis(); //saves the current time
    rev=0;

Serial.println("RPM" + rpm);

attachInterrupt(digitalPinToInterrupt(18),isr,RISING); 

        if (connected){
  OSCMessage msg("/RPM");
  msg.add((unsigned int) rpm);

     udp.beginPacket(udpAddress, udpPort);
    msg.send(udp);
    udp.endPacket();
    msg.empty();



}

