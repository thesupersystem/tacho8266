

#if !defined(ESP32)
  #error This code is intended to run on the ESP32 platform! Please check your Tools->Board setting.
#elif ( defined(ARDUINO_ESP32S3_DEV) || defined(ARDUINO_ESP32_S3_BOX) || defined(ARDUINO_TINYS3) || \
        defined(ARDUINO_PROS3) || defined(ARDUINO_FEATHERS3) )
  #error ESP32_S3 is not supported yet
#endif

// These define's must be placed at the beginning before #include "TimerInterrupt_Generic.h"
// _TIMERINTERRUPT_LOGLEVEL_ from 0 to 4
#define _TIMERINTERRUPT_LOGLEVEL_     0

// To be included only in main(), .ino with setup() to avoid `Multiple Definitions` Linker Error
#include "ESP32TimerInterrupt.h"

// Don't use PIN_D1 in core v2.0.0 and v2.0.1. Check https://github.com/espressif/arduino-esp32/issues/5868
#define PIN_D2              2         // Pin D2 mapped to pin GPIO2/ADC12/TOUCH2/LED_BUILTIN of ESP32
#define PIN_D3              3         // Pin D3 mapped to pin GPIO3/RX0 of ESP32
#define PIN_D4              4         // Pin D4 mapped to pin GPIO4/ADC10/TOUCH0 of ESP32

unsigned int SWPin = PIN_D4;

#define TIMER0_INTERVAL_MS        1
#define DEBOUNCING_INTERVAL_MS    80

#define LOCAL_DEBUG               1
#include <AsyncTCP.h>
#include <ESPAsyncWebServer.h>
#include <WiFiUdp.h>
#include <OSCMessage.h>
#include <WiFi.h>

const char* ssid     = "TheB3st$$1";
const char* password = "l0ck0N3$$1";


WiFiUDP udp;


// Init ESP32 timer 0
ESP32Timer ITimer0(0);


///SERVER///

AsyncWebServer server(80);

// WiFi network name and password:
const char * networkName = "TheB3st$$1";
const char * networkPswd = "l0ck0N3$$1";


const char* PARAM_INPUT_1 = "input1";
const char* PARAM_INPUT_2 = "input2";
const char* PARAM_INPUT_3 = "input3";
const char* PARAM_INPUT_4 = "input4";
const char* PARAM_INPUT_5 = "input5";

    String inputMessage_1;
    String inputParam_1;
    String inputMessage_2;
    String inputParam_2;
    String inputMessage_3;
    String inputParam_3;
    String inputMessage_4;
    String inputParam_4;
    String inputMessage_5;
    String inputParam_5;


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


void notFound(AsyncWebServerRequest *request) {
  request->send(404, "text/plain", "Not found");
}






volatile unsigned long rotationTime = 0;

// Not using float => using RPM = 100 * real RPM
float RPM       = 0;
float avgRPM    = 0;
//uint32_t RPM       = 0;
//uint32_t avgRPM    = 0;

volatile int debounceCounter;

// With core v2.0.0+, you can't use Serial.print/println in ISR or crash.
// and you can't use float calculation inside ISR
// Only OK in core v1.0.6-
bool IRAM_ATTR TimerHandler0(void * timerNo)
{ 
  if ( !digitalRead(SWPin) && (debounceCounter >= DEBOUNCING_INTERVAL_MS / TIMER0_INTERVAL_MS ) )
  {
    //min time between pulses has passed
    // Using float calculation / vars in core v2.0.0 and core v2.0.1 will cause crash
    // Not using float => using RPM = 100 * real RPM
    RPM = ( 6000000 / ( rotationTime * TIMER0_INTERVAL_MS ) );

    avgRPM = ( 2 * avgRPM + RPM) / 3;

    rotationTime = 0;
    debounceCounter = 0;
  }
  else
  {
    debounceCounter++;
  }

  //if (rotationTime >= 5000)
  if (rotationTime >= 1000)
  {
    // If idle, set RPM to 0, don't increase rotationTime
    RPM = 0;

    avgRPM = ( avgRPM + 3 * RPM) / 4;
     
    rotationTime = 0;
  }
  else
  {
    rotationTime++;
  }

  return true;
}

void setup()
{
  
  pinMode(SWPin, INPUT_PULLUP);
  
  Serial.begin(115200);
  connectToWiFi(networkName, networkPswd);
  while (!Serial);

  delay(200);
  
  Serial.print(F("\nStarting RPM_Measure on ")); Serial.println(ARDUINO_BOARD);
  Serial.println(ESP32_TIMER_INTERRUPT_VERSION);
  Serial.print(F("CPU Frequency = ")); Serial.print(F_CPU / 1000000); Serial.println(F(" MHz"));

  // Using ESP32  => 80 / 160 / 240MHz CPU clock ,
  // For 64-bit timer counter
  // For 16-bit timer prescaler up to 1024

  // Interval in microsecs
  if (ITimer0.attachInterruptInterval(TIMER0_INTERVAL_MS * 1000, TimerHandler0))
  {
    Serial.print(F("Starting  ITimer0 OK, millis() = ")); Serial.println(millis());
  }
  else
    Serial.println(F("Can't set ITimer0. Select another freq. or timer"));

  Serial.flush();   
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

void loop()
{
      int ip1 = inputMessage_1.toInt();
    int ip2 = inputMessage_2.toInt();
    int ip3 = inputMessage_3.toInt();
    int ip4 = inputMessage_4.toInt();
    int portNumber = inputMessage_5.toInt();


    // a network broadcast address
    const IPAddress udpAddress(ip1, ip2, ip3, ip4);
    const int udpPort = portNumber;


   udp.begin(WiFi.localIP(),udpPort);

  
  if (avgRPM > 0)
  {
    Serial.print(F("RPM  = "));
    Serial.print((float) RPM / 100.f);
    Serial.print(F(", avgRPM  = ")); 
    Serial.println((float) avgRPM / 100.f);
    OSCMessage msg("/RPM");
    msg.add((unsigned int) RPM);

     udp.beginPacket(udpAddress, udpPort);
    msg.send(udp);
    udp.endPacket();
    msg.empty();
  }

  delay(1000);
}

