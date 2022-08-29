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
#include <WiFi.h>
#include <WiFiUdp.h>
#include <OSCMessage.h>
// Don't use PIN_D1 in core v2.0.0 and v2.0.1. Check https://github.com/espressif/arduino-esp32/issues/5868
#define PIN_D2              2         // Pin D2 mapped to pin GPIO2/ADC12/TOUCH2/LED_BUILTIN of ESP32
#define PIN_D3              3         // Pin D3 mapped to pin GPIO3/RX0 of ESP32
#define PIN_D4              4         // Pin D4 mapped to pin GPIO4/ADC10/TOUCH0 of ESP32


unsigned int SWPin = PIN_D4;

#define TIMER0_INTERVAL_MS        1
#define DEBOUNCING_INTERVAL_MS    80

#define LOCAL_DEBUG               1

char ssid[] = "TheB3st$$1";          // your network SSID (name)
char pass[] = "l0ck0N3$$1";                    // your network password

WiFiUDP Udp;                                // A UDP instance to let us send and receive packets over UDP
const IPAddress outIp(192,168,1,202);        // remote IP of your computer
const unsigned int outPort = 8000;          // remote port to receive OSC
const unsigned int localPort = 8888;        // local port to listen for OSC packets (actually not used for sending)

// Init ESP32 timer 0
ESP32Timer ITimer0(0);

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
    RPM = ( 6000 / ( rotationTime * TIMER0_INTERVAL_MS ) );

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

      // Connect to WiFi network
    Serial.println();
    Serial.println();
    Serial.print("Connecting to ");
    Serial.println(ssid);
    WiFi.begin(ssid, pass);

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
#ifdef ESP32
    Serial.println(localPort);
#else
    Serial.println(Udp.localPort());
#endif

  
  while (!Serial);

  delay(100);
  
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

void loop()
{
  if (avgRPM > 0)
  {
    

    
 
    Serial.print(F("RPM  = ")); 
    Serial.print((float) RPM / 100.f);
    Serial.print(F(", avgRPM  = ")); 
    Serial.println((float) avgRPM / 100.f);
    OSCMessage msg("/RPM");
    msg.add(( float) avgRPM);
    Udp.beginPacket(outIp, outPort);
    msg.send(Udp);
    Udp.endPacket();
    msg.empty();
  }

  delay(100);
}
