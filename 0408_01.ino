

unsigned long lastflash;
int RPM;
void ICACHE_RAM_ATTR sens() {
  RPM++;
}

void setup() {
Serial.begin(9600);



// IR Infrared sensor
attachInterrupt(5,sens,RISING); //SENSOR: GPIO0 (NodeMCU - D3)
}

void loop() {

  
  noInterrupts();
  Serial.print(RPM);
  Serial.print(" RPM sek / ");
  Serial.print(RPM*60);
  Serial.println("RPM min. "); 
  RPM=0;
  interrupts(); 
  delay(100);  //1 sek.
}
