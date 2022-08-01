// Arduino Sketch for the solar box
#include <TinyGPS++.h>
#include <SoftwareSerial.h>
#include<Wire.h>
#include "U8glib.h"
int led = 7;
static const int RXPin = 4, TXPin = 3;
static const uint32_t GPSBaud = 9600;

// The TinyGPS++ object
TinyGPSPlus gps;

// The serial connection to the GPS device
SoftwareSerial ss(RXPin, TXPin);


// Display
U8GLIB_SSD1306_128X64 u8g(U8G_I2C_OPT_NO_ACK); 


 
void setup(){
  Serial.begin(9600);
  pinMode(led, OUTPUT);
  ss.begin(GPSBaud);
}

void loop(){

  digitalWrite(led, HIGH);
  
  // This sketch displays information every time a new sentence is correctly encoded.
  while (ss.available() > 0){
    gps.encode(ss.read());
    if (gps.location.isUpdated()){
      //Serial.print("Latitude= "); 
      //Serial.print(gps.location.lat(), 6);
      //Serial.print(" Longitude= "); 
      //Serial.println(gps.location.lng(), 6);
    }
     delay(500);

     u8g.firstPage();  
  do {
    
    u8g.setFont(u8g_font_unifont);
    u8g.setPrintPos(0, 20); 
    u8g.print("Latitude = ");
    u8g.print(gps.location.lat(), 6);
    u8g.setPrintPos(0, 50);
    u8g.print("Longitude = ");
    u8g.print(gps.location.lng(), 6);
    
  } while(u8g.nextPage());
}
}
  
