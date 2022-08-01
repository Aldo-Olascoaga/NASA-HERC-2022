// Final Arduino Sketch utilised for the IKTAN Roving's telemetry system.


// Encoder
// Motor encoder output pulses per 360 degree revolution (measured manually)
#define ENC_COUNT_REV 620
 
// Encoder output to Arduino Interrupt pin. Tracks the pulse count.
#define ENC_IN_RIGHT_A 3
 
// Other encoder output to Arduino to keep track of wheel direction
// Tracks the direction of rotation.
#define ENC_IN_RIGHT_B 4
 
// True = Forward; False = Reverse
boolean Direction_right = false;
 
// Keep track of the number of right wheel pulses
volatile long right_wheel_pulse_count = 0;
 
// One-second interval for measurements
int interval = 0; // 300
  
// Counters for milliseconds during interval
long previousMillis = 0;
long currentMillis = 0;
 
// Variable for RPM measuerment
float rpm_right = 0;
 
// Variable for angular velocity measurement
float ang_velocity_right = 0;
float ang_velocity_right_deg = 0;
float vel_lineal_ms = 0;
float vel_lineal_kmh = 0;
 
const float rpm_to_radians = 0.10471975512;
const float rad_to_deg = 57.29578;


// Increment the number of pulses by 1
void right_wheel_pulse() {
   
  // Read the value for the encoder for the right wheel
  int val = digitalRead(ENC_IN_RIGHT_B);
 
  if(val == LOW) {
    Direction_right = false; // Reverse
  }
  else {
    Direction_right = true; // Forward
  }
   //Serial.println(Direction_right);
  if (Direction_right) {
    right_wheel_pulse_count++;
  }
  else {
    right_wheel_pulse_count--;
  }
}






// Giroscopio
#include<Wire.h>
const int MPU_addr=0x68;
int16_t AcX,AcY,AcZ,Tmp,GyX,GyY,GyZ;
 
int minVal=265;
int maxVal=402;
 
double x;
double y;
double z;

// Datos del sensor de presión atmosférica:
#include <Wire.h>
#include <SparkFunBME280.h>
BME280 mySensor; //Global sensor object
const int sensorMin = 0; // sensor minimum
const int sensorMax = 1024; // sensor maximum
// Pines de comunicación:
int RELAY1 = 20;
int RELAY2 = 21;
int delayValue=3000;
long count=0;


// Archivo para el sensor de CO2:
#define RZERO 62.88
#include "MQ135.h"
const int ANALOGPIN=0;
MQ135 gasSensor = MQ135(ANALOGPIN);

// Pines de los sensores de vibración:
int sens1 = 46; // Sensor de vibración
int sens2 = 48;
int sens3 = 50;
int sens4 = 52;


void setup() {
    Serial.begin(9600);


  // Encoder:
  // Open the serial port at 9600 bps
  // Set pin states of the encoder
  pinMode(ENC_IN_RIGHT_A , INPUT_PULLUP);
  pinMode(ENC_IN_RIGHT_B , INPUT);
  // Every time the pin goes high, this is a pulse
  attachInterrupt(digitalPinToInterrupt(ENC_IN_RIGHT_A), right_wheel_pulse, RISING);



  // Giroscopio:
  Wire.begin();
  Wire.beginTransmission(MPU_addr);
  Wire.write(0x6B);
  Wire.write(0);
  Wire.endTransmission(true);
 
  // Sensores de vibración:
  pinMode(sens1, INPUT);
  pinMode(sens2, INPUT); 
  pinMode(sens3, INPUT); 
  pinMode(sens4, INPUT);  
   // Sensor de presión atmosférica:
   //For I2C, enable the following and disable the SPI section
  mySensor.settings.commInterface = I2C_MODE;
  mySensor.settings.I2CAddress = 0x76;

  pinMode(RELAY1,OUTPUT);
  pinMode(RELAY2,OUTPUT);
  digitalWrite(RELAY1,HIGH);
  digitalWrite(RELAY2,HIGH);
  while(!Serial);
  //Serial.println("Reading basic values from BME280");
  //Serial.println("");


  if (mySensor.beginI2C() == false) //Begin communication over I2C
  {
  //Serial.println("The chip did not respond. Please check wiring.");
  while(1){
  //Freeze
  }
 }
}



void loop() {
  // Record the time
  currentMillis = millis();
 
  // If one second has passed, print the number of pulses
  if (currentMillis - previousMillis > interval) {
 
    previousMillis = currentMillis;
 
    // Calculate revolutions per minute
    rpm_right = (float)(right_wheel_pulse_count * 60 / ENC_COUNT_REV);
    
    ang_velocity_right = rpm_right * rpm_to_radians;   
    ang_velocity_right_deg = ang_velocity_right * rad_to_deg;
    vel_lineal_ms = (0.35 * rpm_right);
    vel_lineal_kmh = (vel_lineal_ms * 3.6);
    //Serial.print("Velocidad lineal: ");
    //Serial.print(abs(vel_lineal_kmh));
    //Serial.print(" km/h");
    //Serial.println("");
    right_wheel_pulse_count = 0;
  }


  
  // Giroscopio:
  Wire.beginTransmission(MPU_addr);
  Wire.write(0x3B);
  Wire.endTransmission(true);
  Wire.requestFrom(MPU_addr,14,true);
  AcX=Wire.read()<<8|Wire.read();
  AcY=Wire.read()<<8|Wire.read();
  AcZ=Wire.read()<<8|Wire.read();
  int xAng = map(AcX,minVal,maxVal,-90,90);
  int yAng = map(AcY,minVal,maxVal,-90,90);
  int zAng = map(AcZ,minVal,maxVal,-90,90);
 
  x= RAD_TO_DEG * (atan2(-yAng, -zAng)+PI);
  y= RAD_TO_DEG * (atan2(-xAng, -zAng)+PI);
  z= RAD_TO_DEG * (atan2(-yAng, -xAng)+PI);
  delay(500);
  
  
  // Código de los sensores de vibración:
  String measurement_result1;
  long measurement1 = vibration(sens1);
  if (measurement1 == 0){
    measurement_result1 = "NULL";
    //Serial.print();
  }
  else if (measurement1 > 0 && measurement1 <= 20){
    measurement_result1 = "LOW";
    //Serial.print("\nSensor 1: Vibración BAJA");
  }  
  else if (measurement1 >= 20 && measurement1 <= 50){
    measurement_result1 = "MODERATE";
    //Serial.print("\nSensor 1:Vibración MEDIA");
  }
  else
    measurement_result1 = "HIGH";
    //Serial.print("\nSensor 1: Vibración ALTA. ¡PRECAUCIÓN!"); 
  //delay(50);


  String measurement_result2;
  long measurement2 = vibration(sens2);
  if (measurement2 == 0){
    measurement_result2 = "NULL";
    //Serial.print("\nSensor 2: Vibración NULA");
  }    
  else if (measurement2 > 0 &&  measurement2 <= 20){
    measurement_result2 = "LOW";
    //Serial.print("\nSensor 2: Vibración BAJA"); 
  } 
  else if (measurement2 >= 20 && measurement2 <= 50){
    measurement_result2 = "MODERATE";
    //Serial.print("\nSensor 2: Vibración MEDIA");
  } 
  else
    measurement_result2 = "HIGH"; 
    //Serial.print("\nSensor 2: Vibración ALTA. ¡PRECAUCIÓN!");
  //delay(50);


  String measurement_result3;
  long measurement3 = vibration(sens3);
  if (measurement3 == 0){
    measurement_result3 = "NULL";
    //Serial.print("\nSensor 3: Vibración NULA");
  } 
  else if (measurement3 > 0 && measurement3 <= 20){
    measurement_result3 = "LOW";
    //Serial.print("\nSensor 3: Vibración BAJA"); 
  } 
  else if (measurement3 >= 20 && measurement3 <= 50){
    measurement_result3 = "MODERATE";
    //Serial.print("\nSensor 3: Vibración MEDIA");
  }
  else 
    measurement_result3 = "HIGH";
    //Serial.print("\nSensor 3: Vibración ALTA. ¡PRECAUCIÓN!");
  //delay(50);


  String measurement_result4;
  long measurement4 = vibration(sens4);
  if (measurement4 == 0){
    measurement_result4 = "NULL";
    //Serial.print("\nSensor 4: Vibración NULA");
  }
  else if (measurement4 > 0 && measurement4 <= 20){
    measurement_result4 = "LOW";
    //Serial.print("\nSensor 4: Vibración BAJA");
  }   
  else if (measurement4 >= 20 && measurement4 <= 50){
    measurement_result4 = "MODERATE";
    //Serial.print("\nSensor 4: Vibración MEDIA");
  }
  else
    measurement_result4 = "HIGH";
    //Serial.print("\nSensor 4: Vibración ALTA. ¡PRECAUCIÓN!");
  //delay(50);
  //Serial.print("\n");

  
  // Código del sensor MQ135:
  //delay(500);
  float ppm = gasSensor.getPPM();                                       //UNCOMMENT THIS
  //Serial.print("\n");
  //Serial.print("ppm de CO2: ");
  //Serial.print(ppm);
  //Serial.println("    ");
  //Serial.println("    ");
  //delay(500);


  // Código del sensor de presión atmosférica:
  //delay(delayValue);

int sensorReading = analogRead(A0);                                   //UNCOMMENT THIS
//Serial.print("Humedad(%): ");
//Serial.print(mySensor.readFloatHumidity(), 0);
//Serial.println("");

//Serial.print("Presión Atm. Bar.(PA): ");
//Serial.print(mySensor.readFloatPressure(), 0);
//Serial.println("");

//Serial.print("Altitud(m.s.n.m): ");
//Serial.print(mySensor.readFloatAltitudeMeters(), 1);
//Serial.println("");
//Serial.print(mySensor.readFloatAltitudeFeet(), 1);

//Serial.print("Temperatura(°C): ");
//Serial.print(mySensor.readTempC(), 2);
//Serial.println("");
//Serial.print(mySensor.readTempF(), 2);
 Serial.println(measurement_result1+" "+measurement_result2+" "+measurement_result3+" "+measurement_result4+" "+ppm+" "+mySensor.readFloatHumidity()+" "+mySensor.readFloatPressure()+" "+mySensor.readFloatAltitudeMeters()+" "+mySensor.readTempC()+ " " + x + " " + y + " " + z + " " + abs(vel_lineal_kmh)); 

} 






// Funciones a utilizar
// Funciones:
long vibration(int sensor){
  long measurement = pulseIn (sensor, HIGH, 300);  //wait for the pin to get HIGH and returns measurement
  return measurement;
}








 
