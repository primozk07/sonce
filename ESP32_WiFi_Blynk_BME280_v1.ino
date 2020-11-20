/*************************************************************
  Download latest Blynk library here:
    https://github.com/blynkkk/blynk-library/releases/latest

  Blynk is a platform with iOS and Android apps to control
  Arduino, Raspberry Pi and the likes over the Internet.
  You can easily build graphic interfaces for all your
  projects by simply dragging and dropping widgets.

    Downloads, docs, tutorials: http://www.blynk.cc
    Sketch generator:           http://examples.blynk.cc
    Blynk community:            http://community.blynk.cc
    Follow us:                  http://www.fb.com/blynkapp
                                http://twitter.com/blynk_app

  Blynk library is licensed under MIT license
  This example code is in public domain.

 *************************************************************
  This example runs directly on ESP32 chip.

  Note: This requires ESP32 support package:
    https://github.com/espressif/arduino-esp32

  Please be sure to select the right ESP32 module
  in the Tools -> Board menu!

  Change WiFi ssid, pass, and Blynk auth token to run :)
  Feel free to apply it to any other example. It's simple!
 *************************************************************/

/* Comment this out to disable prints and save space */
#define BLYNK_PRINT Serial

#include <WiFi.h>
#include <WiFiClient.h>
#include <BlynkSimpleEsp32.h>

// You should get Auth Token in the Blynk App.
// Go to the Project Settings (nut icon).
char auth[] = "yKwkICjMoN7yrLDeiDK_sm4CZuWyZcjR";

// Your WiFi credentials.
// Set password to "" for open networks.
char ssid[] = "D74a";
char pass[] = "Yeti2015Rapid2016";

BlynkTimer timer;

// ------------ BME280 -------------------
//#include <Wire.h>
#include <Adafruit_Sensor.h>
#include <Adafruit_BME280.h>

/*#include <SPI.h>
#define BME_SCK 18
#define BME_MISO 19
#define BME_MOSI 23
#define BME_CS 5*/

#define SEALEVELPRESSURE_HPA (1013.25)

Adafruit_BME280 bme; // I2C
//Adafruit_BME280 bme(BME_CS); // hardware SPI
//Adafruit_BME280 bme(BME_CS, BME_MOSI, BME_MISO, BME_SCK); // software SPI

//-----------------Voltage-Current----------------------------------
#include <Adafruit_ADS1015.h>

Adafruit_ADS1115 ads;  /* Use this for the 16-bit version */
//Adafruit_ADS1015 ads;     /* Use thi for the 12-bit version */

  // The ADC input range (or gain) can be changed via the following
  // functions, but be careful never to exceed VDD +0.3V max, or to
  // exceed the upper and lower limits if you adjust the input range!
  // Setting these values incorrectly may destroy your ADC!
  //                                                                ADS1015  ADS1115
  //                                                                -------  -------
  // ads.setGain(GAIN_TWOTHIRDS);  // 2/3x gain +/- 6.144V  1 bit = 3mV      0.1875mV (default)
  // ads.setGain(GAIN_ONE);        // 1x gain   +/- 4.096V  1 bit = 2mV      0.125mV
  // ads.setGain(GAIN_TWO);        // 2x gain   +/- 2.048V  1 bit = 1mV      0.0625mV
  // ads.setGain(GAIN_FOUR);       // 4x gain   +/- 1.024V  1 bit = 0.5mV    0.03125mV
  // ads.setGain(GAIN_EIGHT);      // 8x gain   +/- 0.512V  1 bit = 0.25mV   0.015625mV
  // ads.setGain(GAIN_SIXTEEN);    // 16x gain  +/- 0.256V  1 bit = 0.125mV  0.0078125mV
float ADC_GAIN = 0.1250;
float ACS712_GAIN = 0.100; // mV/Ampere
float ACS712_0Amp = 2.36; //V
int32_t ACS712_ADC2_0_value = 0;

float Voltage_divider_R1 = 100.0;
float Voltage_divider_R2 =  10.0;

//--------------------------------------------------

// Attach virtual serial terminal to Virtual Pin V1
WidgetTerminal terminal(V9);

// You can send commands from Terminal to your hardware. Just use
// the same Virtual Pin as your Terminal Widget
BLYNK_WRITE(V9)
{
/*
  // if you type "Marco" into Terminal Widget - it will respond: "Polo:"
  if (String("Marco") == param.asStr()) {
    terminal.println("You said: 'Marco'") ;
    terminal.println("I said: 'Polo'") ;
  } else {

    // Send it back
    terminal.print("You said:");
    terminal.write(param.getBuffer(), param.getLength());
    terminal.println();
  }

  // Ensure everything is sent
  terminal.flush();
  */
}




unsigned long delayTime=5000; //reading every 10k miliseconds
unsigned long timex;

void setup()
{
  // Debug console
  Serial.begin(9600);

  Blynk.begin(auth, ssid, pass);
  timer.setInterval(delayTime, myTimerEvent);

  terminal.clear();

  // This will print Blynk Software version to the Terminal Widget when
  // your hardware gets connected to Blynk Server
  terminal.println(F("Blynk v" BLYNK_VERSION ": Device started"));
  terminal.println(F("-------------"));
  terminal.flush();

  Serial.println(F("BME280 with Blynk timer"));
  Serial.print("Blynk timer: ");Serial.print(delayTime/1000);Serial.print(" seconds");
  bool status;

  // default settings
  // (you can also pass in a Wire library object like &Wire2)
  status = bme.begin(0x76);  
  if (!status) {
    Serial.println("Could not find a valid BME280 sensor, check wiring!");
    //while (1);
  }

  ads.setGain(GAIN_ONE);        // 1x gain   +/- 4.096V  1 bit = 2mV      0.125mV
  Serial.println("Getting single-ended readings from AIN0..3");
  Serial.println("ADC Range: ADC0 - R:100/10k @5V --> 44V; GAiN_ONE");
  ads.begin();

  // get average 0 current value
  int i;
  int Average_loop = 100;
  ACS712_ADC2_0_value = 0;
  
   for (int i = 0; i < Average_loop; i++) {
      ACS712_ADC2_0_value += ads.readADC_SingleEnded(2); 
      Serial.print(".");
      delay(50);
   }
   Serial.println(";");
   ACS712_ADC2_0_value /= Average_loop;
   ACS712_0Amp = ACS712_ADC2_0_value*ADC_GAIN/1000.0;
   Serial.print("ADC2 @ 0 current: mV "); Serial.println(ACS712_0Amp*1000); Serial.println();
}

void loop()
{
  Blynk.run();
  timer.run(); // Initiates BlynkTimer
}

void myTimerEvent()
{
  Blynk_PrintValues();
}
  
void Blynk_PrintValues() 
{
  timex=millis();
  float temp=RoundX(bme.readTemperature(),1);
  String str = String(temp, 1) + "℃";
  Serial.print("Temperature = ");  Serial.println(str);
  Blynk.virtualWrite(V1, temp);
  
  float press=RoundX(bme.readPressure() / 100.0F,0);
  str = String(press, 0) + " hPa";
  Serial.print("Pressure = ");  Serial.println(str);
  Blynk.virtualWrite(V2, press);

  Serial.print("Approx. Altitude = ");
  Serial.print(bme.readAltitude(SEALEVELPRESSURE_HPA));
  Serial.println(" m");

  float humi=RoundX(bme.readHumidity(),0);
  str = String(humi, 1) + "%";
  Serial.print("Humidity = ");  Serial.println(str);
  Blynk.virtualWrite(V3, humi);

  Serial.print("Read execution time: "); Serial.println(millis()-timex);
  
  int32_t adc0, adc1, adc2, adc3;
  float volt0, volt1,volt2,volt3; 

  adc0=0; adc1=0;adc2=0;adc3=0;
  //volt0=0; volt1=0;volt2=0;volt3=0;
  int Average_loop = 10;
  
  //read 10x to average
  for (int i = 0; i < Average_loop; i++) {
    adc0 += ads.readADC_SingleEnded(0); 
    adc1 += ads.readADC_SingleEnded(1);
    adc2 += ads.readADC_SingleEnded(2); 
    adc3 += ads.readADC_SingleEnded(3); 
    delay(50);
  }
  //volt0/=Average_loop; volt1/=Average_loop; volt2/=Average_loop; volt3/=Average_loop;
  
  volt0 = adc0/Average_loop*ADC_GAIN/1000.0*(Voltage_divider_R1+Voltage_divider_R2)/Voltage_divider_R2;
  volt1 = adc1/Average_loop*ADC_GAIN/1000.0*1;    //(Voltage_divider_R1+Voltage_divider_R2)/Voltage_divider_R2;
  volt2 = adc2/Average_loop*ADC_GAIN/1000.0; // ACS712 connected
  volt3 = adc3/Average_loop*ADC_GAIN/1000.0; // reference voltage VDD on ADS1115
   
  Serial.print("Read execution time (10x ADC read: "); Serial.println(millis()-timex);

  Serial.print("  ADC0 = ");  Serial.print(RoundX(volt0,1)); Blynk.virtualWrite(V4, RoundX(volt0,1)); Serial.print("V"); 
  Serial.print("  ADC1 = ");          Serial.print(RoundX(volt1,1)); Blynk.virtualWrite(V5, RoundX(volt1,1)); //Serial.println("V"); 
  
  float VDD_ADS1115_voltage_value = volt3*2;    //1-1 divider
  Serial.print("  ADC3 (5V) = ");  Serial.print(RoundX(VDD_ADS1115_voltage_value,1));  Serial.print("V; ADC2-"); Serial.print(volt2*1000); 
  
  //float current_value = (volt2 - (VDD_ADS1115_voltage_value/2))/ACS712_GAIN ;
  float current_value = (volt2 - ACS712_0Amp)/ACS712_GAIN;     // očitno 0 Amp ni odvisna od vhodne napetosti ampak vedno 2,36V
  Serial.print("  I = ");  Serial.print(RoundX(current_value,2));  Serial.print("A");
  Blynk.virtualWrite(V6, RoundX(current_value,2));

  float power_value = current_value * volt0;
  Serial.print("  PWR = ");  Serial.print(RoundX(power_value,2));  Serial.print("W");
  Blynk.virtualWrite(V7, RoundX(power_value,2));

  Serial.println();
  Serial.print("Read execution time: "); Serial.println(millis()-timex);
  Serial.println();

  terminal.print("U0: "); terminal.print(RoundX(volt0,1)); terminal.print("V; ");
  //terminal.print("U1: "); terminal.print(RoundX(volt1,1)); terminal.print("V; ");
  terminal.print("I: "); terminal.print(RoundX(current_value,2)); terminal.print("A ");
  terminal.print("P: "); terminal.print(RoundX(power_value,2)); terminal.println("W");
  terminal.flush();
  
}

float RoundX(float val, byte dec)
{
    float x = val * pow(10, dec);
    float y = round(x);
    float z = x - y;
    if ((int)z == 5)
    {
        y++;
    } else {}
    x = y / pow(10, dec);
    return x;
}
