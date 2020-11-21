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
char auth[] = "yKwkICjMoN7yrLDeiDK_sm4CZuWyZcjR";  //Blynk cloud ...ESP32 BME280 project
//char auth[] = "55836bb660e54f3995d29ed3dfa3b2b0";    // blynk local server @10.10.10.112

// Your WiFi credentials.
// Set password to "" for open networks.
//char ssid[] = "D74a";
//char pass[] = "Yeti2015Rapid2016";
char* ssid[] = {"iPhone 2017 p", "D74a",              "xx"}; //list a necessary wifi networks
char* pass[] = {"hsiphone2017" , "Yeti2015Rapid2016", "XY"}; //list a passwords



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

//copied from https://community.blynk.cc/t/how-i-can-use-multiple-wifi-network/31667/15


void MultyWiFiBlynkBegin() {
  int ssid_count=0;
  int ssid_mas_size = sizeof(ssid) / sizeof(ssid[0]);
  do {
    Serial.println("Trying to connect to wi-fi " + String(ssid[ssid_count]));
    WiFi.begin(ssid[ssid_count], pass[ssid_count]);    
    int WiFi_timeout_count=0;
    while (WiFi.status() != WL_CONNECTED && WiFi_timeout_count<50) { //waiting 10 sec
      delay(200);
      Serial.print(".");
      ++WiFi_timeout_count;
    }
    if (WiFi.status() == WL_CONNECTED) {
      Serial.println("Connected to WiFi! Now I will check the connection to the Blynk server");
      Blynk.config(auth);
      Blynk.connect(5000); //waiting 5 sec
    }
    ++ssid_count; 
  }  while (!Blynk.connected() && ssid_count<ssid_mas_size);
  
  if (!Blynk.connected() && ssid_count==ssid_mas_size) {
    Serial.println("I could not connect to blynk Ignore and move on. but still I will try to connect to wi-fi " + String(ssid[ssid_count-1]));
  }
  if (!Blynk.connected()) {
    Serial.println("Restarting in 10 sec...");
    delay(10000);  // 10 sec
    ESP.restart();
  }
}


// copied from https://github.com/khoih-prog/Blynk_WM/blob/master/examples/Blynk_WM_Template/Blynk_WM_Template.ino
//void connectToWLANandBlynk()
//{
//  // Setup WLAN and Blynk
//  Serial.print ( "\nSetting up WLAN and Blynk " );  
//    Serial.println ( "Starting WiFi.begin (no WM)" );  
//    WiFi.begin ( ssid, pass );
//    Serial.println ( "... waiting a few seconds for WiFi ..." );    
//      
//    // REBOOT if we do not have a good WiFi connection
//    if ( WiFi.status() != WL_CONNECTED )
//    {
//      Serial.println ( "Resetting in a few seconds...\n\n\n\n\n" );
//      delay ( 3000 );  
//      ESP.restart();
//    } 
//    
//    // Configure and launch Blynk
//    Blynk.config ( auth );
//    Blynk.connect ( 2500 ); // Don't get stuck hanging for more than 2500 millis.
//
//  if ( Blynk.connected() ) 
//  {
//    Serial.println ( "Blynk connected just fine" ); 
//    Serial.print   ( "  IP address  " ); Serial.println ( WiFi.localIP() ) ;
//    Serial.print   ( "  MAC address " ); Serial.println ( WiFi.macAddress() );  
//    Serial.println();  
//  }
//  else Serial.println ( "Blynk NOT CONNECTED \n\n" );  
//} // end connectToWLANandBlynk


unsigned long delayTime=5000; // miliseconds
unsigned long timex;

void setup()
{
  // Debug console
  Serial.begin(9600);

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
  int Average_loop = 20;
  ACS712_ADC2_0_value = 0; 
  for (int i = 0; i < Average_loop; i++) {
    Serial.print("  ADC0: "); Serial.print(ads.readADC_SingleEnded(0)*ADC_GAIN/1000.0*1); 
    Serial.print("  ADC1: "); Serial.print(ads.readADC_SingleEnded(1)*ADC_GAIN/1000.0*1); 
    Serial.print("  ADC2: "); Serial.print(ads.readADC_SingleEnded(2)*ADC_GAIN/1000.0*1); 
    Serial.print("  ADC3: "); Serial.print(ads.readADC_SingleEnded(3)*ADC_GAIN/1000.0*1); 
    Serial.println();
  }
  
//   for (int i = 0; i < Average_loop; i++) {
      ACS712_ADC2_0_value += ads.readADC_SingleEnded(2); 
      Serial.print(".");
      delay(50);
//  }
   Serial.println(";");
//   ACS712_ADC2_0_value /= Average_loop;
   ACS712_0Amp = ACS712_ADC2_0_value*ADC_GAIN/1000.0;
   Serial.print("ADC2 @ 0 current: mV "); Serial.print(ACS712_0Amp*1000); 
   Serial.print(" ADC rough value:"); Serial.print(ACS712_ADC2_0_value); 
   Serial.print(" ADC rough value * 0,125:"); Serial.print(ACS712_ADC2_0_value*ADC_GAIN/1000.0); 
   Serial.println();
   Serial.print("ADC1: "); Serial.print(ads.readADC_SingleEnded(1)*ADC_GAIN/1000.0*1); 
   Serial.println();

  MultyWiFiBlynkBegin(); //instead Blynk.begin(auth, ssid, pass);
  //connectToWLANandBlynk();
  // Blynk.begin(auth, ssid, pass);
  ////Blynk.begin(auth, ssid, pass, IPAddress(10,10,10,112));
  timer.setInterval(delayTime, myTimerEvent);


}

void loop()
{
    if ( WiFi.status() != WL_CONNECTED )
    {
      Serial.println ( "loop: No WiFi - Resetting in 10 seconds...\n\n\n\n\n" );
      delay ( 10000 );  
      ESP.restart();
    } 

  Blynk.run();
  timer.run(); // Initiates BlynkTimer
}

// preveri tudi opcijo z Blynk connect + timer 
// https://community.blynk.cc/t/solved-how-to-run-blynk-run-only-when-wifi-connection-is-established/6492/13



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

    Serial.print("1.1 ADC0: "); Serial.print(ads.readADC_SingleEnded(0)*ADC_GAIN/1000.0*1); 
    Serial.print("  ADC1: "); Serial.print(ads.readADC_SingleEnded(1)*ADC_GAIN/1000.0*1); 
    Serial.print("  ADC2: "); Serial.print(ads.readADC_SingleEnded(2)*ADC_GAIN/1000.0*1); 
    Serial.print("  ADC3: "); Serial.print(ads.readADC_SingleEnded(3)*ADC_GAIN/1000.0*1); 
    Serial.println();
        Serial.print("1.2  ADC0: "); Serial.print(ads.readADC_SingleEnded(0)*ADC_GAIN/1000.0*1); 
    Serial.print("  ADC1: "); Serial.print(ads.readADC_SingleEnded(1)*ADC_GAIN/1000.0*1); 
    Serial.print("  ADC2: "); Serial.print(ads.readADC_SingleEnded(2)*ADC_GAIN/1000.0*1); 
    Serial.print("  ADC3: "); Serial.print(ads.readADC_SingleEnded(3)*ADC_GAIN/1000.0*1); 
    Serial.println();

    adc0 = ads.readADC_SingleEnded(0); 
    adc1 = ads.readADC_SingleEnded(1);
    adc2 = ads.readADC_SingleEnded(2); 
    adc3 = ads.readADC_SingleEnded(3); 
  
  volt0 = adc0*ADC_GAIN/1000.0*(Voltage_divider_R1+Voltage_divider_R2)/Voltage_divider_R2;
  volt1 = adc1*ADC_GAIN/1000.0*1;    //(Voltage_divider_R1+Voltage_divider_R2)/Voltage_divider_R2;
  volt2 = adc2*ADC_GAIN/1000.0; // ACS712 connected
  volt3 = adc3*ADC_GAIN/1000.0; // reference voltage VDD on ADS1115

    Serial.print("1.3  ADC0: "); Serial.print(ads.readADC_SingleEnded(0)*ADC_GAIN/1000.0*1); 
    Serial.print("  ADC1: "); Serial.print(ads.readADC_SingleEnded(1)*ADC_GAIN/1000.0*1); 
    Serial.print("  ADC2: "); Serial.print(ads.readADC_SingleEnded(2)*ADC_GAIN/1000.0*1); 
    Serial.print("  ADC3: "); Serial.print(ads.readADC_SingleEnded(3)*ADC_GAIN/1000.0*1); 
    Serial.println();

  Serial.print("Read execution time: "); Serial.println(millis()-timex);

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
