/*************************************************************
 * 
 * ver ..-11.2020 - dodan Multi-Wifi
 * ver 09-12-2020 - dodan Blynk Local server
 * ver 15-12-2020 - dodani DS18B20 senzorji
 * 
 * 
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

#define BLYNK_CLOUD 1

#ifdef BLYNK_CLOUD
  char auth[] = "yKwkICjMoN7yrLDeiDK_sm4CZuWyZcjR";  //Blynk cloud ...ESP32 BME280 project
#else
  char auth[] = "55836bb660e54f3995d29ed3dfa3b2b0";    // blynk local server @10.10.10.112
#endif


// Your WiFi credentials.
// Set password to "" for open networks.
char ssid1[] = "D74a";
char pass1[] = "Yeti2015Rapid2016";

char* ssid[] = {"iPhone 2017 p", "D74a",              "krek wifi",     "D74k"      }; //list a necessary wifi networks
char* pass[] = {"hsiphone2017" , "Yeti2015Rapid2016", "pozabljeno123", "Klet74LS"  }; //list a passwords
int wifi_ssid=0;


BlynkTimer timer, timer1;

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
/*
//Adafruit_ADS1015 ads;     //* Use thi for the 12-bit version

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
  */
float ADC_GAIN = 0.1250;
float ACS712_GAIN = 0.100; // mV/Ampere
float ACS712_0Amp = 2.36; //V
int32_t ACS712_ADC2_0_value = 0;

float Voltage_divider_R1 = 100.0;
float Voltage_divider_R2 =  10.0;

int32_t adc0, adc1, adc2, adc3;  // global declaration for averaged reading
int     adc_reading_count;       // global declaration for averaged reading

//--------------------------------------------------

#include <Adafruit_GFX.h>
#include <Adafruit_SSD1306.h>

#define SCREEN_WIDTH 128 // OLED display width, in pixels
#define SCREEN_HEIGHT 32 // OLED display height, in pixels

// Declaration for an SSD1306 display connected to I2C (SDA, SCL pins)
#define OLED_RESET     4 // Reset pin # (or -1 if sharing Arduino reset pin)
Adafruit_SSD1306 display(SCREEN_WIDTH, SCREEN_HEIGHT, &Wire, OLED_RESET);

#define NUMFLAKES     10 // Number of snowflakes in the animation example

//--------------------------------------------------
/*********
  Rui Santos
  Complete project details at https://RandomNerdTutorials.com  
*********/

#include <OneWire.h>
#include <DallasTemperature.h>

// Data wire is plugged TO GPIO 4
#define ONE_WIRE_BUS 4

// Setup a oneWire instance to communicate with any OneWire devices (not just Maxim/Dallas temperature ICs)
OneWire oneWire(ONE_WIRE_BUS);

// Pass our oneWire reference to Dallas Temperature. 
DallasTemperature sensors(&oneWire);

// Number of temperature devices found
int numberOfDevices;

// We'll use this variable to store a found device address
DeviceAddress tempDeviceAddress; 


float RefLow[] = {0,0,0};
float RefHigh[] = {86.6, 86.6, 86.6};
float RefRange[] = {86.6, 86.6, 86.6};

float RawLow[] = { -6.25, -7.25, -7.00};
float RawHigh[]= {80.75, 86.25, 88.25};
float RawRange[] = {87.00, 93.50, 95.25};

//float RawRange = RawHigh - RawLow;
//float ReferenceRange = ReferenceHigh - ReferenceLow;

float RawValue[3], CorrectedValue[3];


//--------------------------------------------------

// Attach virtual serial terminal to Virtual Pin V9
WidgetTerminal terminal(V9);

// You can send commands from Terminal to your hardware. Just use
// the same Virtual Pin as your Terminal Widget
BLYNK_WRITE(V9) {
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


//Widget Colors ----------------------------
#define BLYNK_White     "#ffffff"
#define BLYNK_GREEN     "#23C48E"  // 1
#define BLYNK_BLUE      "#04C0F8"  // 2
#define BLYNK_YELLOW    "#ED9D00"  // 3
#define BLYNK_RED       "#D3435C"  // 4
#define BLYNK_DARK_BLUE "#5F7CD8"  // 5

WidgetLED led1(V10), led_cool(V14);

void MultyWiFiBlynkBegin() {
  int ssid_count=0;
  int ssid_mas_size = sizeof(ssid) / sizeof(ssid[0]);
  do {
    Serial.print("Trying to connect to wi-fi <" + String(ssid[ssid_count]));Serial.println(">");
    WiFi.begin(ssid[ssid_count], pass[ssid_count]);    
    int WiFi_timeout_count=0;
    while (WiFi.status() != WL_CONNECTED && WiFi_timeout_count<50) { //waiting 10 sec
      delay(200);
      Serial.print(".");
      ++WiFi_timeout_count;
    }
    Serial.println(";");
    if (WiFi.status() == WL_CONNECTED) {
      Serial.println("Connected to WiFi! Now I will check the connection to the Blynk server");
      display.print("wi-fi <" + String(ssid[ssid_count]));display.println(">"); display.display(); delay(1000);
      wifi_ssid=ssid_count;
      #ifdef BLYNK_CLOUD
        Blynk.config(auth);                                   //use when connecting to Cloud
        Serial.println("Blynk server -- CLOUD");
      #else  
        Blynk.config(auth, IPAddress(10,10,10,112),8080);   //local blynk
        Serial.println("Blynk server -- LOCAL");
      #endif  
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

void ScanI2C() {
  byte error, address;
  int nDevices;
  Serial.println("******************************************************");
  Serial.println("I2C Scanning...");
  nDevices = 0;
  for(address = 1; address < 127; address++ ) {
    Wire.beginTransmission(address);
    error = Wire.endTransmission();
    if (error == 0) {
      Serial.print("   I2C device found at address 0x");
      if (address<16) {
        Serial.print("0");
      }
      Serial.println(address,HEX);
      nDevices++;
    }
    else if (error==4) {
      Serial.print("    Unknow error at address 0x");
      if (address<16) {
        Serial.print("0");
      }
      Serial.println(address,HEX);
    }    
  }
  if (nDevices == 0) {
    Serial.println("No I2C devices found\n");
  }
  else {
    Serial.println("done\n");
  }
}

void Setup_DS18B20() {
   // Start up the library
  sensors.begin();
  
  // Grab a count of devices on the wire
  numberOfDevices = sensors.getDeviceCount();
  
  // locate devices on the bus
  Serial.print("Locating devices...");
  Serial.print("Found ");
  Serial.print(numberOfDevices, DEC);
  Serial.println(" devices.");

  // Loop through each device, print out address
  for(int i=0;i<numberOfDevices; i++){
    // Search the wire for address
    if(sensors.getAddress(tempDeviceAddress, i)){
      Serial.print("Found device ");
      Serial.print(i, DEC);
      Serial.print(" with address: ");
      printAddress(tempDeviceAddress);
      Serial.println();
    } else {
      Serial.print("Found ghost device at ");
      Serial.print(i, DEC);
      Serial.print(" but could not detect address. Check power and cabling");
    }
  }
}

unsigned long delayTime=1000; // miliseconds
unsigned long timex;
// led semaphore
const byte Led_Red=33, Led_Yellow=25, Led_Green=32;
const byte PresRelais=26; // output pins
float Led_Limit_Yellow=100, Led_Limit_Green=0;

const byte CoolRelais=16; // output pins
float Cool_Led_Limit_Yellow=0, Cool_Led_Limit_Green=-20;

void setup() {
  // Debug console
  Serial.begin(9600);

  // setup led semaphore and pressure relais
  pinMode(Led_Red, OUTPUT); 
  pinMode(Led_Yellow, OUTPUT); 
  pinMode(Led_Green, OUTPUT); 
  pinMode(PresRelais, OUTPUT); digitalWrite(PresRelais, HIGH); led1.off(); 
  digitalWrite(Led_Red, LOW);digitalWrite(Led_Yellow, LOW);digitalWrite(Led_Green, LOW);
  digitalWrite(Led_Red, HIGH);delay(500);digitalWrite(Led_Yellow, HIGH);delay(500);digitalWrite(Led_Green, HIGH);delay(500);
  digitalWrite(Led_Red, LOW);digitalWrite(Led_Yellow, LOW);digitalWrite(Led_Green, LOW); delay(500);
  digitalWrite(Led_Red, HIGH);digitalWrite(Led_Yellow, HIGH);digitalWrite(Led_Green, HIGH);delay(500);
  digitalWrite(Led_Red, LOW);digitalWrite(Led_Yellow, LOW);digitalWrite(Led_Green, LOW);

   // setup led cooling relais
   pinMode(CoolRelais, OUTPUT); digitalWrite(PresRelais, HIGH); led_cool.off(); 

   
  Serial.println("*************************************************************");

  // SSD1306_SWITCHCAPVCC = generate display voltage from 3.3V internally
  if(!display.begin(SSD1306_SWITCHCAPVCC, 0x3C)) { // Address 0x3C for 128x32
    Serial.println(F("SSD1306 allocation failed"));
    //for(;;); // Don't proceed, loop forever
  }

  // Show initial display buffer contents on the screen --
  // the library initializes this with an Adafruit splash screen.
  display.display();
  delay(2000); // Pause for 2 seconds

  // Clear the buffer
  display.clearDisplay();

  // Draw a single pixel in white
  display.drawPixel(10, 10, SSD1306_WHITE);

  // Show the display buffer on the screen. You MUST call display() after
  // drawing commands to make them visible on screen!
  display.display();
  delay(2000);
  // display.display() is NOT necessary after every single drawing command,
  // unless that's what you want...rather, you can batch up a bunch of
  // drawing operations and then update the screen all at once by calling
  // display.display(). These examples demonstrate both approaches...

  // Invert and restore display, pausing in-between
  //display.invertDisplay(true);
  //delay(1000);
  //display.invertDisplay(false);
  //delay(1000);
 
  display.clearDisplay();
  display.setTextSize(1);             // Normal 1:1 pixel scale
  display.setTextColor(SSD1306_WHITE);        // Draw white text
  display.setCursor(0,0);             // Start at top-left corner
  display.println(F("Hello - ESP32 here"));
  display.println(F("ESP32+BME280+ADC1115"));
  display.display();

  Serial.println("*** setup ***************************************************");
  
  Wire.begin();
  ScanI2C();

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
  ResetADS_counters(); 
  
  // get average 0 current value
  int i;
  int Average_loop = 3;
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
   Serial.println("---------------------------------------------------------------");

  MultyWiFiBlynkBegin(); //instead Blynk.begin(auth, ssid, pass);
  Serial.println("---------------------------------------------------------------");
  ////connectToWLANandBlynk();
  // Blynk.begin(auth, ssid, pass);
  ////Serial.println("Blynk.begin(auth, ssid1, pass1, IPAddress(10,10,10,112), 8080);");
  ////Blynk.begin(auth, ssid1, pass1, IPAddress(10,10,10,112),8080);
  timer.setInterval(delayTime, myTimerEvent);
  timer1.setInterval(2000L, ReadTemps_DS18B20);    // Temperature sensor read interval.
  terminal.clear();

  // This will print Blynk Software version to the Terminal Widget when
  // your hardware gets connected to Blynk Server
  terminal.println(F("Blynk v" BLYNK_VERSION ": Device started"));
  terminal.print("WiFi: <" + String(ssid[wifi_ssid])); terminal.println(">");
  terminal.flush();

  display.display();
  
  Serial.println("-----TEMP DS18B20 setup -------------------------------------------------");
  Setup_DS18B20();
  
  
  Serial.println("*** end setup ***************************************************");

  digitalWrite(Led_Red, LOW);digitalWrite(Led_Yellow, LOW);digitalWrite(Led_Green, LOW);
}

void loop() {
    if ( WiFi.status() != WL_CONNECTED )
    {
      Serial.println ( "loop: No WiFi - Resetting in 10 seconds...\n\n\n\n\n" );
      delay ( 10000 );  
      ESP.restart();
    } 

  Blynk.run();
  timer.run(); // Initiates BlynkTimer
  timer1.run(); // Initiates BlynkTimer for Temp DS18B20 reading
  ReadADS();
}

// preveri tudi opcijo z Blynk connect + timer 
// https://community.blynk.cc/t/solved-how-to-run-blynk-run-only-when-wifi-connection-is-established/6492/13


// Osnovni sekundni timer
void myTimerEvent() {
  Blynk_PrintValues();
}
  
void Blynk_PrintValues() {
  timex=millis();
  float temp=RoundX(bme.readTemperature(),1);
  float press=RoundX(bme.readPressure() / 100.0F,0);
  float humi=RoundX(bme.readHumidity(),0);
  Serial.print("BME280 exec: "); Serial.print(millis()-timex); Serial.print("ms");
  timex=millis();

  String str = String(temp, 1) + "℃";
  Serial.print("  Temp: ");  Serial.print(str);
  Blynk.virtualWrite(V1, temp);
  
  str = String(press, 0) + " hPa";
  Serial.print("  Press: ");  Serial.print(str);
  Blynk.virtualWrite(V2, press);

//  Serial.print("Approx. Altitude = ");
//  Serial.print(bme.readAltitude(SEALEVELPRESSURE_HPA));
//  Serial.println(" m");

  str = String(humi, 1) + "%";
  Serial.print("  Humi: ");  Serial.print(str);
  Blynk.virtualWrite(V3, humi);

  Serial.print("   P&B: "); Serial.print(millis()-timex); Serial.print("ms");
  Serial.println();
  timex=millis();
  
  float volt0, volt1,volt2,volt3; 

  ReadADS();
  ConvertADSreadings_to_average();
    
  volt0 = adc0*ADC_GAIN/1000.0*(Voltage_divider_R1+Voltage_divider_R2)/Voltage_divider_R2;
  volt1 = adc1*ADC_GAIN/1000.0*1;    //(Voltage_divider_R1+Voltage_divider_R2)/Voltage_divider_R2;
  volt2 = adc2*ADC_GAIN/1000.0; // ACS712 connected
  volt3 = adc3*ADC_GAIN/1000.0; // reference voltage VDD on ADS1115

  //  Serial.print("1.3  ADC0: "); Serial.print(ads.readADC_SingleEnded(0)*ADC_GAIN/1000.0*1); 
  //  Serial.print("  ADC1: "); Serial.print(ads.readADC_SingleEnded(1)*ADC_GAIN/1000.0*1); 
  //  Serial.print("  ADC2: "); Serial.print(ads.readADC_SingleEnded(2)*ADC_GAIN/1000.0*1); 
  //  Serial.print("  ADC3: "); Serial.print(ads.readADC_SingleEnded(3)*ADC_GAIN/1000.0*1); 
  //  Serial.println();

  Serial.print("ADC exec: "); Serial.print(millis()-timex); Serial.print("ms");
  Serial.print("  #read: "); Serial.print(adc_reading_count);
  timex=millis();

  float procVacuum = RoundX(volt1/4.1*100,1);
  Serial.print("  ADC0: ");  Serial.print(RoundX(volt0,1)); Blynk.virtualWrite(V4, RoundX(volt0,1)); Serial.print("V"); 
  Serial.print("  ADC1: ");  Serial.print(RoundX(volt1,1)); Blynk.virtualWrite(V5, RoundX(procVacuum ,1)); //Serial.println("V"); 
  
  float VDD_ADS1115_voltage_value = volt3*2;    //1-1 divider
  Serial.print("  ADC3 (5V): ");  Serial.print(RoundX(VDD_ADS1115_voltage_value,1)); Serial.print("V; ADC2-"); Serial.print(volt2*1000); 
  
  //float current_value = (volt2 - (VDD_ADS1115_voltage_value/2))/ACS712_GAIN ;
  float current_value = 0-((volt2 - ACS712_0Amp)/ACS712_GAIN);     // očitno 0 Amp ni odvisna od vhodne napetosti ampak vedno 2,36V
  Serial.print("  I: ");  Serial.print(RoundX(current_value,2));  Serial.print("A");
  Blynk.virtualWrite(V6, RoundX(current_value,2));

  float power_value = current_value * volt0;
  Serial.print("  PWR: ");  Serial.print(RoundX(power_value,2));  Serial.print("W");
  Blynk.virtualWrite(V7, RoundX(power_value,2));

  terminal.print("U0: "); terminal.print(RoundX(volt0,1)); terminal.print("V; ");
  terminal.print("U1: "); terminal.print(RoundX(volt1,1)); terminal.print("V; ");
  //terminal.print("I: "); terminal.print(RoundX(current_value,2)); terminal.print("A ");
  terminal.print("P: "); terminal.print(RoundX(power_value,2)); terminal.print("W ");
  terminal.print("V%: "); terminal.print(RoundX(procVacuum,1)); terminal.println("");
  
  terminal.flush();

  display.clearDisplay();
  display.setCursor(0,0);
  display.print("U0: "); display.print(RoundX(volt0,1)); display.print("V;");
  display.print(" I: "); display.print(RoundX(current_value,2)); display.println("A;");
  display.print("U1: "); display.print(RoundX(volt1,1)); display.print("V");
  display.print(" p: "); display.print(RoundX(procVacuum,0)); display.println("%");
  display.print("P: "); display.print(RoundX(power_value,2)); display.println("W");
  display.print("Time (sec): ");display.println(RoundX(millis()/1000,0));
  display.display();

  Serial.print("  P&B: "); Serial.print(millis()-timex);Serial.print("ms"); Serial.println();
  Serial.println();

//////////  PressureTrafficLights_ADC1(volt1/4.1*100);    // dano OFF in namseto vacuum TL so za Cooling 16.12.2020
  PressureRelais(volt1/4.1*100);
  ResetADS_counters();
}

float PR_off=40, PR_on=80;
boolean PR_status=false;

//Sider from Blynk This function will be called every time Slider Widget in Blynk app writes values to the Virtual Pin V11
// Vacuum OFF slider
BLYNK_WRITE(V11) {
  int pinValue = param.asInt(); // assigning incoming value from pin V11 to a variable
  // process received value
  PR_off=pinValue;
  terminal.print("PR-off: "); terminal.print(PR_off); terminal.print("  PR-on: "); terminal.println(PR_on); 
  terminal.flush();

  Led_Limit_Yellow = PR_on;
  Led_Limit_Green = PR_off + (PR_on-PR_off)/2.0;
}

// Vacuum ON slider
BLYNK_WRITE(V12) {
  int pinValue = param.asInt(); // assigning incoming value from pin V12 to a variable
  // process received value
  PR_on=pinValue;
  terminal.print("PR-off: "); terminal.print(PR_off); terminal.print("  PR-on: "); terminal.println(PR_on); 
  terminal.flush();

  Led_Limit_Yellow = PR_on;
  Led_Limit_Green = PR_off + (PR_on-PR_off)/2.0;
}

void PressureRelais(float proc) {
    if (proc>PR_on) { digitalWrite(PresRelais, LOW); PR_status=true ; Serial.println("Vacuum Relais-ON"); led1.on(); }    // terminal.println("Realais-ON"); 
    if (proc<PR_off) { digitalWrite(PresRelais, HIGH); PR_status=false; Serial.println("Vacuum Relais-OFF"); led1.off();}  //terminal.println("Realais-OFF"); 
 
}

void PressureTrafficLights_ADC1(float proc) {
  if (proc < Led_Limit_Green) { digitalWrite(Led_Green, HIGH); digitalWrite(Led_Yellow, LOW);  digitalWrite(Led_Red, LOW); led1.setColor(BLYNK_GREEN);}
  if (proc < Led_Limit_Yellow and proc >= Led_Limit_Green) { digitalWrite(Led_Green, LOW);  digitalWrite(Led_Yellow, HIGH); digitalWrite(Led_Red, LOW); led1.setColor(BLYNK_YELLOW);}
  if (proc >= Led_Limit_Yellow) {  digitalWrite(Led_Green, LOW);  digitalWrite(Led_Yellow, LOW);  digitalWrite(Led_Red, HIGH); led1.setColor(BLYNK_RED);}

  Serial.print("Vacuum: ");  Serial.print(RoundX(proc,0));  Blynk.virtualWrite(V5, RoundX(proc,1)); Serial.print("%"); 
  Serial.print("  OFF: ");  Serial.print(RoundX(PR_off,1)); Blynk.virtualWrite(V11, RoundX(PR_off,1)); Serial.print("%"); 
  Serial.print("  ON: ");  Serial.print(RoundX(PR_on,1));   Blynk.virtualWrite(V12, RoundX(PR_on,1)); Serial.println("%"); 
}

void ReadADS() {
  adc0 += ads.readADC_SingleEnded(0); 
  adc1 += ads.readADC_SingleEnded(1);
  adc2 += ads.readADC_SingleEnded(2); 
  adc3 += ads.readADC_SingleEnded(3); 
  adc_reading_count++;
}

void ConvertADSreadings_to_average(){
  adc0 /= adc_reading_count;
  adc1 /= adc_reading_count;
  adc2 /= adc_reading_count;
  adc3 /= adc_reading_count;
}

void ResetADS_counters() {
  adc0=0; adc1=0;adc2=0;adc3=0;
  adc_reading_count=0;
}

float RoundX(float val, byte dec) {
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

void ReadTemps_DS18B20() {
  sensors.requestTemperatures(); // Send the command to get temperatures

  // Loop through each device, print out temperature data
  for(int i=0;i<numberOfDevices; i++){
    // Search the wire for address
    if(sensors.getAddress(tempDeviceAddress, i)){
      // Output the device ID
      Serial.print("Temperature for device: ");
      Serial.print(i,DEC);
      // Print the data
      float tempC = sensors.getTempC(tempDeviceAddress);
      RawValue[i] = tempC;
      CorrectedValue[i] = (((RawValue[i] - RawLow[i]) * RefRange[i]) / RawRange[i]) + RefLow[i];
      Serial.print("   Temp C: ");
      Serial.println(CorrectedValue[i]);
      //Blynk.virtualWrite(15+i, CorrectedValue[i]);
      switch (i) {
         case 0:
            Blynk.virtualWrite(V15, RoundX(CorrectedValue[i],1));
            CoolingRelais(CorrectedValue[i]);
            CoolingTrafficLights(CorrectedValue[i]);
            break;
         case 1:
            Blynk.virtualWrite(V16, RoundX(CorrectedValue[i],1));
            break;
        case 2:
            Blynk.virtualWrite(V17, RoundX(CorrectedValue[i],1));
            break;
        default:
         // if nothing else matches, do the default
         break;
      }
    }
  }
}

// function to print a device address
void printAddress(DeviceAddress deviceAddress) {
  for (uint8_t i = 0; i < 8; i++){
    if (deviceAddress[i] < 16) Serial.print("0");
      Serial.print(deviceAddress[i], HEX);
  }
}

// cooling relais
float Cool_off=8.1, Cool_on=29.1;       // set some default values
boolean Cool_status=false;

//Sider from Blynk - This function will be called every time Slider Widget in Blynk app writes values to the Virtual Pin V1
// Slider Cooling ON
BLYNK_WRITE(V18) {
  int pinValue = param.asInt(); // assigning incoming value from pin V11 to a variable
  // process received value
  Cool_off=pinValue;
  terminal.print("Cool-off: "); terminal.print(Cool_off); terminal.print("  Cool-on: "); terminal.println(Cool_on); 
  terminal.flush();

  Cool_Led_Limit_Yellow = Cool_on;
  Cool_Led_Limit_Green = Cool_off + (Cool_on-Cool_off)/2.0;
}

// Slider Cooling OFF
BLYNK_WRITE(V19) {
  int pinValue = param.asInt(); // assigning incoming value from pin V12 to a variable
  // process received value
  Cool_on=pinValue;
  terminal.print("Cool-off: "); terminal.print(Cool_off); terminal.print("  Cool-on: "); terminal.println(Cool_on); 
  terminal.flush();

  Cool_Led_Limit_Yellow = Cool_on;
  Cool_Led_Limit_Green = Cool_off + (Cool_on-Cool_off)/2.0;
}

void CoolingRelais(float proc) {
  Serial.print(" ....CoolingRelais: "); Serial.println(proc);
    if (proc>Cool_on) { digitalWrite(CoolRelais, LOW); Cool_status=true ; Serial.println("Cooling Relais-ON"); led_cool.on(); }    // terminal.println("Realais-ON"); 
    if (proc<Cool_off) { digitalWrite(CoolRelais, HIGH); Cool_status=false; Serial.println("Cooling Relais-OFF"); led_cool.off();}  //terminal.println("Realais-OFF"); 
}


void CoolingTrafficLights(float proc) {
  if (proc < Cool_Led_Limit_Green) { digitalWrite(Led_Green, HIGH); digitalWrite(Led_Yellow, LOW);  digitalWrite(Led_Red, LOW); led_cool.setColor(BLYNK_GREEN);}
  if (proc < Cool_Led_Limit_Yellow and proc >= Cool_Led_Limit_Green) { digitalWrite(Led_Green, LOW);  digitalWrite(Led_Yellow, HIGH); digitalWrite(Led_Red, LOW); led_cool.setColor(BLYNK_YELLOW);}
  if (proc >= Cool_Led_Limit_Yellow) {  digitalWrite(Led_Green, LOW);  digitalWrite(Led_Yellow, LOW);  digitalWrite(Led_Red, HIGH); led_cool.setColor(BLYNK_RED);}

  Serial.print("Cooling: ");  Serial.print(RoundX(proc,1));  Blynk.virtualWrite(V15, RoundX(proc,1)); Serial.print("C"); 
  Serial.print("  OFF: ");  Serial.print(RoundX(Cool_off,1)); Blynk.virtualWrite(V18, RoundX(Cool_off,1)); Serial.print("C"); 
  Serial.print("  ON: ");  Serial.print(RoundX(Cool_on,1));   Blynk.virtualWrite(V19, RoundX(Cool_on,1)); Serial.println("C"); 
}
