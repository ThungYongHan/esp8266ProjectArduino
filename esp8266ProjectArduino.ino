// Wi-Fi and Firebase connection
#include <ESP8266WiFi.h>
#include <FirebaseArduino.h>
#define WIFI_SSID "POCO X3 Pro"
#define WIFI_PASSWORD "yonghanpswd"
#define FIREBASE_HOST "group-project-esp8266-default-rtdb.asia-southeast1.firebasedatabase.app"
#define FIREBASE_AUTH "NKH6PeDAccfhdILOG7kcZgVvXOyjWzmm7YVDfvpQ"

// Internet date and time
#include <WiFiUdp.h>
#include <NTPClient.h>
WiFiUDP ntpUDP;
NTPClient timeClient(ntpUDP, "pool.ntp.org");

#define ANALOGPIN A0

// MQ-135
#include <MQUnifiedsensor.h>
#define placa "Arduino UNO"
#define Voltage_Resolution 5
#define ADC_Bit_Resolution 10 
//RS / R0 = 3.6 ppm 
#define RatioMQ135CleanAir 3.6 
MQUnifiedsensor MQ135(placa, Voltage_Resolution, ADC_Bit_Resolution, ANALOGPIN, "MQ-135");
float fltAlcohol, fltAcetone;
String alcohol, acetone;

// MQ-2
#include "MQ2.h"
MQ2 mq2(ANALOGPIN);
String LPG, CO;

//DHT-11
#include <Adafruit_Sensor.h>
#include <DHT.h>
#define DHT11PIN D4
#define DHTTYPE DHT11
DHT dht11(DHT11PIN,DHTTYPE);  
float fltTemperature, fltHumidity;
String temperature, humidity;

//DSM501A
#define DSM501APIN D5
byte buff[2];
unsigned long durationPM25;
unsigned long starttime;
unsigned long endtime;
unsigned long sampletime_ms = 30000;
unsigned long lowpulseoccupancyPM25 = 0;
float fltTimeDifference;

//Multiplexer control pins
int s0 = D0;
int s1 = D1;
int s2 = D2;
int s3 = D3;

//Multiplexer in "SIG" pin
int SIG_pin = ANALOGPIN;

void setup(){
  // Multiplexer digital pins
  pinMode(s0, OUTPUT); 
  pinMode(s1, OUTPUT); 
  pinMode(s2, OUTPUT); 
  pinMode(s3, OUTPUT); 
  digitalWrite(s0, LOW);
  digitalWrite(s1, LOW);
  digitalWrite(s2, LOW);
  digitalWrite(s3, LOW);
  
  Serial.begin(9600);

  // Connect to Wi-Fi
  WiFi.begin(WIFI_SSID, WIFI_PASSWORD);
  Serial.print("Connecting");
  while (WiFi.status() != WL_CONNECTED) {
    Serial.print(".");
    delay(500);
  }
  Serial.println();
  Serial.print("Connected: ");
  Serial.println(WiFi.localIP());

  // Setup pin and set pull timings to avoid NaN readings
  dht11.begin();          

  // GMT-8 timezone
  timeClient.begin();
  timeClient.setTimeOffset(28800);

  Firebase.begin(FIREBASE_HOST, FIREBASE_AUTH);

  // DSM501A
  pinMode(DSM501APIN,INPUT);
  starttime = millis(); 
  
  // Calibrate MQ-2 sensor
  Serial.println(readMux(1));
  Serial.println("Calibrating MQ-2");
  mq2.begin();
  Serial.println("MQ-2 calibration complete");

  // DHT11
  delay(1000);
  fltTemperature = dht11.readTemperature();
  fltHumidity = dht11.readHumidity();

  // Calibrate MQ-135 sensor
  Serial.println(readMux(0));
  MQ135.setRegressionMethod(1); 
  MQ135.init(); 
  MQ135.setRL(1);
  Serial.println("Calibrating MQ-135");
  float calcR0 = 0;
  for(int i = 1; i<=10; i ++)
  {
    // Update data, the arduino will read the voltage from the analog pin
    MQ135.update(); 
    calcR0 += MQ135.calibrate(RatioMQ135CleanAir);
  }
  MQ135.setR0(calcR0/10);
  Serial.println("MQ-135 calibration complete");

  if(isinf(calcR0)) {
    Serial.println("Warning: Conection issue, R0 is infinite (Open circuit detected) please check your wiring and supply"); 
    while(1);
    }
  if(calcR0 == 0){
    Serial.println("Warning: Conection issue found, R0 is zero (Analog pin shorts to ground) please check your wiring and supply"); 
    while(1);
    }
}

void loop(){
  // Get latest Internet date and time
  timeClient.update();
  time_t epochTime = timeClient.getEpochTime();
  struct tm *ptm = gmtime ((time_t *)&epochTime);
  int monthDay = ptm->tm_mday;
  int currentMonth = ptm->tm_mon+1;
  int currentYear = ptm->tm_year+1900;
  String date = String(currentYear) + "-" + String(currentMonth) + "-" + String(monthDay);
  String time = timeClient.getFormattedTime();

  if (date != "1970-1-1"){
   // DHT11
   delay(1000);
   fltTemperature = dht11.readTemperature();
   fltHumidity = dht11.readHumidity();
   temperature = String(fltTemperature);
   humidity = String(fltHumidity);

   Serial.println("Temperature: " + temperature);
   Serial.println("Humidity: " + humidity); 
                  
   if(!isnan(fltTemperature)){
       String firebaseTemperature = "/DHT11/Temperature/" + date + "/" + time + "/";
       Firebase.pushString(firebaseTemperature,temperature);        
   }             
   if(!isnan(fltHumidity)){
      String firebaseHumidity = "/DHT11/Humidity/" + date + "/" + time + "/";
      Firebase.pushString(firebaseHumidity, humidity);  
   }

  for(int i = 0; i < 2; i ++){
    // Channel 0
    if (i == 0){
      //MQ-135
      delay(1000);
      Serial.print("Value at channel 0 is: ");
      Serial.println(readMux(0));
      MQ135.update();

      // Calibration curves provided by MQUnifiedSensor.h library example
      MQ135.setA(77.255); MQ135.setB(-3.18); 
      fltAlcohol = MQ135.readSensor(); 
    
      MQ135.setA(34.668); MQ135.setB(-3.369); 
      fltAcetone = MQ135.readSensor(); 
      
      // 1 decimal place
      Alcohol = String(fltAlcohol, 1);
      Acetone = String(fltAcetone, 1);
      
      Serial.println("Alcohol: " + alcohol);
      Serial.println("Acetone: " + acetone);

      String firebaseAlcohol = "/MQ135/Alcohol/" + date + "/" + time + "/";
      String firebaseAcetone = "/MQ135/Acetone/" + date + "/" + time + "/";

      Firebase.pushString(firebaseAlcohol, alcohol);
      Firebase.pushString(firebaseAcetone, acetone);
    }
    
    // Channel 1
    if (i == 1){
      //MQ-2
      delay(1000);
      Serial.print("Value at channel 1 is: ");
      Serial.println(readMux(1));
      // Do not print out in prefined manner provided by MQ2.h library example
      float* values= mq2.read(false);

      // 1 decimal place
      LPG = String(values[0], 1);
      CO = String(values[1], 1);
      
      Serial.println("LPG: " + LPG);
      Serial.println("CO: " + CO);
      
      String firebaseLPG = "/MQ2/LPG/" + date + "/" + time + "/";
      String firebaseCO = "/MQ2/CO/" + date + "/" + time + "/";
      
      Firebase.pushString(firebaseLPG, LPG);
      Firebase.pushString(firebaseCO, CO);
    }
  }

  // DSM501A
  durationPM25 = pulseIn(DSM501APIN, LOW);
  lowpulseoccupancyPM25 += durationPM25;

  endtime = millis();
  String starttimeStr = String(starttime);
  String endtimeStr = String(endtime);
  Serial.println("Start time: " + starttimeStr);
  Serial.println("End time: " + endtimeStr);
  String timeDifference = String(endtime-starttime);
  fltTimeDifference = endtime - starttime;
  Serial.println("Time difference: " + timeDifference);
  
  // Calculate the ratio only after 30s has passed
  if ((endtime-starttime) > sampletime_ms) 
  {
    float fltPM25 = calculateConcentration(lowpulseoccupancyPM25,30);
    Serial.print("PM25: ");
    Serial.println(fltPM25);
    
    String PM25 = String(fltPM25);
    lowpulseoccupancyPM25 = 0;
    starttime = millis();
    
    if (fltPM25>0){
      String firebasePM25 = "/DSM501A/PM25/" +  date + "/" + time + "/";
      Firebase.pushString(firebasePM25,PM25);
    }
  }
  else{
    Serial.println("Not enough time!");
  }
  delay(1000);
  }
}

// Switch between and read multiplexed input
int readMux(int channel){
  int controlPin[] = {s0, s1, s2, s3};
  int muxChannel[2][4]={
    {0,0,0,0}, //Channel 0
    {1,0,0,0}, //Channel 1
  };

  //Loop through the 4 sig
  for(int i = 0; i < 4; i ++){
    digitalWrite(controlPin[i], muxChannel[channel][i]);
  }

  //Read value at the SIG pin
  int val = analogRead(SIG_pin);

  //Return value at the SIG pin
  return val;
}

// Calculate DSM501A readings
float calculateConcentration(long lowpulseInMicroSeconds, long durationinSeconds){
// Calculate the ratio
// float ratio = (lowpulseInMicroSeconds/1000000.0)/30.0*100.0; 
  float ratio = (lowpulseInMicroSeconds/1000000.0)/fltTimeDifference*100.0; 
  //Calculate the mg/m3
  float concentration = 0.001915 * pow(ratio,2) + 0.09522 * ratio - 0.04884;
  Serial.print("lowpulseoccupancy:");
  Serial.print(lowpulseInMicroSeconds);
//  String firelowpulse = String(lowpulseInMicroSeconds);
  Serial.print("    ratio:");
  Serial.print(ratio);
//  String fireratio = String (ratio);
  Serial.print("    Concentration:");
  Serial.println(concentration);
//  String firecon = String (concentration);
  return concentration;
}
