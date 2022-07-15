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

//DHT-11
#include <Adafruit_Sensor.h>
#include <DHT.h>
#define DHT11PIN D4
#define DHTTYPE DHT11
DHT dht11(DHT11PIN,DHTTYPE);  
float flttemperature, flthumidity;
String temperature, humidity;

// MQ-135
#include <MQ135.h>
// MQ-135 calibration
// RZero: 3.4 | Corrected RZero: 3.8 (rounded off) (28.90 degrees / 79.00 humidity)
#define RZERO 3.8
// 1k ohm resistor of MQ-135
#define RLOAD 1.0
MQ135 mq135(ANALOGPIN, RZERO, RLOAD);
float fltCO2;
String CO2;

// MQ-2
#include <MQ2.h>
MQ2 mq2(ANALOGPIN);
String LPG, CO, Smoke; 

//DSM501A
#define DSM501APIN D5
byte buff[2];
unsigned long durationPM25;
unsigned long starttime;
unsigned long endtime;
unsigned long sampletime_ms = 30000;
unsigned long lowpulseoccupancyPM25 = 0;

//Multiplexer control pins
int s0 = D0;
int s1 = D1;
int s2 = D2;
int s3 = D3;

//Multiplexer in "SIG" pin
int SIG_pin = ANALOGPIN;

void setup(){
  // DSM501A
  pinMode(DSM501APIN,INPUT);
  starttime = millis(); 

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

  // Connect to wifi
  WiFi.begin(WIFI_SSID, WIFI_PASSWORD);
  Serial.print("Connecting");
  while (WiFi.status() != WL_CONNECTED) {
    Serial.print(".");
    delay(500);
  }
  Serial.println();
  Serial.print("Connected: ");
  Serial.println(WiFi.localIP());

  // Necessary to avoid NaN readings
  dht11.begin();          

  mq2.begin();
  
  timeClient.begin();
  // GMT-8
  timeClient.setTimeOffset(28800);

  Firebase.begin(FIREBASE_HOST, FIREBASE_AUTH);
  delay(20000);
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

   // DHT11
   delay(1000);
   flttemperature = dht11.readTemperature();
   flthumidity = dht11.readHumidity();
   temperature = String(flttemperature);
   humidity = String(flthumidity);

   Serial.println("Temperature: " + temperature);
   Serial.println("Humidity: " + humidity); 
                  
   if(!isnan(flttemperature)){
       String firebaseTemperature = "/DHT11/Temperature/" + date + "/" + time + "/";
       Firebase.pushString(firebaseTemperature,temperature);        
   }             
   if(!isnan(flthumidity)){
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

      fltCO2 = mq135.getCorrectedPPM(flttemperature, flthumidity);
      CO2 = String(fltCO2);
      
      Serial.print("CO2:");
      Serial.print(CO2);
      Serial.println("ppm");
      
      String firebaseCO2 = "/MQ135/CO2/" + date + "/" + time + "/";

      Firebase.pushString(firebaseCO2, CO2);
    }
    
    // Channel 1
    if (i == 1){
      //MQ-2
      delay(1000);
      Serial.print("Value at channel 1 is: ");
      Serial.println(readMux(1));
      
      float* values= mq2.read(true);
      
      LPG = String(mq2.readLPG());
      CO = String(mq2.readCO());
      Smoke = String(mq2.readSmoke());
      
      String firebaseLPG = "/MQ2/LPG/" + date + "/" + time + "/";
      String firebaseCO = "/MQ2/CO/" + date + "/" + time + "/";
      String firebaseSmoke = "/MQ2/Smoke/" + date + "/" + time + "/";
      
      Firebase.pushString(firebaseLPG, LPG);
      Firebase.pushString(firebaseCO, CO);
      Firebase.pushString(firebaseSmoke, Smoke);
    }
  }
  delay(30000);
  // DSM501A
  durationPM25 = pulseIn(DSM501APIN, LOW);
  lowpulseoccupancyPM25 += durationPM25;
  endtime = millis();
  // Calculate the ratio only after 30s has passed
  if ((endtime-starttime) > sampletime_ms) 
  {
    float conPM25 = calculateConcentration(lowpulseoccupancyPM25,30);
    Serial.print("PM25: ");
    Serial.println(conPM25);
    
    String PM25 = String(conPM25);
    lowpulseoccupancyPM25 = 0;
    starttime = millis();
    String firebasePM25 = "/DSM501A/PM25/" +  date + "/" + time + "/";
    
    Firebase.pushString(firebasePM25,PM25);
  }
  delay(30000);
}

// Read multiplexed input
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
  //Calculate the ratio
  float ratio = (lowpulseInMicroSeconds/1000000.0)/30.0*100.0; 
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
