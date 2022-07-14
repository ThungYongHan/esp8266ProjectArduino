#include <ESP8266WiFi.h>
#include <FirebaseArduino.h>
#include <NTPClient.h>
#include <WiFiUdp.h>

// Firebase and Wi-Fi connection
#define FIREBASE_HOST "group-project-esp8266-default-rtdb.asia-southeast1.firebasedatabase.app"
#define FIREBASE_AUTH "NKH6PeDAccfhdILOG7kcZgVvXOyjWzmm7YVDfvpQ"
#define WIFI_SSID "POCO X3 Pro"
#define WIFI_PASSWORD "yonghanpswd"

#define ANALOGPIN A0

// MQ-135
#include <MQ135.h>
// MQ-135 calibration
// RZero: 3.4 | Corrected RZero: 3.8 (rounded off) (28.90 degrees / 79.00 humidity)
#define RZERO 3.8
// 1k ohm resistor of MQ-135
#define RLOAD 1.0

// MQ-2
#include <MQ2.h>

//DHT-11
#include <Adafruit_Sensor.h>
#include <DHT.h>
#define DHTTYPE DHT11
#define DHTPIN D4

//DSM501A
byte buff[2];
unsigned long durationPM25;
unsigned long starttime;
unsigned long endtime;
unsigned long sampletime_ms = 30000;
unsigned long lowpulseoccupancyPM25 = 0;

// MQ-135
MQ135 mq135_sensor(ANALOGPIN, RZERO, RLOAD);

// MQ-2
int LPG, CO, Smoke;
int mq2_Pin = ANALOGPIN;
MQ2 mq2(mq2_Pin);

// DHT-11
DHT dht(DHTPIN,DHTTYPE);  
float flttemperature;
float flthumidity;
String temperature;
String humidity;

// Internet date and time
WiFiUDP ntpUDP;
NTPClient timeClient(ntpUDP, "pool.ntp.org");

//Multiplexer control pins
int s0 = D0;
int s1 = D1;
int s2 = D2;
int s3 = D3;

//Multiplexer in "SIG" pin
int SIG_pin = ANALOGPIN;

void setup(){
  // DSM501A
  pinMode(D5,INPUT);
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
  Serial.print("connecting");
  while (WiFi.status() != WL_CONNECTED) {
    Serial.print(".");
    delay(500);
  }
  Serial.println();
  Serial.print("connected: ");
  Serial.println(WiFi.localIP());

  // Necessary to avoid NaN readings
  dht.begin();          
  
  timeClient.begin();
  // GMT-8
  timeClient.setTimeOffset(28800);

  Firebase.begin(FIREBASE_HOST, FIREBASE_AUTH);
  // remove
//  delay(20000);
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
   flttemperature = dht.readTemperature();
   flthumidity = dht.readHumidity();
   
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

      float correctedPPM = mq135_sensor.getCorrectedPPM(flttemperature, flthumidity);
      Serial.print("CO2:");
      Serial.print(correctedPPM);
      Serial.println("ppm");
      
      String CO2 = String(correctedPPM);
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
      LPG = mq2.readLPG();
      CO = mq2.readCO();
      Smoke = mq2.readSmoke();
      
      String LPG = String(mq2.readLPG());
      String CO = String(mq2.readCO());
      String Smoke = String(mq2.readSmoke());
      
      String firebaseLPG = "/MQ2/LPG/" + date + "/" + time + "/";
      String firebaseCO = "/MQ2/CO/" + date + "/" + time + "/";
      String firebaseSmoke = "/MQ2/Smoke/" + date + "/" + time + "/";
      
//      Serial.println("LPG Value: " + LPG);
//      Serial.println("CO Value: " + CO);
//      Serial.println("Smoke Value: " + Smoke);
      
      Firebase.pushString(firebaseLPG, LPG);
      Firebase.pushString(firebaseCO, CO);
      Firebase.pushString(firebaseSmoke, Smoke);
    }
  }

  // DSM501A
  durationPM25 = pulseIn(D5, LOW);
  lowpulseoccupancyPM25 += durationPM25;
  endtime = millis();
  // Calculate the ratio only after 30s has passed
  if ((endtime-starttime) > sampletime_ms) 
  {
    float conPM25 = calculateConcentration(lowpulseoccupancyPM25,30);
    Serial.print("PM25: ");
    Serial.println(conPM25);
    
    String firePM25 = String(conPM25);
    lowpulseoccupancyPM25 = 0;
    starttime = millis();
    String pm25 = "/DSM501A/PM25/" +  date + "/" + time + "/";
    
    Firebase.pushString(pm25,firePM25);
  }
  // change                    
//  delay(60000);
   delay(10000);
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

  //Return value
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
  String firelowpulse = String(lowpulseInMicroSeconds);
  Serial.print("    ratio:");
  Serial.print(ratio);
  String fireratio = String (ratio);
  Serial.print("    Concentration:");
  Serial.println(concentration);
  String firecon = String (concentration);
  return concentration;
}
