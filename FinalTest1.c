#include "TinyGPS++.h"
#include <Wire.h>
#include <LSM303.h>
#include <LiquidCrystal_I2C.h>
#include <Servo.h>
#include <ctype.h>
#include <string.h>

float GetLatLon(double DEST_LAT, double DEST_LON);
void Drive(double DEST_LAT, double DEST_LON);
void ChangeDirection(int direction, double DEST_LAT, double DEST_LON);
void DriveForward(int duration, double DEST_LAT, double DEST_LON);
void GetDistance(double DEST_LAT, double DEST_LON);
float GetBearing();
int GetSonicDistance();
void SpiralSearch();
void MoveAroundObject();
static void smartDelay(unsigned long ms, double DEST_LAT, double DEST_LON);

double DEST_LAT1 = 37.85312271118, DEST_LON1 = -107.7709655761;
double DEST_LAT2 = 39.85312271118, DEST_LON2 = -104.7709655761;
double DEST_LAT3 = 39.85312271118, DEST_LON3 = -104.7709655761;
static const char* cardinals[] = {"N", "NNE", "NE", "ENE", "E", "ESE", "SE", "SSE", "S", "SSW", "SW", "WSW", "W", "WNW", "NW", "NNW"};

LiquidCrystal_I2C lcd(0x27, 16, 2);
LSM303 compass;
TinyGPSPlus gps; // create gps object
Servo myservo, myservoSteer;

const char *gpsdirection;
double latitude;
double longitude;
float FinalBearing;
int distanceInch;
float Distance_M;
const int trigPin = 10;
const int echoPin = 11;
float courseTo;

void setup() 
{
  Wire.begin();

pinMode(trigPin, OUTPUT); 
pinMode(echoPin, INPUT);
Serial.begin(9600); // connect serial
Serial1.begin(9600); // connect serial
Serial.println("The GPS Received Signal:");

lcd.init();
delay(500);
lcd.backlight();
delay(500);
compass.m_min = (LSM303::vector<int16_t>) {-1753,  -1835,  -2256};
compass.m_max = (LSM303::vector<int16_t>) {+4774,  +4686,  +4169};
myservo.attach(9);
myservoSteer.attach(12);
}

void loop()
{
while(!gps.location.isValid())
{
lcd.clear();
lcd.print("Getting Location");
lcd.clear();
lcd.print("Please wait");
  smartDelay(1000);
}
lcd.print("loc_0");
delay(500);
//Drive(DEST_LAT1, DEST_LON1);
lcd.clear();
lcd.print("loc_1");
delay(1000);
//Drive(DEST_LAT2, DEST_LON2);
lcd.clear();
lcd.print("loc_2");
delay(1000);
//Drive(DEST_LAT3, DEST_LON3);
lcd.print("loc_3");
delay(1000);
SpiralSearch();
myservo.write(90);
delay(100000000);
}

void GetDistance(double DEST_LAT, double DEST_LON)
{
Distance_M = TinyGPSPlus::distanceBetween(latitude, longitude, DEST_LAT, DEST_LON);
}

float GetBearing()
{
Serial.println("gets bearing");
compass.init();
compass.enableDefault();
compass.read();
int heading = compass.heading();
lcd.setCursor(0, 0);
heading = heading + 40;
if (heading > 360)
{
  heading = heading - 360;
}
if (heading < 0)
{
  heading = 360 + heading;
}
Serial.print("heading is :");
Serial.print(heading);
return heading;
}

static void smartDelay(unsigned long ms, double DEST_LAT, double DEST_LON)
{
  unsigned long start = millis();
  do 
  {
    while (Serial1.available())
    {
    gps.encode(Serial1.read());
    courseTo = TinyGPSPlus::courseTo(latitude, longitude, DEST_LAT, DEST_LON);
    gpsdirection = TinyGPSPlus::cardinal(courseTo); 
    }
  } while (millis() - start < ms);
}

float GetLatLon(double DEST_LAT, double DEST_LON)
{
    Serial.println("gets lat lon");
    smartDelay(1000, DEST_LAT, DEST_LON);
    latitude = gps.location.lat();
    longitude = gps.location.lng();
    
    Serial.println(courseTo);
    Serial.println(latitude, 11);
    delay(500);
    Serial.println(longitude, 11);
    delay(500);
    lcd.clear();
    lcd.setCursor(0, 0);
    lcd.print(latitude, 11);
    lcd.setCursor(0, 1);
    lcd.print(longitude, 11);
    return courseTo;
}

int GetSonicDistance()
{
float duration;
digitalWrite(trigPin, LOW);
delayMicroseconds(2);
digitalWrite(trigPin, HIGH);
delayMicroseconds(10);
digitalWrite(trigPin, LOW);
duration = pulseIn(echoPin, HIGH);
distanceInch = (duration * 0.3343)/4/10;
Serial.print("distanceinch: ");
Serial.println(distanceInch);
  delay(500);
return distanceInch;
}

void MoveAroundObject()
{
while(GetSonicDistance() <= 24)
{
  Serial.print("moving around object");
  myservo.write(90);
  delay(500);
  myservo.write(0);
  delay(500);
  myservoSteer.write(0);
  delay(500);
  myservo.write(103);
  delay(1500);
  myservoSteer.write(90);
}
Serial.print("finished moving around object");
}   

void SpiralSearch() //needs work, this might spin in a circle but not a spiral, we will have to test the turning angle
{
  while(GetSonicDistance() > 5)
  {
    myservo.write(105);
    delay(500);
    myservoSteer.write(0);
    delay(1000);
    myservo.write(105);
    delay(500);
    myservoSteer.write(90);
    delay(500);
  }
}

void ChangeDirection(int direction, double DEST_LAT, double DEST_LON) // 0 for left 1 for right, ie 0 renders 0 for write and 1 renders 180
{
  int turnradius;
  Serial.println("gets into change direction");
  delay(500);
  if(GetSonicDistance() <= 24)
  {
    MoveAroundObject();
  }
  else
  {
    if(direction  == 1)
    {
      turnradius = 180;
    }
    else 
    {
      turnradius = 0;
    }
    myservoSteer.write(turnradius);
    delay(500);
    myservo.write(103);
    delay(1000);
  }
  FinalBearing = GetBearing();
  GetLatLon(DEST_LAT, DEST_LON);
}

void DriveForward(int duration, double DEST_LAT, double DEST_LON)
{
  Serial.println("gets into drive forward");
  delay(500);
  if(GetSonicDistance() <= 24)
  {
    MoveAroundObject();
  }
  else 
  {
    myservoSteer.write(90);
    delay(500);
    myservo.write(103);
    delay(duration);
  }
  FinalBearing = GetBearing();
  GetLatLon(DEST_LAT, DEST_LON);
}

void Drive(double DEST_LAT, double DEST_LON)
{
  FinalBearing = GetBearing();
  delay(500);
  GetDistance(DEST_LAT, DEST_LON);
  delay(500);
  GetLatLon(DEST_LAT, DEST_LON);
  
  delay(500);
  Serial.println("gets into drive");
  delay(500);
  Serial.println(gpsdirection);
  delay(500);
     
      while (Distance_M > 0) //(DEST_LAT - latitude !=0 && DEST_LON - longitude !=0)
      {
        GetDistance(DEST_LAT, DEST_LON);
        
        while (strcmp(gpsdirection, cardinals[0]) == 0 | strcmp(gpsdirection, cardinals[1]) == 0 | strcmp(gpsdirection, cardinals[2]) == 0  |
         strcmp(gpsdirection, cardinals[14]) == 0  | strcmp(gpsdirection, cardinals[15]) == 0)
        {
         Serial.println("gets into north steer");
         if (FinalBearing > 30 && FinalBearing < 340)
         {
          ChangeDirection(1, DEST_LAT, DEST_LON);
        }
        else
        {
          DriveForward(1000, DEST_LAT, DEST_LON);
        }
        myservo.write(90);
        delay(2000);
      }
      while (strcmp(gpsdirection, cardinals[6]) == 0 | strcmp(gpsdirection, cardinals[7]) == 0 | strcmp(gpsdirection, cardinals[8]) == 0  |
       strcmp(gpsdirection, cardinals[9]) == 0  | strcmp(gpsdirection, cardinals[10]) == 0 )
      {
        Serial.println("gets into south steer");
        if(FinalBearing > 315 | FinalBearing < 290)
        {
         ChangeDirection(1, DEST_LAT, DEST_LON);
       }
       else
       {
         DriveForward(1000, DEST_LAT, DEST_LON);
       }
       myservo.write(90);
        delay(2000);
     }
     while (strcmp(gpsdirection, cardinals[3]) == 0 | strcmp(gpsdirection, cardinals[4]) == 0 | strcmp(gpsdirection, cardinals[5]) == 0 )
     {
      Serial.println("gets into east steer");
      if (FinalBearing > 80 | FinalBearing < 35)
      {
        ChangeDirection(1, DEST_LAT, DEST_LON);
      }
      else
      {
        DriveForward(1000, DEST_LAT, DEST_LON);
      }
      myservo.write(90);
      delay(2000);
    }
    while (strcmp(gpsdirection, cardinals[11]) == 0 | strcmp(gpsdirection, cardinals[12]) == 0 | strcmp(gpsdirection, cardinals[13]) == 0 )
    {
      Serial.println("gets into west steer");
      if(FinalBearing > 350 | FinalBearing < 320)
      {
        ChangeDirection(1, DEST_LAT, DEST_LON);
      }
      else
      {
        DriveForward(1000, DEST_LAT, DEST_LON);
      }
      myservo.write(90);
        delay(2000);
    }
  }
}
