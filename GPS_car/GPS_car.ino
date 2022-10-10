//Libraries
#include <PWMServo.h>
#include <TinyGPS++.h>
#include <SoftwareSerial.h>

//Variables
const int RXPin = 4, TXPin = 3;
const uint32_t GPSBaud = 9600; //Default baud of NEO-6M is 9600
const int motorPin = 11;
const int motorPwmVal = 255;

double currentLat;
double currentLon;
double distanceToTarget;
int targetCourse;
int currentCourse;

bool A7_state = false;


//Library Objects
PWMServo steerServo;
TinyGPSPlus gps;
SoftwareSerial gpsSerial(RXPin, TXPin);

//Target Coordinates
const double targetLat = -31.98017;
const double targetLon = 115.81783;

//Runs Once
void setup() {
  Serial.begin(9600);
  gpsSerial.begin(GPSBaud);
  steerServo.attach(SERVO_PIN_A); //pin 9 for arduino nano (handled by library)

  pinMode(A7, INPUT);
}

//Runs Infinitely
void loop()
{

  if (analogRead(A7) > 1)
  {
    A7_state = HIGH;
  }
  else
  {
    A7_state = LOW;
    analogWrite(motorPin, 0);
  }

  if (A7_state == HIGH)
  {
    startMoving();
  }

  else
  {
    stopMoving();
  }

  if (gpsSerial.available() > 0) {
    if (gps.encode(gpsSerial.read())) {
      if (gps.location.isValid()) {
        currentLat = gps.location.lat();
        currentLon = gps.location.lng();
        distanceToTarget = TinyGPSPlus::distanceBetween(currentLat, currentLon, targetLat, targetLon);
        targetCourse = TinyGPSPlus::courseTo(gps.location.lat(), gps.location.lng(), targetLat, targetLon);
        currentCourse = gps.course.deg();

        if (currentCourse < targetCourse)
        {
          turnSlightLeft();
          delay(500);
          goStraight();
        }

        else if (currentCourse > targetCourse)
        {
          turnSlightRight();
          delay(500);
          goStraight();
        }

        else if (abs((currentCourse - targetCourse)) < 10)
        {
          goStraight();
        }

        else if (distanceToTarget < 5)
        {
          stopMoving();
        }
        //        Serial.print("target course: ");
        //        Serial.print(targetCourse);
        //        Serial.print(" current course: ");
        //        Serial.println(currentCourse);

        Serial.print("course to target: ");
        Serial.print(TinyGPSPlus::courseTo(gps.location.lat(), gps.location.lng(), targetLat, targetLon));
        Serial.print(F(" current course: "));
        Serial.println(gps.course.deg());
      }
      else
      {
        Serial.println(F("- location: INVALID"));
      }
    }
  }
}

void goStraight()
{
  steerServo.write(90);
  //Serial.println("Go straight");
}

void turnSlightLeft()
{
  steerServo.write(80);
  Serial.println("Turn Left SLightly");
}

void turnSlightRight()
{
  steerServo.write(100);
  Serial.println("Turn Right Slightly");
}

void turnLeft()
{
  steerServo.write(70);
  Serial.println("Turn Left");
}

void turnRight()
{
  steerServo.write(110);
  Serial.println("Turn Right");
}

void startMoving()
{
  analogWrite(motorPin, motorPwmVal);
  Serial.println("Start Moving");
}

void stopMoving()
{
  analogWrite(motorPin, 0);
  Serial.println("Stop Moving");
}
