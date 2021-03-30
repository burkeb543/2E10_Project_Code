

#include <PID_v1.h>
#include <WiFiNINA.h>
//char ssid[] = "VMBF82558"; // Add WiFi Name
//char pass[] = "Nju6af8cwhbh";  // Add WiFi password

const int REYE = 10; // Right IR Sensor digital pin 
const int LEYE = 9;  // Left IR Sensor digital pin

const int LM1 = 2; // Left Motor Logic Pin 1
const int LM2 = 3; // Left Motor Logic Pin 2
const int RM1 = 5; // Right Motor Logic Pin 1
const int RM2 = 6; // Right Motor Logic Pin 2

const int echoPin = 15; // Echo Pin for Ultrasonic
const int trigPin = 14; // Trig Pin for Ultrasonic

int LEYE_Status = digitalRead( LEYE ); // Current status of Left IR Sensor
int REYE_Status = digitalRead( REYE ); // Current status of Right IR Sensor

const int PMWR = A2;
const int PMWL = A5;

long duration; // Length of time for ultrasonic ping to return
int distance; // Distance from buggy to obstacle

bool keepDriving = false; // True if buggy receives instruction to start
bool obstacleInFront; // True if obstacle within 10cm detected

//WiFiServer server(80); // Creating Server object
 
double Setpoint, Input, Output;
double Kp=13, Ki=0.115, Kd=0;//Kp higher than 10, Ki needs to be very small ,Kd keep as 0 because Kd is a derivative and will only work for steady signals so the rate of error is always changing (human error), ki= 1% of kp or smaller 
PID myPID(&Input, &Output, &Setpoint, Kp, Ki, Kd, REVERSE); //IF CLOSER= SMALL VALUEES
 
void setup()
{
 //initialize the variables we're linked to
 Setpoint = 20;//at 15cm the buggy is going to stop
 
 //turn the PID on
 myPID.SetMode(AUTOMATIC);
 myPID.SetOutputLimits(0,255);

 Serial.begin(9600);
  pinMode( LM2, OUTPUT ); // Set Left Motor Logic pin 2 as output
  pinMode( LM1, OUTPUT ); // Set Left Motor Logic pin 1 as output
  pinMode( RM1, OUTPUT ); // Set Right Motor Logic pin 1 as output
  pinMode( RM2, OUTPUT ); // Set Right Motor Logic pin 2 as output
  pinMode( LEYE, INPUT ); // Set Left IR Sensor as input
  pinMode( REYE, INPUT ); // Set Right IR Sensor as input
  pinMode( LM2, HIGH); // Set Left Motor Logic pin 2 to low
  pinMode( LM1, LOW); // Set Left Motor Logic pin 1 to low
  pinMode( RM1, LOW); // Set Right Motor Logic pin 1 to low
  pinMode( RM2, HIGH); // Set Right Motor Logic pin 2 to low
  pinMode(trigPin, OUTPUT); // trigPin OUTPUT
  pinMode(echoPin, INPUT); // echoPin  INPUT
  obstacleInFront = false; // No obstacle in front

  pinMode(PMWL, OUTPUT );
  pinMode(PMWR, OUTPUT );
  pinMode(PMWR, HIGH);
  pinMode(PMWL, HIGH);
  //WiFi.begin(ssid, pass); // Initialize the WiFi library's network settings
  //IPAddress ip = WiFi.localIP(); // IP Address of arduino
  //Serial.print("IP Address:"); 
  //Serial.println(ip);
 // server.begin(); // Tells server to listen for incoming connections
}
 
void loop()
{
 digitalWrite(trigPin, LOW); 
  delayMicroseconds(2);
  digitalWrite(trigPin, HIGH);
  delayMicroseconds(10);
  digitalWrite(trigPin, LOW);

 duration = pulseIn(echoPin, HIGH);
 distance = (duration-10) * 0.034 / 2;
 
 Input = distance;
 myPID.Compute();//now output has value
 analogWrite(PMWL, Output);
 analogWrite(PMWR, Output);

 Serial.println(Output);
 
}
