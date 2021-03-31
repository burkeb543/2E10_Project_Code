#include <PID_v1.h>
#include <WiFiNINA.h>
char ssid[] = "VMBF82558"; // Add WiFi Name
char pass[] = "Nju6af8cwhbh";  // Add WiFi password

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

WiFiServer server(80); // Creating Server object
 
double Setpoint, Input, Output;
double Kp=13, Ki=0.115, Kd=0;//Kp higher than 10, Ki needs to be very small ,Kd keep as 0 because Kd is a derivative and will only work for steady signals so the rate of error is always changing (human error), ki= 1% of kp or smaller 
PID myPID(&Input, &Output, &Setpoint, Kp, Ki, Kd, REVERSE); //IF CLOSER= SMALL VALUEES
 
void setup()
{
 Setpoint = 20;//at 20cm the buggy is going to stop
 myPID.SetMode(AUTOMATIC);//turn the PID on 
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
  pinMode(PMWL, OUTPUT);
  pinMode(PMWR, OUTPUT);
  pinMode(PMWR, HIGH);
  pinMode(PMWL, HIGH);
  
  obstacleInFront = false; // No obstacle in front
  WiFi.begin(ssid, pass); // Initialize the WiFi library's network settings
  IPAddress ip = WiFi.localIP(); // IP Address of arduino
  Serial.print("IP Address:"); 
  Serial.println(ip);
  server.begin(); // Tells server to listen for incoming connections
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
 
 WiFiClient client = server.available(); // Waiting for a client to connect
  if (client.connected()) {
    char c = client.read();
if (c == 'z'){ // If start signal received from client
      callDrive(); 
      keepDriving = true; // Buggy has been started
    }
if (c == 's'){ // If Stop signal received from client
      S();
      keepDriving = false; // Buggy has been stopped
    }
 }
else{  // If client briefly disconnects
    if (keepDriving){  // Keep driving if previously driving
      callDrive(); 
      }
    }
  if(obstacleInFront){ 
    server.write('o'); // Send the char 'o' to the Processor PC to signal an obstacle in front of the buggy
  }
}


void Drive() { // Dictates motion of buggy
  
   Input = distance;
   myPID.Compute();//now output has value
   //Serial.println(Output);
    
  LEYE_Status = digitalRead( LEYE ); // Store current status of Left Eye
  REYE_Status = digitalRead( REYE ); // Store current status of Right Eye

    if( !LEYE_Status ){ // Check if Left IR sensor detects white
      analogWrite(PMWL, Output);
       }else{ // If black detected stop the Left Motor
        LMS();
      }
  
     if( !REYE_Status ){ //Check if Right IR sensor detects white
       analogWrite(PMWR, Output);
       }else{ // If black detected stop the Rigth Motor
      RMS();
      }

    if (obstacleInFront) {
      Serial.println(" Obstacle has been cleared");
      Serial.println("");
      obstacleInFront = false; // Set to false as obstacle no longer in front of buggy
    }
    else{ // If an object obstructs the way stop the buggy
       S();
      if (!obstacleInFront){ // Check if obstacle was in front during previous code loop
      Serial.println(" An obstacle is detected"); // Print if this is a new obstacle
      obstacleInFront = true; // As obstacle in front of buggy set to true
    }
  }
}

void S() { // Stop Buggy
  RMS();
  LMS();
}


void RMS() { // Right Motor Stop
  pinMode( RM1, LOW);
  pinMode( RM2, LOW);
}



void LMS() { // Left Motor Stop
  pinMode( LM2, LOW);
  pinMode( LM1, LOW);
}

void callDrive(){ // Calls Drive() multiple times to ensure fast response 
  for(int n = 0; n < 50; n++){
    Drive();
  }
}


