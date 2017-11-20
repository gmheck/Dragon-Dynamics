/* 
 * This is the program header put things like the date, your name
 * And a description of what your program does here
 * 
 * Michal Britt-Crane 7/6/2015
 * 
 * This program moves a servo to two different positions  
 * 
 * Challange: use button two to 5 degrees when pressed  ----------
 */  

#include <Servo.h>             // import the servo library, this allows us to use a bunch of additional code to make it really easy to control a servo
Servo myServo;                 // create "Servo" object called "myServo" to control a servo

// Constants
const int redLED = 6;          // red LED on pin 10
const int buttonPin = 7;        // button connected to pin 7
const int servoPin = 5;         // servo attached to pin 5
const int potPin = A0;          // potentiometer attached to pin A0 (analog input #0)

const int minCoastTime =  1000; // minimum cost time =  1 second
const int maxCoastTime = 20000; // minimum cost time = 15 seconds
const int riseDriveTime = 12000;// the time to drive for in the rise direction

void setup() {                        // start setup, this code runs once at startup
  pinMode(redLED, OUTPUT);            // initialize digital pin "redLED" as an output so you can turn the LED on and off
  pinMode(buttonPin, INPUT_PULLUP);   // initialize buttonPin as an input and turn on the arduino's internal pull-up resistor on buttonPin 
  Serial.begin(9600);                 // start serial port
}                                     // end setup 

void loop() {                             // start main loop  
  dive();
  delay(readPot());
  rise();
  delay(readPot());
}                                         // end main method, repeat to top

void dive(){
  digitalWrite(redLED, 1);              // turn on the red LED
  Serial.println("Diving till button is pressed. ");
  myServo.attach(servoPin);             // attaches the servo on pin "servoPin" to the servo object. servo must be attached to send it a command. 
  myServo.write(0);                   // drive the servo to 175 deg
  while(digitalRead(buttonPin) == true){
    // just wait... keep driving until the button (limmit switch) is pressed
  }                                       // end of while loop
  myServo.detach();                       // attaches the servo on pin "servoPin" to the servo object. servo must be attached to send it a command. 
  digitalWrite(redLED, 0);                // turn off the red LED
}

void rise(){
  Serial.print("Rising for ");
  Serial.print(riseDriveTime/1000);
  Serial.println(" secconds. ");
  myServo.attach(servoPin);               // attaches the servo on pin "servoPin" to the servo object. servo must be attached to send it a command. 
  myServo.write(180);                       // drive the servo to 175 deg
  delay(riseDriveTime);                   // 
  myServo.detach();                       // attaches the servo on pin "servoPin" to the servo object. servo must be attached to send it a command. 
}

int readPot(){
  int potValue = analogRead(potPin);
  int pauseTime = map(potValue, 0, 1023, minCoastTime, maxCoastTime);
  Serial.print("Coasting for ");
  Serial.print(pauseTime/1000);
  Serial.println(" secconds. ");
  return pauseTime;
}

