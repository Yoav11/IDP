#include <ultrasound_utils.h>

/*
* Ultrasonic Sensor HC-SR04 and Arduino Tutorial
*
* by Dejan Nedelkovski,
* www.HowToMechatronics.com
*
*/

// defines pins numbers
//const int trigPin = 9;
//const int echoPin = 10;



void ultrasound_setup(int trigPin, int echoPin) {
  pinMode(trigPin, OUTPUT); // Sets the trigPin as an Output
  pinMode(echoPin, INPUT); // Sets the echoPin as an Input

}


/*
This function returns the distance from the sensor to an object in cm. To use this function, to the following
-----
delay(1000/20); // delay in mm
Serial.println(get_distance(trigPin, echoPin));
-----
*/
int get_distance(int trigPin, int echoPin) {
  // defines variables
  long duration;
  int distance;

  // Clears the trigPin
  digitalWrite(trigPin, LOW);
  delayMicroseconds(2);
  // Sets the trigPin on HIGH state for 10 micro seconds
  digitalWrite(trigPin, HIGH);
  delayMicroseconds(10);
  digitalWrite(trigPin, LOW);
  // Reads the echoPin, returns the sound wave travel time in microseconds
  duration = pulseIn(echoPin, HIGH);
  // Calculating the distance
  distance= duration*0.034/2;
  return distance;
}

// Test print the distance onto serial at a specified frequency. (f < 40 for the sensor)
void test_distance_sensor(int trigPin, int echoPin, int frequency) {
  delay(1000/frequency); // delay in mm
  Serial.println(get_distance(trigPin, echoPin));
}
