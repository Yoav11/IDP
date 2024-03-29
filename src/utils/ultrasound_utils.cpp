#include <ultrasound_utils.h>

// Set up an ultrasound distance sensor
void ultrasound_setup() {
  pinMode(trigPinFront, OUTPUT);
  pinMode(echoPinFront, INPUT);
  pinMode(trigPinLeft, OUTPUT);
  pinMode(echoPinLeft, INPUT);
  pinMode(trigPinBack, OUTPUT);
  pinMode(echoPinBack, INPUT);
}

// This function returns the distance from the sensor to an object in cm. Make sure to run the set up function before use!
int get_distance(int trigPin, int echoPin) {
  delay(25);
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
