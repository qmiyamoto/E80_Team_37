#include <Arduino.h>
#include <Wire.h>
#include <avr/interrupt.h>

int hallSensor = 14; //connect hall effect sensor output to the corresponding pin (A0)
int revolution = 0; //initialize revolution count

double endTime = 0; //time of last rotation
double currentTime = 0; //Current Rotation Time
double timeDifference;
double sampleTime = 1000; //how long to count number of rotations for one sample 

constexpr int N = 5; //number of samples to average across for cleanliness
double sampleSet[N];
int sampleIndex = 0;

double RPS; //number of rotations per second

const double anemometerConst = 20; // Anemometer constant (from wind tunnel) to change RPS to m/s
double windSpeed;


void setup() {
  Serial.begin(9600);
  pinMode(hallSensor, INPUT_PULLUP);
  //when hallSensor pin goes from HIGH to LOW, call the ISR function
  attachInterrupt(digitalPinToInterrupt(hallSensor), ISR, FALLING);
}

void loop() {
  currentTime = millis();
  float timeDifference = currentTime - endTime; // Basic Loop Setup

  if (timeDifference > sampleTime) { 
    // Code to get the RPS of the anemometer
    RPS = (revolution / sampleTime) * 1000;

    // Store the RPS value in the history array (for running average)
    sampleSet[sampleIndex] = RPS;
    sampleIndex = (sampleIndex + 1) % N; // Loop back to the start when the array is full

    endTime = currentTime;
    revolution = 0; // Reset revolution count for next sample set
  }
  // Calculate running average
  double avgRPS = calculateAverage(sampleSet, N);
  windSpeed = avgRPS * anemometerConst;

  // Serial Monitor
  Serial.print("Average RPS (Hz): ");
  Serial.println(avgRPS);
  Serial.print("Wind Speed (m/s): ");
  Serial.println(windSpeed);

  // Wait for 1 second before printing again
  delay(1000);
}

//ISR function: increments revolution count and prints to serial monitor
void ISR(){ 
  revolution += 1;
  Serial.print("Revolution count: ");
  Serial.println(revolution);
}

/*
 * Calculates a running average of a sample set
 *
 * @param sampleSet The sample set
 * @param N The number of samples
 * @return The running average
 */
 double calculateAverage(double sampleSet[], int N) {
    double sum = 0.0;

    for (int i = 0; i < N; i++) {
        sum += sampleSet[i];
    }

    return sum / N;
}

// double runningAverage(double average, double newSample) {
//     average -= average / N;
//     average += newSample / N;

//     return average;
// }
