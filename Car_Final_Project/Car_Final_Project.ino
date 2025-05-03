#include <Servo.h>

#include <ECE3.h>
String dummy;

uint16_t sensorValues[8];

int current_position = -40;
int increment_position = 4;

int number_samples = 5;
bool print_directions = true;

//hehe red amogus sus :)
void setup() {
  ECE3_Init();
  Serial.begin(9600);  // set the data rate in bits per second for serial data transmission
}

void loop() {

  // hile (Serial.available() == 0) {}
  dummy = Serial.readString();
  int summed_values[8] = { 0 };

  // Take the average of 5 consecutive values for each sensor
  for (int j = 0; j < number_samples; j++) {
    // Read raw sensor values
    ECE3_read_IR(sensorValues);

    // Add the current sensor values using a for loop
    for (unsigned char i = 0; i < 8; i++) {
      summed_values[i] += sensorValues[i];
    }
  }

  // Print average values (average value = summed_values / number_samples
  for (unsigned char i = 0; i < 8; i++) {
    Serial.print(summed_values[i] / number_samples);
    Serial.print('\t');  // tab to format the raw data into columns in the Serial monitor
  }
  Serial.println();
}
