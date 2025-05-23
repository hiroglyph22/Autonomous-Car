#include <Servo.h>

#include <ECE3.h>

uint16_t sensorValues[8];
//uint16_t summed_values[8];

int current_position = -40;
int increment_position = 4;

int number_samples = 5;
bool print_directions = true;
int error = 0;
int prev_error = 0;
int base_speed = 15;
int max_error = 3000;
float Kp = 0.003;
float Kd = 0;
int steering_correction = 0;
int Kd_correction = 0;

const int left_nslp_pin=31; // nslp HIGH ==> awake & ready for PWM
const int right_nslp_pin=11; // nslp HIGH ==> awake & ready for PWM
const int left_dir_pin=29;
const int right_dir_pin=30;
const int left_pwm_pin=40;
const int right_pwm_pin=39;

const int LED_RF = 75;

//hehe red amogus sus :)
void setup() {
  ECE3_Init();
  Serial.begin(9600);  // set the data rate in bits per second for serial data transmission
  
  pinMode(left_nslp_pin,OUTPUT);
  pinMode(left_dir_pin,OUTPUT);
  pinMode(left_pwm_pin,OUTPUT);
  pinMode(right_nslp_pin,OUTPUT);
  pinMode(right_dir_pin,OUTPUT);
  pinMode(right_pwm_pin,OUTPUT);

  pinMode(LED_RF, OUTPUT);

  digitalWrite(left_nslp_pin,HIGH);
  digitalWrite(right_nslp_pin,HIGH);
  // digitalWrite(left_nslp_pin,LOW);
  // digitalWrite(right_nslp_pin,LOW);
  
  digitalWrite(left_dir_pin,LOW);  // Set car direction to forward
  digitalWrite(right_dir_pin,LOW);

  resetEncoderCount_left();
  resetEncoderCount_right();

  Kd = Kp * 4;

  delay(2000);
}

void loop() {

  //dummy = Serial.readString();

  // Read raw sensor values
  ECE3_read_IR(sensorValues);

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

  for (unsigned char i = 0; i < 8; i++) {
    summed_values[i] /= number_samples;
  }

  summed_values[0] -= 1007;
  summed_values[1] -= 934;
  summed_values[2] -= 938;
  summed_values[3] -= 965;
  summed_values[4] -= 979;
  summed_values[5] -= 1077;
  summed_values[6] -= 1031;
  summed_values[7] -= 1195;

  // Serial.println(summed_values[0]);
  // Serial.println(summed_values[1]);
  // Serial.println(summed_values[2]);
  // Serial.println(summed_values[3]);
  // Serial.println(summed_values[4]);
  // Serial.println(summed_values[5]);
  // Serial.println(summed_values[6]);
  // Serial.println(summed_values[7]);
  // Serial.println();
  
  for (unsigned char i = 0; i < 8; i++) {
    summed_values[i] *= 1000;
  }

  summed_values[0] /= 1493;
  summed_values[1] /= 1566;
  summed_values[2] /= 1562;
  summed_values[3] /= 1535;
  summed_values[4] /= 1521;
  summed_values[5] /= 1423;
  summed_values[6] /= 1469;
  summed_values[7] /= 1305;

  error = (summed_values[0] * -40 + summed_values[1] * -20 + summed_values[2] * -12 + summed_values[3] * -8 + summed_values[4] * 8 + summed_values[5] * 12 + summed_values[6] * 20 + summed_values[7] * 40) / 8;

  //error = (summed_values[0] * -15 + summed_values[1] * -14 + summed_values[2] * -12 + summed_values[3] * -8 + summed_values[4] * 8 + summed_values[5] * 12 + summed_values[6] * 14 + summed_values[7] * 15) / 8;
  // Serial.println(summed_values[0]);
  // Serial.println(summed_values[1]);
  // Serial.println(summed_values[2]);
  // Serial.println(summed_values[3]);
  // Serial.println(summed_values[4]);
  // Serial.println(summed_values[5]);
  // Serial.println(summed_values[6]);
  // Serial.println(summed_values[7]);
  // Serial.println();
  // delay(500);
  // Serial.print(summed_values[0] + " " + summed_values[1] + " " + summed_values[2] + " " + summed_values[3] + " " + summed_values[4] + " " + summed_values[5] + " " + summed_values[6] + " " + summed_values[7]);
  // Serial.println();
  int sum1 = 0;
  int sum2 = 0;
  for (int i = 0; i <= 7; i++) {
    if (summed_values[i] > 0) {
      sum1 += summed_values[i];
    }
    if (sum1 > 750 && (i+2) <= 7) {
      for (int j = i + 2; j <= 7; j++) {
        if (summed_values[j] > 0) {
          sum2 += summed_values[j];
        }
      }
    }
    if (sum2 > 750) {
      Serial.println("Split detected.");
    }
  }


  steering_correction = -Kp * error;

  Kd_correction = Kd * (error - prev_error);  

  // Serial.println(steering_correction);
  // Serial.println();
  
  // ChangeWheelSpeeds(0, steering_correction, 0, -steering_correction);

  int total_correction = steering_correction + Kd_correction;
  


  if (total_correction > base_speed) {
    digitalWrite(right_dir_pin,HIGH);
    analogWrite(right_pwm_pin, total_correction - base_speed);
    //analogWrite(left_pwm_pin, total_correction - base_speed);
    //analogWrite(right_pwm_pin, 50);
    // Serial.print("Spinning Right Backwards: ");
    // Serial.println(total_correction - base_speed);
  }
  else if (-total_correction > base_speed ) {
    digitalWrite(left_dir_pin, HIGH);
    analogWrite(left_pwm_pin,(-total_correction - base_speed));
    //analogWrite(right_pwm_pin, total_correction - base_speed);
    //analogWrite(left_pwm_pin, 50);
    // Serial.print("Spinning Left Backwards: ");
    // Serial.println(-total_correction - base_speed);

  }
  else {
    digitalWrite(right_dir_pin, LOW);
    digitalWrite(left_dir_pin, LOW);
    analogWrite(left_pwm_pin,base_speed + steering_correction + Kd_correction);
    analogWrite(right_pwm_pin,base_speed  - steering_correction - Kd_correction);
    // analogWrite(right_pwm_pin,base_speed);
    // analogWrite(left_pwm_pin,base_speed);
    // Serial.println("test");
  }


  // // // Print average values (average value = summed_values / number_samples
  // // for (unsigned char i = 0; i < 8; i++) {
  // //   Serial.print(summed_values[i] / number_samples);
  // //   Serial.print('\t');  // tab to format the raw data into columns in the Serial monitor
  // // }
  // // Serial.println();
  
  prev_error = error;

}
