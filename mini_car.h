#include <Servo.h>
#include <Wire.h>
#include <VL53L0X.h>

#define BUTTON_PIN 12
#define SHARP_PIN A0
#define XSHUT_1 7
#define XSHUT_2 6
#define XSHUT_3 4

#define SENSOR_AMOUNT 5 // 3 IR and button and sharp

#define SERVO_PIN 5

#define NEUTRAL_POINT 100
#define MIN_POINT 65
#define MAX_POINT 120

#define INA 9
#define INB 10
#define PWM 11

const int SAMPLE_NUM = 5;

int compare_int (const void * a, const void * b){
  if ( *(int*)a < *(int*)b ) return -1;
  if ( *(int*)a == *(int*)b ) return 0;
  if ( *(int*)a > *(int*)b ) return 1;
}

float read_sensor(int Sensor, int model) {
  int raw[SAMPLE_NUM];
  for (int i = 0; i < SAMPLE_NUM; i++){
    raw[i] = analogRead(Sensor);
  }
  qsort(raw, SAMPLE_NUM, sizeof(int), compare_int);

  float volts = (raw[SAMPLE_NUM / 2] + 0.5) * 5.0 / 1024.0;

  if (model == 20150) {
    float val = 60.374 * pow(volts, -1.16);       // было 60.374
    return constrain(val, 20.0, 250.0) * 10.0;
  } else if (model == 1080) {
    float val = 29.988 * pow(volts, -1.173);
    return constrain(val, 10.0, 80.0) * 10.0;
  }
}

class Car {
  private:
    Servo steer_servo;
    VL53L0X Sensor1;
    VL53L0X Sensor2;
    VL53L0X Sensor3;

 
  public:
    int ss;
    const int sensor_amount = SENSOR_AMOUNT;
    void write_speed(int s);
    void write_steer(int s);
    bool timeout_occurred();
    int* read_sensors();
    void init();
};

bool Car::timeout_occurred(){
  return Sensor1.timeoutOccurred() or Sensor2.timeoutOccurred() or Sensor3.timeoutOccurred();
}

void set_high_distance(VL53L0X sensor) {
  //sensor.setSignalRateLimit(0.1);
  //sensor.setVcselPulsePeriod(VL53L0X::VcselPeriodPreRange, 18);
  //sensor.setVcselPulsePeriod(VL53L0X::VcselPeriodFinalRange, 14);
  //sensor.setMeasurementTimingBudget(200000);
  sensor.setMeasurementTimingBudget(30000);
} 
  

void Car::init(){
  pinMode(BUTTON_PIN, INPUT_PULLUP);
  
  steer_servo.attach(SERVO_PIN);
  pinMode(INA, OUTPUT);
  pinMode(INB, OUTPUT);

  pinMode(SHARP_PIN, INPUT);

  pinMode(XSHUT_1, OUTPUT);
  pinMode(XSHUT_2, OUTPUT);
  pinMode(XSHUT_3, OUTPUT);  
  digitalWrite(XSHUT_1, LOW);
  digitalWrite(XSHUT_2, LOW);
  digitalWrite(XSHUT_3, LOW);
  delay(10);
 
  Wire.begin();
  pinMode(XSHUT_1, INPUT); // XSHUT isn't 5v tolerant
  delay(10); // Time for sensor boot
  Sensor1.setAddress(41+1);

  pinMode(XSHUT_2, INPUT);
  delay(10);
  Sensor2.setAddress(41+2);

  pinMode(XSHUT_3, INPUT);
  delay(10);
  Sensor3.setAddress(41+3);

  Sensor1.setTimeout(500);
  Sensor2.setTimeout(500);
  Sensor3.setTimeout(500);

  
  Sensor1.init();
  Sensor2.init();
  Sensor3.init();

  
  set_high_distance(Sensor1);
  set_high_distance(Sensor2);
  set_high_distance(Sensor3);
  
  Sensor1.startContinuous(20);
  Sensor2.startContinuous(20);
  Sensor3.startContinuous(20);
 
  write_steer(0);
  write_speed(0);


}

void Car::write_speed(int s){
  s = constrain(s, -1000, 1000);
  ss = s;
  s = map(s, -1000, 1000, -255, 255);
  if (s > 0){
    digitalWrite(INA, HIGH);
    digitalWrite(INB, LOW);
  } else {
    digitalWrite(INA, LOW);
    digitalWrite(INB, HIGH);
  }
  analogWrite(PWM, abs(s));
}

int* Car::read_sensors(){
  static int values[SENSOR_AMOUNT];
  values[0] = Sensor1.readRangeContinuousMillimeters();
  values[1] = Sensor2.readRangeContinuousMillimeters();
  values[2] = Sensor3.readRangeContinuousMillimeters();
  values[3] = digitalRead(BUTTON_PIN);

  values[0] = constrain(values[0], 0, 1000);
  values[1] = constrain(values[1], 0, 1000);
  values[2] = constrain(values[2], 0, 1000);
  values[4] = read_sensor(SHARP_PIN, 20150);
  
  return values;
}

void Car::write_steer(int s){
  s = s * -1;
  s = constrain(s, -1000, 1000);
  
  if (s < 0){
    s = map(s, -1000, 0, MIN_POINT, NEUTRAL_POINT);
  } else {
    s = map(s, 0, 1000, NEUTRAL_POINT, MAX_POINT);
  }
  steer_servo.write(s);
}
