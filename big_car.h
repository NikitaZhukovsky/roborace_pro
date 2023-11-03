#include <Servo.h>

#define SENSOR_AMOUNT 4

#define SERVO_PIN 10
#define MOTOR_PIN 6
#define TAHO_PIN 2

#define NEUTRAL_POINT 90
#define MIN_POINT 40
#define MAX_POINT 140 // 130 start

#define NEUTRAL_SPEED 90
#define MIN_SPEED 94 // was 100
#define MAX_SPEED 106 // was 200
#define MIN_BSPEED 85

#define IR_PIN1 A7
#define IR_PIN2 A6
#define IR_PIN3 A1
#define IR_PIN4 A0

#define MOTOR_P 100
#define MOTOR_D 110

class Car {
  private:
  public:
    Servo steer_servo;
    Servo motor;
    float target_speed;
    float last_error;

    const int sensor_amount = SENSOR_AMOUNT;
    void write_speed(int s);
    void write_speed_ms(float s);
    void pid_control_motor();
    void write_steer(int s);
    int* read_sensors();
    void init();
};

volatile unsigned long last_turnover = 0, turnover = 0, turnover_time = 0, cur_time = 0;
void taho_interrupt() {
  turnover =  (long)(micros() - last_turnover);
  if (turnover > 20000) {
    turnover_time = turnover;
    last_turnover = micros();
  }
}

const float pi = 3.141592653589;

float get_speed() {
  cur_time = (long)(micros() - last_turnover);
  cur_time = max(cur_time, turnover_time);
  if (cur_time > turnover_time * 3) return 0;
  return 2.0 * pi / ((float)cur_time / 1000000) * 0.027 / 10;
  //return turnover_time;
}

void Car::init() {

  steer_servo.attach(SERVO_PIN);
  motor.attach(MOTOR_PIN);
  pinMode(12, OUTPUT);
  pinMode(2, INPUT);
  attachInterrupt(0, taho_interrupt, RISING);

  write_steer(0);
  write_speed(0);
}

void Car::pid_control_motor() {
  float error = target_speed - get_speed();
  int control = round(error * MOTOR_P + (error - last_error) * MOTOR_D);
  write_speed(constrain(control, 0, 100)); // set low to 1 to disable braking
  last_error = error;
}

void Car::write_speed_ms(float s) {
  target_speed = s;
}

void Car::write_speed(int s) {
  s = constrain(s, -1000, 1000);
  if (s > 0) {
    s = map(s, 1, 1000, MIN_SPEED, MAX_SPEED);
  } else if (s < 0)  {
    s = map(s, -1000, -1, 0, MIN_BSPEED);
  } else {
    s = NEUTRAL_SPEED;
  }
  motor.write(s);
}

const int SAMPLE_NUM = 5;

int compare_int (const void * a, const void * b) {
  if ( *(int*)a < * (int*)b ) return -1;
  if ( *(int*)a == *(int*)b ) return 0;
  if ( *(int*)a > *(int*)b ) return 1;
  return 0;
}

float read_sensor(int Sensor, int model) {
  int raw[SAMPLE_NUM];
  for (int i = 0; i < SAMPLE_NUM; i++) {
    raw[i] = analogRead(Sensor);
  }
  qsort(raw, SAMPLE_NUM, sizeof(int), compare_int);

  float volts = (raw[SAMPLE_NUM / 2]) * 0.0048828125;

  if (model == 20150) {
    float val = 60.374 * pow(volts, -1.16);
    //return val;
    return constrain(val, 20.0, 150.0) * 10.0;
  } else if (model == 1080) {
    float val = 29.988 * pow(volts, -1.173);
    return constrain(val, 10.0, 80.0) * 10.0;
  }

  return 0;
}

int* Car::read_sensors() {
  static int values[SENSOR_AMOUNT];

  values[0] = read_sensor(IR_PIN1, 20150);
  values[1] = read_sensor(IR_PIN2, 20150);
  values[2] = read_sensor(IR_PIN3, 20150);
  values[3] = read_sensor(IR_PIN4, 20150);

  return values;
}

void Car::write_steer(int s) {
  s = s * -1;
  s = constrain(s, -1000, 1000);

  if (s < 0) {
    s = map(s, -1000, 0, MIN_POINT, NEUTRAL_POINT);
  } else {
    s = map(s, 0, 1000, NEUTRAL_POINT, MAX_POINT);
  }
  steer_servo.write(s);
}
