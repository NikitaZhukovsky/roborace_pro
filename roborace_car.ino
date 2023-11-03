#pragma GCC optimization("Ofast")
#include "big_car.h"
#include "tests.h"
#define LOOP_TIME 40;

Car car;


void go_back() {
  car.write_speed(0);
  while (get_speed() > 0.1) {}
  car.write_speed(-100);
  delay(200);
  car.write_speed(0);
  delay(80);
  car.write_speed(-100);
  delay(700);
  car.write_speed(0);
}
 
void go_back_2() {
  car.write_speed(0);
  while (get_speed() > 0.1) {}
  car.write_speed(-100);
  delay(1000);
  car.write_speed(0);
  delay(80);
  car.write_speed(-100);
  delay(1800);
  car.write_speed(0);
}

void work() {
  int* s = car.read_sensors();

  int front_dist = (s[1] + s[2]) / 2;
  int diff;

  car.pid_control_motor();

  bool f_l = s[1] < 1000;
  bool f_r = s[2] < 1000;

  if ( s[0] > 1200 && s[3] > 1200) {
    diff = 800; //was -800
  } else {
    diff = (s[3] - s[0]);
  }



  int how_clear = f_l + f_r;

  float coef;
  float spd;

  switch (how_clear) {
    case 0:
      coef = 0.4; spd = 2.9;
      break; 
    case 1:
    case 2:
      coef = 0.7; spd = 1.7;
      break;
  }
  car.write_steer(diff * coef);
  car.write_speed_ms(spd);

  bool c_fl = s[1] < 201; // close front left
  bool c_fr = s[2] < 201; // close front right

  bool low_speed = get_speed() < 0.1;
  static int stuck_time = 0;
  if (c_fl or c_fr or low_speed) {
    stuck_time += 1;
  } else {
    stuck_time = 0;
  }

  if (stuck_time > 25) {
    car.write_steer(0);
    go_back();
    car.write_speed_ms(2);
    stuck_time = 0;
  }


  // Тут код для обнаружения разворотов
  static float turns = 0;
  turns +=  diff * get_speed() / -1000;
  turns = constrain(turns, -2700, 15);

  if (turns < -18.0) {
    car.write_speed(0);
    delay(100);
    car.write_steer(1000);
    delay(20);
    go_back_2();
    car.write_steer(-700);
    car.write_speed_ms(2);
    unsigned long strt = millis();
    while ((millis() - strt) < 900) {
      car.pid_control_motor();
    }
    turns = 0;
  }
}


unsigned long start_time = 0;
unsigned long next_loop = 0;


void setup() {
  Serial.begin(9600);
  car.init();
  delay(3700);
}

void loop() {
  unsigned long cur_time = millis();
  if (cur_time > next_loop) {
    next_loop = next_loop + LOOP_TIME;
    next_loop = max(cur_time, next_loop);
    work();

  }

}
