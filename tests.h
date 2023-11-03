#include <Wire.h>

/*
 sprintf(data, "a%04db%04dc%04dd%04de%04df%04d", s[0], s[1], s[2], s[3], s[4], s[5]);
  Serial.println();
  Serial.println(data);
  Serial.flush();

  /* delay(3000);
  car.write_speed(-100);
  delay(50);
  car.write_speed(0);
  delay(50);
  car.write_speed(-100);
  delay(1000);

  car.write_speed(0);
  delay(5000); */

void print_sensors(Car car){
  int* sensors = car.read_sensors();
  for (int i = 0; i < car.sensor_amount; i++){
    Serial.print(i);
    Serial.print(": ");
    Serial.println(sensors[i]);
  }
  Serial.println("----------");
}

void wiggle(Car car){
  for (int i = -100; i < 100; i++){
    car.write_steer(i*10);
    delay(5);
  }
  for (int i = 100; i > -100; i--){
    car.write_steer(i*10);
    delay(5);
  }
}

void turn_left(Car car){
  car.write_steer(-1000);
}

void turn_right(Car car){
  car.write_steer(1000);
}

void turn_forward(Car car){
  car.write_steer(0);
}

void max_speed_test(Car car){
  delay(1500);
  // Forward speed up
  for (int i = 0; i < 100; i++){
    car.write_speed(i*10);
    delay(20);
  }
  delay(200);
  for (int i = 100; i > 0; i--){
    car.write_speed(i*10);
    delay(10);
  }

  delay(1500);
  // Backward speed up
  for (int i = 0; i < 100; i++){
    car.write_speed(i*-10);
    delay(10);
  }
  delay(200);
  for (int i = 100; i > 0; i--){
    car.write_speed(i*-10);
    delay(20);
  }
}

void small_speed_test(Car car){
  delay(1500);
  // Forward speed up
  for (int i = 0; i < 100; i++){
    car.write_speed(i*3);
    delay(20);
  }
  delay(200);
  for (int i = 100; i > 0; i--){
    car.write_speed(i*3);
    delay(20);
  }

  delay(1500);
  // Backward speed up
  for (int i = 0; i < 100; i++){
    car.write_speed(i*-3);
    delay(20);
  }
  delay(200);
  for (int i = 100; i > 0; i--){
    car.write_speed(i*-3);
    delay(20);
  }
}

// copypasted from google search "i2c bus scan arduino"
void i2c_bus_scan(){
  byte error, address;
  int nDevices;
 
  Serial.println("Scanning...");
  nDevices = 0;
  for(address = 1; address < 127; address++ )
  {
    // The i2c_scanner uses the return value of
    // the Write.endTransmisstion to see if
    // a device did acknowledge to the address.
    Wire.beginTransmission(address);
    error = Wire.endTransmission();
 
    if (error == 0)
    {
      Serial.print("I2C device found at address 0x");
      if (address<16)
        Serial.print("0");
      Serial.print(address,HEX);
      Serial.println("  !");
 
      nDevices++;
    }
    else if (error==4)
    {
      Serial.print("Unknown error at address 0x");
      if (address<16)
        Serial.print("0");
      Serial.println(address,HEX);
    }    
  }
  if (nDevices == 0)
    Serial.println("No I2C devices found\n");
  else
    Serial.println("done\n");
}
