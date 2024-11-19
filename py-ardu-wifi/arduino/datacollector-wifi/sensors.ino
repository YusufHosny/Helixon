#include "sensors.h"

void setup_bno() {
  if(!bno.begin())
  {
    Serial.print("No BNO055 detected");
    while(true);
  }

  bno.setExtCrystalUse(true);
}

void setup_bmp() {
  if (!bmp.begin_I2C()) {
    Serial.println("Could not find a valid BMP3 sensor, check wiring!");
    while (1);
  }

  // Set up oversampling and filter initialization
  bmp.setTemperatureOversampling(BMP3_OVERSAMPLING_32X);
  bmp.setPressureOversampling(BMP3_OVERSAMPLING_32X);
  bmp.setIIRFilterCoeff(BMP3_IIR_FILTER_COEFF_3);
  bmp.setOutputDataRate(BMP3_ODR_200_HZ);
}

void fillData(DataEntry *data) {
  sensors_event_t event;
  data->microsT = micros();
  bno.getEvent(&event, Adafruit_BNO055::VECTOR_ACCELEROMETER);
  data->accelx = event.acceleration.x;
  data->accely = event.acceleration.y;
  data->accelz = event.acceleration.z;
  bno.getEvent(&event, Adafruit_BNO055::VECTOR_LINEARACCEL);
  data->linaccelx = event.acceleration.x;
  data->linaccely = event.acceleration.y;
  data->linaccelz = event.acceleration.z;
  bno.getEvent(&event, Adafruit_BNO055::VECTOR_GYROSCOPE);
  data->gyrox = event.gyro.x;
  data->gyroy = event.gyro.y;
  data->gyroz = event.gyro.z;
  bno.getEvent(&event, Adafruit_BNO055::VECTOR_MAGNETOMETER);
  data->magnx = event.magnetic.x;
  data->magny = event.magnetic.y;
  data->magnz = event.magnetic.z;
  bno.getEvent(&event, Adafruit_BNO055::VECTOR_EULER);
  data->roll = event.orientation.roll;
  data->pitch = event.orientation.pitch;
  data->yaw = event.orientation.heading;
  data->tempbno = bno.getTemp();
  bmp.performReading();
  data->tempbmp = bmp.temperature;
  data->pressure = bmp.pressure;
}


void printEvent(sensors_event_t* event) {
  Serial.println();
  Serial.print(event->type);
  double x = -1000000, y = -1000000 , z = -1000000; //dumb values, easy to spot problem
  if (event->type == SENSOR_TYPE_ACCELEROMETER) {
    x = event->acceleration.x;
    y = event->acceleration.y;
    z = event->acceleration.z;
  }
  else if (event->type == SENSOR_TYPE_ORIENTATION) {
    x = event->orientation.x;
    y = event->orientation.y;
    z = event->orientation.z;
  }
  else if (event->type == SENSOR_TYPE_MAGNETIC_FIELD) {
    x = event->magnetic.x;
    y = event->magnetic.y;
    z = event->magnetic.z;
  }
  else if ((event->type == SENSOR_TYPE_GYROSCOPE) || (event->type == SENSOR_TYPE_ROTATION_VECTOR)) {
    x = event->gyro.x;
    y = event->gyro.y;
    z = event->gyro.z;
  }

  Serial.print(": x= ");
  Serial.print(x);
  Serial.print(" | y= ");
  Serial.print(y);
  Serial.print(" | z= ");
  Serial.println(z);
}