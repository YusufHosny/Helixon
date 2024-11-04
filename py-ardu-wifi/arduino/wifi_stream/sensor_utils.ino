Adafruit_BNO055 bno = Adafruit_BNO055(55, 0x28);
Adafruit_BMP3XX bmp;

void setup_bno() {
  if(!bno.begin())
  {
    Serial.print("No BNO055 detected");
    while(true);
  }

  bno.setExtCrystalUse(true);
}

imu::Quaternion getQuaternion() {
  imu::Quaternion q = bno.getQuat();
  
  return q;
}

void setup_bmp() {
  if (!bmp.begin_I2C()) {
    Serial.println("Could not find a valid BMP3 sensor, check wiring!");
    while (1);
  }

  // Set up oversampling and filter initialization
  bmp.setTemperatureOversampling(BMP3_OVERSAMPLING_8X);
  bmp.setPressureOversampling(BMP3_OVERSAMPLING_4X);
  bmp.setIIRFilterCoeff(BMP3_IIR_FILTER_COEFF_3);
  bmp.setOutputDataRate(BMP3_ODR_50_HZ);
}

void fillData(DataEntry *data) {
  sensors_event_t event;
  data->microsT = micros();
  bno.getEvent(&event, Adafruit_BNO055::VECTOR_ACCELEROMETER);
  data->accelx = event.acceleration.x;
  data->accely = event.acceleration.y;
  data->accelz = event.acceleration.z;
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
  data->tempbmp = bmp.temperature;
  data->pressure = bmp.pressure;

}


const int fs = 20; // sampling freq
const int ts = 1000/fs; // sample time in ms
const double _acc_thres = 12;
const unsigned int _swipeDelay = 500; // ms delay until next swipe
long unsigned int _swipeTprev;
char _swipeState = 'i'; // '^' going up, 'v': going down, 'i': idle, 'u' up swipe, 'd' swipe
long unsigned int _stateT = 0;
char getSwipe() {
  // check if enough time has passed
  if(millis() - _swipeTprev < ts) return 'i';
  _swipeTprev = millis();

  sensors_event_t event;
  bno.getEvent(&event, Adafruit_BNO055::VECTOR_LINEARACCEL);

  // get accel data
  double x, y, z;
  x = event.acceleration.x;
  y = event.acceleration.y;
  z = event.acceleration.z;
  imu::Vector<3> raw_acc = {x, y, z};

  // rotate to earth coordinate frame
  imu::Quaternion q = bno.getQuat();
  imu::Vector<3> abs_acc = q.rotateVector(raw_acc); // rotated into earth frame coordinate space (Gravity on z axis)

  // Serial.print("x: "); Serial.print(abs_acc.x());
  // Serial.print("y: "); Serial.print(abs_acc.y());
  // Serial.print("z: "); Serial.println(abs_acc.z());

  // FSM update
  switch(_swipeState) {
    case '^':
      if(millis() - _stateT > _swipeDelay) {
        // up swipe done
        _swipeState = 'i';
        return 'u';
      }
      else return 'i';

    case 'v':
      if(millis() - _stateT > _swipeDelay) {
        // down swipe done      
        _swipeState = 'i';
        return 'd';
      }
      else return 'i';

    case 'i':
      if(abs_acc.z() > _acc_thres) {
        _swipeState = '^';
      }
      else if(abs_acc.z() < -_acc_thres) {
        _swipeState = 'v';
      }
      _stateT = millis();
      return 'i';
    }

  return 'i';

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