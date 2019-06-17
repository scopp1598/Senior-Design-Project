//#include <SPI.h>
//#include <Wire.h>
//#include <VarSpeedServo.h>
//#include <Adafruit_Sensor.h>
//#include <Adafruit_LSM9DS0.h>

#define LSM9DS0_GYRO_CS 9
#define LSM9DS0_XM_CS 10
#define LSM9DS0_MOSI 11
#define LSM9DS0_MISO 12
#define LSM9DS0_SCLK 13

//Declare the lsm module and pass the address for I2C use
Adafruit_LSM9DS0 lsm = Adafruit_LSM9DS0(1000);

//The function used to display the sensor readings to the console
void displaySensorDetails(void){
  sensor_t accel, mag, gyro, temp;
  
  lsm.getSensor(&accel, &mag, &gyro, &temp);
  
  Serial.println(F("------------------------------------"));
  Serial.print  (F("Sensor:       ")); Serial.println(accel.name);
  Serial.print  (F("Driver Ver:   ")); Serial.println(accel.version);
  Serial.print  (F("Unique ID:    ")); Serial.println(accel.sensor_id);
  Serial.print  (F("Max Value:    ")); Serial.print(accel.max_value); Serial.println(F(" m/s^2"));
  Serial.print  (F("Min Value:    ")); Serial.print(accel.min_value); Serial.println(F(" m/s^2"));
  Serial.print  (F("Resolution:   ")); Serial.print(accel.resolution); Serial.println(F(" m/s^2"));  
  Serial.println(F("------------------------------------"));
  Serial.println(F(""));

  Serial.println(F("------------------------------------"));
  Serial.print  (F("Sensor:       ")); Serial.println(mag.name);
  Serial.print  (F("Driver Ver:   ")); Serial.println(mag.version);
  Serial.print  (F("Unique ID:    ")); Serial.println(mag.sensor_id);
  Serial.print  (F("Max Value:    ")); Serial.print(mag.max_value); Serial.println(F(" uT"));
  Serial.print  (F("Min Value:    ")); Serial.print(mag.min_value); Serial.println(F(" uT"));
  Serial.print  (F("Resolution:   ")); Serial.print(mag.resolution); Serial.println(F(" uT"));  
  Serial.println(F("------------------------------------"));
  Serial.println(F(""));

  Serial.println(F("------------------------------------"));
  Serial.print  (F("Sensor:       ")); Serial.println(gyro.name);
  Serial.print  (F("Driver Ver:   ")); Serial.println(gyro.version);
  Serial.print  (F("Unique ID:    ")); Serial.println(gyro.sensor_id);
  Serial.print  (F("Max Value:    ")); Serial.print(gyro.max_value); Serial.println(F(" rad/s"));
  Serial.print  (F("Min Value:    ")); Serial.print(gyro.min_value); Serial.println(F(" rad/s"));
  Serial.print  (F("Resolution:   ")); Serial.print(gyro.resolution); Serial.println(F(" rad/s"));  
  Serial.println(F("------------------------------------"));
  Serial.println(F(""));

  Serial.println(F("------------------------------------"));
  Serial.print  (F("Sensor:       ")); Serial.println(temp.name);
  Serial.print  (F("Driver Ver:   ")); Serial.println(temp.version);
  Serial.print  (F("Unique ID:    ")); Serial.println(temp.sensor_id);
  Serial.print  (F("Max Value:    ")); Serial.print(temp.max_value); Serial.println(F(" C"));
  Serial.print  (F("Min Value:    ")); Serial.print(temp.min_value); Serial.println(F(" C"));
  Serial.print  (F("Resolution:   ")); Serial.print(temp.resolution); Serial.println(F(" C"));  
  Serial.println(F("------------------------------------"));
  Serial.println(F(""));
  
  delay(500);
}
//The function for configuring the sensor i.e. setting the sensitivity
void configureSensor(void){
  // 1.) Set the accelerometer range
  lsm.setupAccel(lsm.LSM9DS0_ACCELRANGE_2G); //Can use: 2G, 4G, 6G, 8G, 16G
  // 2.) Set the magnetometer sensitivity
  lsm.setupMag(lsm.LSM9DS0_MAGGAIN_2GAUSS); //Can use: 4GAUSS, 8GUASS, 12GUASS
  // 3.) Setup the gyroscope
  lsm.setupGyro(lsm.LSM9DS0_GYROSCALE_245DPS); //Can use: 245DPS, 500DPS, 2000DPS
}
/*
 * Required function for arduino
 * This is used to setup everything that is going to be used in the loop
 * of the program before the program enters the infinite loop.
 */
void setup(void) {
    /*
   * Declaring a servo struct, then assinging it to pin 9,
   * then the write function is used to set the servo to a specific position.
   */
  VarSpeedServo myservo;
  myservo.attach(9);
  myservo.write(0, 255, true);
  
  /*
   * Start a serial connection, the passed argument is the baud rate.
   * We are using a fast baud rate to try to fake a sort of easily readable user interface
   */
  Serial.begin(115200);
  
  /*
   * Notify that the sensor is going to be tested. If the test fails the program will
   * enter an infinite loop that will cause the program to get stuck. If the sensor
   * is found and is working, a notification will be sent to the console along with
   * the sensor details. Then the sensor will be configured using the settings in the
   * configureSensor function.
   */
  Serial.println(F("LSM9DS0 9DOF Sensor Test")); Serial.println("");
  //Initialize the sensor, if sensor isn't working print that the sensor wasn't detected
  if(!lsm.begin()){
    Serial.print(F("Ooops, no LSM9DS0 detected ... Check your wiring or I2C ADDR!"));
    //There is an issue with the sensor don't allow program to continue
    while(1);
  }
  Serial.println(F("Found LSM9DS0 9DOF"));
  displaySensorDetails();
  configureSensor();

  Serial.println("");
}

/*
 * Another function required by the arduino. This loops indefinitely
 */
void loop(void) {
  /*
   * Create an event for gather data from the sensors on chip, 
   * then use the getEvent function to get the data from the chip
   */
  sensors_event_t accel, mag, gyro, temp;
  lsm.getEvent(&accel, &mag, &gyro, &temp);

  // print out accelleration data
  Serial.print("Accel X: "); Serial.print(accel.acceleration.x); Serial.print(" ");
  Serial.print("  \tY: "); Serial.print(accel.acceleration.y);       Serial.print(" ");
  Serial.print("  \tZ: "); Serial.print(accel.acceleration.z);     Serial.println("  \tm/s^2");

  // print out magnetometer data
  Serial.print("Magn. X: "); Serial.print(mag.magnetic.x); Serial.print(" ");
  Serial.print("  \tY: "); Serial.print(mag.magnetic.y);       Serial.print(" ");
  Serial.print("  \tZ: "); Serial.print(mag.magnetic.z);     Serial.println("  \tgauss");
  
  // print out gyroscopic data
  Serial.print("Gyro  X: "); Serial.print(gyro.gyro.x); Serial.print(" ");
  Serial.print("  \tY: "); Serial.print(gyro.gyro.y);       Serial.print(" ");
  Serial.print("  \tZ: "); Serial.print(gyro.gyro.z);     Serial.println("  \tdps");

  // print out temperature data
  Serial.print("Temp: "); Serial.print(temp.temperature); Serial.println(" *C");

  Serial.println("**********************\n");

  delay(150);
}
