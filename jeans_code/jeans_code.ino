#include <SPI.h>
#include <Wire.h>
#include <VarSpeedServo.h>
#include <Adafruit_Sensor.h>
#include <Adafruit_LSM9DS0.h>
#include <Adafruit_Simple_AHRS.h>

VarSpeedServo myservo;
// Create LSM9DS0 board instance.
Adafruit_LSM9DS0     lsm(1000);  // Use I2C, ID #1000

// Create simple AHRS algorithm using the LSM9DS0 instance's accelerometer and magnetometer.
Adafruit_Simple_AHRS ahrs(&lsm.getAccel(), &lsm.getMag());

// Function to configure the sensors on the LSM9DS0 board.
// You don't need to change anything here, but have the option to select different
// range and gain values.
void configureLSM9DS0(void)
{
  // 1.) Set the accelerometer range
  lsm.setupAccel(lsm.LSM9DS0_ACCELRANGE_2G);
  //lsm.setupAccel(lsm.LSM9DS0_ACCELRANGE_4G);
  //lsm.setupAccel(lsm.LSM9DS0_ACCELRANGE_6G);
  //lsm.setupAccel(lsm.LSM9DS0_ACCELRANGE_8G);
  //lsm.setupAccel(lsm.LSM9DS0_ACCELRANGE_16G);
  
  // 2.) Set the magnetometer sensitivity
  lsm.setupMag(lsm.LSM9DS0_MAGGAIN_2GAUSS);
  //lsm.setupMag(lsm.LSM9DS0_MAGGAIN_4GAUSS);
  //lsm.setupMag(lsm.LSM9DS0_MAGGAIN_8GAUSS);
  //lsm.setupMag(lsm.LSM9DS0_MAGGAIN_12GAUSS);

  // 3.) Setup the gyroscope
  lsm.setupGyro(lsm.LSM9DS0_GYROSCALE_245DPS);
  //lsm.setupGyro(lsm.LSM9DS0_GYROSCALE_500DPS);
  //lsm.setupGyro(lsm.LSM9DS0_GYROSCALE_2000DPS);
}

void setup(void) 
{
  Serial.begin(38400);
  Serial.println(F("Adafruit LSM9DS0 9 DOF Board AHRS Example")); Serial.println("");
  
  // Initialise the LSM9DS0 board.
  if(!lsm.begin())
  {
    // There was a problem detecting the LSM9DS0 ... check your connections
    Serial.print(F("Ooops, no LSM9DS0 detected ... Check your wiring or I2C ADDR!"));
    while(1);
  }
  
  // Setup the sensor gain and integration time.
  configureLSM9DS0();


  //SERVO //////////////////////////////////////
  myservo.attach(9);
  myservo.write(90, 255, true);     //mid point

  //Excel Data/////////////////////////////////////////////////////////////////////////////////////////
  /* We're ready to go! */
  Serial.println("");

  Serial.println("CLEARDATA"); //clears up any data left from previous projects

Serial.println("LABEL,Time,Timer,Gyro X,Gyro Y,Gyro Z,Accel X,Accel Y,Accel Z"); //always write LABEL, so excel knows the next things will be the names of the columns (instead of Acolumn you could write Time for instance)

Serial.println("RESETTIMER"); //resets timer to 0
}

void loop(void) 
{
  sensors_vec_t   orientation;

  // Use the simple AHRS function to get the current orientation.
  if (ahrs.getOrientation(&orientation))
  {
    /* 'orientation' should have valid .roll and .pitch fields 
    Serial.print(F("Orientation: "));
    Serial.print(orientation.roll);
    Serial.print(F(" "));
    Serial.print(orientation.pitch);
    Serial.print(F(" "));
    Serial.print(orientation.heading);
    Serial.println(F(""));     */

     /////////////////////////// for data collection////
  
Serial.print("DATA,TIME,TIMER,"); //writes the time in the first column A and the time since the measurements started in column B

 Serial.print(orientation.pitch); Serial.print(",");
 Serial.print(orientation.roll); Serial.print(",");
 Serial.print(orientation.heading); Serial.print(",");


 Serial.println(); //be sure to add println to the last command so it knows to go into the next row on the second run


//  if (orientation.pitch > 30){
//    myservo.write(120, 255, true);
//    delay (1);
//    //myservo.write(90, 255, true);
//  }
//  if (orientation.pitch < -30){
//    myservo.write(60, 255, true);
//    delay (1);
//    //myservo.write(120, 255, true);
//  }

  

  
  delay(100);
}
}


