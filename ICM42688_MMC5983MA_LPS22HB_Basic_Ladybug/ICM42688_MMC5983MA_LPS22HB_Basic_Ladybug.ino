/* 01/14/2022 Copyright Tlera Corporation

    Created by Kris Winer

  This sketch uses SDA/SCL on pins 21/20 (Ladybug default), respectively, and it uses the Ladybug STM32L432 Breakout Board.
  The ICM42688 is a combo sensor with embedded accel and gyro, here used as 6 DoF in a 9 DoF absolute orientation solution
  along with the MMC5983MA magnetometer and the LPS22HB barometer. This is a basic sketch that initializes the sensors,
  perfroms the self tests, calibrates the sensors, sets up data ready interrupts, collects the data at the user specified
  rate and then performs Madgqick sensor fusion and outputs the scaled sensor data a long with quaternions and Euler angles.

  There are still some errors in the treatment of linear acceleration but the sketch serves as a useful starting point for using
  this sensor combination for accurate absolute orientation estimation.

  Library may be used freely and without limit with proper attribution.

*/
#include "ICM42688.h"
#include "MMC5983MA.h"
#include "LPS22HB.h"
#include "RTC.h"
#include "I2Cdev.h"

#define I2C_BUS          Wire                           // Define the I2C bus (Wire instance) you wish to use

I2Cdev                   i2c_0(&I2C_BUS);               // Instantiate the I2Cdev object and point to the desired I2C bus

#define SerialDebug true  // set to true to get Serial output for debugging
#define myLed 13

const char        *build_date = __DATE__;   // 11 characters MMM DD YYYY
const char        *build_time = __TIME__;   // 8 characters HH:MM:SS

// global constants for 9 DoF fusion and AHRS (Attitude and Heading Reference System)
float pi = 3.141592653589793238462643383279502884f;
float GyroMeasError = pi * (40.0f / 180.0f);   // gyroscope measurement error in rads/s (start at 40 deg/s)
float GyroMeasDrift = pi * (0.0f  / 180.0f);   // gyroscope measurement drift in rad/s/s (start at 0.0 deg/s/s)
float beta = sqrtf(3.0f / 4.0f) * GyroMeasError;   // compute beta
float zeta = sqrtf(3.0f / 4.0f) * GyroMeasDrift;   // compute zeta, the other free parameter in the Madgwick scheme usually set to a small or zero value
uint32_t delt_t = 0;                      // used to control display output rate
uint32_t sumCount = 0;                    // used to control display output rate
float pitch, yaw, roll;                   // absolute orientation
float a12, a22, a31, a32, a33;            // rotation matrix coefficients for Euler angles and gravity components
float deltat = 0.0f, sum = 0.0f;          // integration interval for both filter schemes
uint32_t lastUpdate = 0, firstUpdate = 0; // used to calculate integration interval
uint32_t Now = 0;                         // used to calculate integration interval
float lin_ax, lin_ay, lin_az;             // linear acceleration (acceleration with gravity component subtracted)
float q[4] = {1.0f, 0.0f, 0.0f, 0.0f};    // vector to hold quaternion
float eInt[3] = {0.0f, 0.0f, 0.0f};       // vector to hold integral error for Mahony method


//ICM42688 definitions
#define ICM42688_intPin1  9  // interrupt1 pin definitions, significant motion
#define ICM42688_intPin2  8  // interrupt2 pin definitions, data ready

/* Specify sensor parameters (sample rate is twice the bandwidth)
 * choices are:
      AFS_2G, AFS_4G, AFS_8G, AFS_16G  
      GFS_15_625DPS, GFS_31_25DPS, GFS_62_5DPS, GFS_125DPS, GFS_250DPS, GFS_500DPS, GFS_1000DPS, GFS_2000DPS 
      AODR_1_5625Hz, AODR_3_125Hz, AODR_6_25Hz, AODR_50AODR_12_5Hz, AODR_25Hz, AODR_50Hz, AODR_100Hz, AODR_200Hz, AODR_500Hz, 
      AODR_1kHz, AODR_2kHz, AODR_4kHz, AODR_8kHz, AODR_16kHz, AODR_32kHz
      GODR_12_5Hz, GODR_25Hz, GODR_50Hz, GODR_100Hz, GODR_200Hz, GODR_500Hz, GODR_1kHz, GODR_2kHz, GODR_4kHz, GODR_8kHz, GODR_16kHz, GODR_32kHz
*/ 
uint8_t Ascale = AFS_2G, Gscale = GFS_250DPS, AODR = AODR_200Hz, GODR = GODR_200Hz;

float aRes, gRes;                                                        // scale resolutions per LSB for the accel and gyro sensor2
float accelBias[3] = {0.0f, 0.0f,0.0f}, gyroBias[3] = {0.0f, 0.0f,0.0f}; // offset biases for the accel and gyro
int16_t accelDiff[3] = {0, 0, 0}, gyroDiff[3] = {0, 0, 0};               // difference betwee ST and normal values
float  STratio[7] = {0.0f, 0.0f, 0.0f, 0.0f, 0.0f, 0.0f, 0.0f};          // self-test results for the accel and gyro
int16_t ICM42688Data[7];                                                 // Stores the 16-bit signed sensor output
float   Gtemperature;                                                    // Stores the real internal gyro temperature in degrees Celsius
float ax, ay, az, gx, gy, gz;                                            // variables to hold latest accel/gyro data values 

bool newICM42688Data = false;
bool newICM42688Tap  = false;

ICM42688 ICM42688(&i2c_0); // instantiate ICM42688 class


//MMC5983MA definitions
#define MMC5983MA_intPin  5 // interrupt for magnetometer data ready

/* Specify sensor parameters (continuous mode sample rate is dependent on bandwidth)
 * choices are: MODR_ONESHOT, MODR_1Hz, MODR_10Hz, MODR_20Hz, MODR_50 Hz, MODR_100Hz, MODR_200Hz (BW = 0x01), MODR_1000Hz (BW = 0x03)
 * Bandwidth choices are: MBW_100Hz, MBW_200Hz, MBW_400Hz, MBW_800Hz
 * Set/Reset choices are: MSET_1, MSET_25, MSET_75, MSET_100, MSET_250, MSET_500, MSET_1000, MSET_2000, so MSET_100 set/reset occurs every 100th measurement, etc.
*/ 
uint8_t MODR = MODR_100Hz, MBW = MBW_100Hz, MSET = MSET_2000;

float mRes = 1.0f/16384.0f;        // mag sensitivity if using 18 bit data
float magBias[3] = {0, 0, 0}, magScale[3]  = {1, 1, 1}, magOffset[3] = {0}; // Bias corrections for magnetometer
uint32_t MMC5983MAData[3];         // Stores the 18-bit unsigned magnetometer sensor output
uint8_t MMC5983MAtemperature;      // Stores the magnetometer temperature register data
float Mtemperature;                // Stores the real internal chip temperature in degrees Celsius
float mx, my, mz;                  // variables to hold latest mag data values 
uint8_t MMC5983MAstatus;
float MMC5983MA_offset = 131072.0f;

bool newMMC5983MAData = false;

MMC5983MA MMC5983MA(&i2c_0); // instantiate MMC5983MA class


// LPS22H definitions
uint8_t LPS22H_intPin = 4;

/* Specify sensor parameters (sample rate is twice the bandwidth) 
   Choices are P_1Hz, P_10Hz P_25 Hz, P_50Hz, and P_75Hz
 */
uint8_t PODR = P_25Hz;     // set pressure amd temperature output data rate
uint8_t LPS22Hstatus;
float Temperature, Pressure, altitude;

bool newLPS22HData = false;

LPS22H LPS22H(&i2c_0);


// RTC parameter for STM32L4 native RTC class
uint8_t seconds, minutes, hours, day, month, year;
uint8_t Seconds, Minutes, Hours, Day, Month, Year;
bool alarmFlag = false; // for RTC alarm interrupt


void setup() {
  Serial.begin(115200);
  
  // Configure led
  pinMode(myLed, OUTPUT);
  digitalWrite(myLed, HIGH); // start with led off

  pinMode(MMC5983MA_intPin, INPUT);
//  pinMode(LPS22H_intPin, INPUT);
  pinMode(ICM42688_intPin1, INPUT);
//  pinMode(ICM42688_intPin2, INPUT);
  
  Wire.begin(); // set master mode 
  Wire.setClock(400000); // I2C frequency at 400 kHz  
  delay(100);
 
  i2c_0.I2Cscan();

  // Read the ICM42688 Chip ID register, this is a good test of communication
  Serial.println("ICM42688 accel/gyro...");
  byte ICM42688ID = ICM42688.getChipID();  // Read CHIP_ID register for ICM42688
  Serial.print("ICM42688 "); Serial.print("I AM "); Serial.print(ICM42688ID, HEX); Serial.print(" I should be "); Serial.println(0x47, HEX);
  Serial.println(" ");
  delay(1000); 

  // Read the MMC5983MA Chip ID register, this is a good test of communication
  Serial.println("MMC5983MA mag...");
  byte MMC5983ID = MMC5983MA.getChipID();  // Read CHIP_ID register for MMC5983MA
  Serial.print("MMC5983MA "); Serial.print("I AM "); Serial.print(MMC5983ID, HEX); Serial.print(" I should be "); Serial.println(0x30, HEX);
  Serial.println(" ");
  delay(1000); 

  Serial.println("LPS22HB barometer...");
  uint8_t LPS22ID = LPS22H.getChipID();
  Serial.print("LPS25H "); Serial.print("I AM "); Serial.print(LPS22ID, HEX); Serial.print(" I should be "); Serial.println(0xB1, HEX);
  delay(1000); 
  

  if(ICM42688ID == 0x47 && MMC5983ID == 0x30 && LPS22ID == 0xB1) // check if all I2C sensors have acknowledged
  {
   Serial.println("ICM42688 and MMC5983MA and LPS22HB are online..."); Serial.println(" ");
   
   digitalWrite(myLed, LOW);

   ICM42688.reset();  // software reset ICM42688 to default registers

   // set sensor resolutions for self test
   aRes = 4.0f/32768.0f;
   gRes = 250.0f/32768.0f

   ICM42688.selfTest(accelDiff, gyroDiff, STratio);
   Serial.println("Accel Self Test:");
   Serial.print("Ax diff: "); Serial.print(accelDiff[0]* aRes * 1000.0f); Serial.println(" mg");
   Serial.print("Ay diff: "); Serial.print(accelDiff[1]* aRes * 1000.0f); Serial.println(" mg");
   Serial.print("Az diff: "); Serial.print(accelDiff[2]* aRes * 1000.0f); Serial.println(" mg");
   Serial.println("Should be between 50 and 1200 mg");

   Serial.println("Gyro Self Test:");
   Serial.print("Gx diff: "); Serial.print(gyroDiff[0] * gRes); Serial.println(" dps");
   Serial.print("Gy diff: "); Serial.print(gyroDiff[1] * gRes); Serial.println(" dps");
   Serial.print("Gz diff: "); Serial.print(gyroDiff[2] * gRes); Serial.println(" dps");
   Serial.println("Should be > 60 dps");
 
   Serial.print("Ax ratio: "); Serial.print(STratio[1]*100.0f, 0); Serial.println(" %");
   Serial.print("Ay ratio: "); Serial.print(STratio[2]*100.0f, 0); Serial.println(" %");
   Serial.print("Az ratio: "); Serial.print(STratio[3]*100.0f, 0); Serial.println(" %");
   Serial.println("Should be between 50 and 150%");

   Serial.println("Gyro Self Test:");
   Serial.print("Gx ratio: "); Serial.print(STratio[4]*100.0f, 0); Serial.println(" %");
   Serial.print("Gy ratio: "); Serial.print(STratio[5]*100.0f, 0); Serial.println(" %");
   Serial.print("Gz ratio: "); Serial.print(STratio[6]*100.0f, 0); Serial.println(" %");
   Serial.println("Should be between 50 and 150%");
   delay(2000);
      
   // get sensor resolutions for user settings, only need to do this once
   aRes = ICM42688.getAres(Ascale);
   gRes = ICM42688.getGres(Gscale);

  
   ICM42688.init(Ascale, Gscale, AODR, GODR);

   Serial.println("Calculate accel and gyro offset biases: keep sensor flat and motionless!");
   delay(4000);

   ICM42688.offsetBias(accelBias, gyroBias);
   Serial.println("accel biases (mg)"); Serial.println(1000.0f * accelBias[0]); Serial.println(1000.0f * accelBias[1]); Serial.println(1000.0f * accelBias[2]);
   Serial.println("gyro biases (dps)"); Serial.println(gyroBias[0]); Serial.println(gyroBias[1]); Serial.println(gyroBias[2]);
   delay(1000); 

   MMC5983MA.selfTest();
   MMC5983MA.getOffset(magOffset);
   Serial.println("mag offsets:"); Serial.println(magOffset[0]); Serial.println(magOffset[1]); Serial.println(magOffset[2]); 
  
   MMC5983MA.reset(); // software reset MMC5983MA to default registers   

   MMC5983MA.SET(); // "deGauss" magnetometer
   MMC5983MA.init(MODR, MBW, MSET);
   MMC5983MA.offsetBias(magBias, magScale);
   Serial.println("mag biases (mG)"); Serial.println(1000.0f * magBias[0]); Serial.println(1000.0f * magBias[1]); Serial.println(1000.0f * magBias[2]); 
   Serial.println("mag scale "); Serial.println(magScale[0]); Serial.println(magScale[1]); Serial.println(magScale[2]); 
   delay(2000); // add delay to see results before serial spew of data

   LPS22H.Init(PODR);  // Initialize LPS22H altimeter
   delay(1000);

   digitalWrite(myLed, HIGH);
  }
  else 
  {
  if(ICM42688ID != 0x6A) Serial.println(" ICM42688 not functioning!");
  if(MMC5983ID  != 0x30) Serial.println(" MMC5983MA not functioning!");    
  if(LPS22ID    != 0xB1) Serial.println(" LPS22HB not functioning!");   
  }

 // Set the time
  SetDefaultRTC();
  
  /* Set up the RTC alarm interrupt */
  RTC.enableAlarm(RTC.MATCH_ANY); // alarm once a second
  
  RTC.attachInterrupt(alarmMatch); // interrupt every time the alarm sounds

  attachInterrupt(ICM42688_intPin1, myinthandler1, RISING);  // define interrupt for intPin1 output of ICM42688
  attachInterrupt(MMC5983MA_intPin, myinthandler2, RISING);  // define interrupt for intPin  output of MMC5983MA
//  attachInterrupt(LPS22H_intPin, myinthandler3, RISING);  // define interrupt for intPin  output of LPS22HB

  MMC5983MA.clearInt();  //  clear MMC5983MA interrupts before main loop
  ICM42688.DRStatus();   // clear ICM42688 data ready interrupt
}
/* End of setup */


void loop() {

  /*  Strategy here is to have all operations in the main loop be interrupt driven to allow the MCU to sleep when
   *  not servicing an interrupt. The mag interrupt works at the mag rate. The quaternions are updated at the
   *  rate of the gyro, and the latest mag data is used in the Madgwick sensor fusion calculation. The barometer is
   *  read once per second or whenever the RTC alarm fires.
   */

   // If intPin goes high, ICM42688 data registers have new data
   if(newICM42688Data == true) {   // On interrupt, read data
      newICM42688Data = false;     // reset newData flag
     
//   ICM42688.DRStatus(); // clear data ready interrupt if using latched interrupt

     ICM42688.readData(ICM42688Data); // INT1 cleared on any read
   
   // Now we'll calculate the accleration value into actual g's
     ax = (float)ICM42688Data[1]*aRes - accelBias[0];  // get actual g value, this depends on scale being set
     ay = (float)ICM42688Data[2]*aRes - accelBias[1];   
     az = (float)ICM42688Data[3]*aRes - accelBias[2];  

   // Calculate the gyro value into actual degrees per second
     gx = (float)ICM42688Data[4]*gRes - gyroBias[0];  // get actual gyro value, this depends on scale being set
     gy = (float)ICM42688Data[5]*gRes - gyroBias[1];  
     gz = (float)ICM42688Data[6]*gRes - gyroBias[2]; 

    for(uint8_t i = 0; i < 20; i++) { // iterate a fixed number of times per data read cycle
    Now = micros();
    deltat = ((Now - lastUpdate)/1000000.0f); // set integration time by time elapsed since last filter update
    lastUpdate = Now;

    sum += deltat; // sum for averaging filter update rate
    sumCount++;

    MadgwickQuaternionUpdate(ay, ax, -az, gy*pi/180.0f, gx*pi/180.0f, -gz*pi/180.0f,  mx,  -my, -mz);
    }
   }
   

   // If intPin goes high, MMC5983MA magnetometer has new data
   if(newMMC5983MAData == true) {   // On interrupt, read data
      newMMC5983MAData = false;     // reset newData flag

     MMC5983MAstatus = MMC5983MA.status();
     if(MMC5983MAstatus & 0x01) // if mag measurement is done
     {
     MMC5983MA.clearInt();  //  clear MMC5983MA interrupts  
     MMC5983MA.readData(MMC5983MAData);  
   
   // Now we'll calculate the accleration value into actual G's
     mx = ((float)MMC5983MAData[0] - MMC5983MA_offset)*mRes - magBias[0];  // get actual G value 
     my = ((float)MMC5983MAData[1] - MMC5983MA_offset)*mRes - magBias[1];   
     mz = ((float)MMC5983MAData[2] - MMC5983MA_offset)*mRes - magBias[2]; 
     mx *= magScale[0];
     my *= magScale[1];
     mz *= magScale[2];   
     }
   }
   // end sensor interrupt handling
   

   if(alarmFlag) { // update RTC output (serial display) whenever the RTC alarm condition is achieved 
      alarmFlag = false;

    // Read RTC
   if(SerialDebug)
    {
    Serial.println("RTC:");
    Day = RTC.getDay();
    Month = RTC.getMonth();
    Year = RTC.getYear();
    Seconds = RTC.getSeconds();
    Minutes = RTC.getMinutes();
    Hours   = RTC.getHours();     
    if(Hours < 10) {Serial.print("0"); Serial.print(Hours);} else Serial.print(Hours);
    Serial.print(":"); 
    if(Minutes < 10) {Serial.print("0"); Serial.print(Minutes);} else Serial.print(Minutes); 
    Serial.print(":"); 
    if(Seconds < 10) {Serial.print("0"); Serial.println(Seconds);} else Serial.println(Seconds);  

    Serial.print(Month); Serial.print("/"); Serial.print(Day); Serial.print("/"); Serial.println(Year);
    Serial.println(" ");
    }
    
    if(SerialDebug) {
    Serial.print("ax = "); Serial.print((int)1000*ax);  
    Serial.print(" ay = "); Serial.print((int)1000*ay); 
    Serial.print(" az = "); Serial.print((int)1000*az); Serial.println(" mg");
    Serial.print("gx = "); Serial.print( gx, 2); 
    Serial.print(" gy = "); Serial.print( gy, 2); 
    Serial.print(" gz = "); Serial.print( gz, 2); Serial.println(" deg/s");
    Serial.print("mx = "); Serial.print((int)1000*mx);  
    Serial.print(" my = "); Serial.print((int)1000*my); 
    Serial.print(" mz = "); Serial.print((int)1000*mz); Serial.println(" mG");
    
    Serial.print("q0 = "); Serial.print(q[0]);
    Serial.print(" qx = "); Serial.print(q[1]); 
    Serial.print(" qy = "); Serial.print(q[2]); 
    Serial.print(" qz = "); Serial.println(q[3]); 
    }

    // get pressure and temperature from the LPS22HB
    LPS22Hstatus = LPS22H.status();

    if(LPS22Hstatus & 0x01) { // if new pressure data available
    Pressure = (float) LPS22H.readAltimeterPressure()/4096.0f;
    Temperature = (float) LPS22H.readAltimeterTemperature()/100.0f; 
    
    altitude = 145366.45f*(1.0f - powf((Pressure/1013.25f), 0.190284f)); 

    if(SerialDebug) {
      Serial.print("Altimeter temperature = "); Serial.print( Temperature, 2); Serial.println(" C"); // temperature in degrees Celsius  
      Serial.print("Altimeter temperature = "); Serial.print(9.0f*Temperature/5.0f + 32.0f, 2); Serial.println(" F"); // temperature in degrees Fahrenheit
      Serial.print("Altimeter pressure = "); Serial.print(Pressure, 2);  Serial.println(" mbar");// pressure in millibar
      Serial.print("Altitude = "); Serial.print(altitude, 2); Serial.println(" feet");
    }
    }

    Gtemperature = ((float) ICM42688Data[0]) / 132.48f + 25.0f; // Gyro chip temperature in degrees Centigrade
    // Print temperature in degrees Centigrade      
    if(SerialDebug) {
      Serial.print("Gyro temperature is ");  Serial.print(Gtemperature, 1);  Serial.println(" degrees C"); // Print T values to tenths of s degree C
    }

    MMC5983MAtemperature = MMC5983MA.readTemperature(); // this is not working....
    Mtemperature = (((float) MMC5983MAtemperature) * 0.80f) - 75.0f; // Mag chip temperature in degrees Centigrade
    // Print temperature in degrees Centigrade      
    if(SerialDebug) {
      Serial.print("Mag temperature is ");  Serial.print(Mtemperature, 1);  Serial.println(" degrees C"); // Print T values to tenths of s degree C
    }

    a12 =   2.0f * (q[1] * q[2] + q[0] * q[3]);
    a22 =   q[0] * q[0] + q[1] * q[1] - q[2] * q[2] - q[3] * q[3];
    a31 =   2.0f * (q[0] * q[1] + q[2] * q[3]);
    a32 =   2.0f * (q[1] * q[3] - q[0] * q[2]);
    a33 =   q[0] * q[0] - q[1] * q[1] - q[2] * q[2] + q[3] * q[3];
    pitch = -asinf(a32);
    roll  = atan2f(a31, a33);
    yaw   = atan2f(a12, a22);
    pitch *= 180.0f / pi;
    yaw   *= 180.0f / pi; 
    yaw   += 13.8f; // Declination at Danville, California is 13 degrees 48 minutes and 47 seconds on 2014-04-04
    if(yaw < 0) yaw   += 360.0f; // Ensure yaw stays between 0 and 360
    roll  *= 180.0f / pi;
    lin_ax = ax + a31;
    lin_ay = ay + a32;
    lin_az = az - a33;

    if(SerialDebug) {
    Serial.print("Yaw, Pitch, Roll: ");
    Serial.print(yaw, 2);
    Serial.print(", ");
    Serial.print(pitch, 2);
    Serial.print(", ");
    Serial.println(roll, 2);

    Serial.print("Grav_x, Grav_y, Grav_z: ");
    Serial.print(-a31*1000.0f, 2);
    Serial.print(", ");
    Serial.print(-a32*1000.0f, 2);
    Serial.print(", ");
    Serial.print(a33*1000.0f, 2);  Serial.println(" mg");
    Serial.print("Lin_ax, Lin_ay, Lin_az: ");
    Serial.print(lin_ax*1000.0f, 2);
    Serial.print(", ");
    Serial.print(lin_ay*1000.0f, 2);
    Serial.print(", ");
    Serial.print(lin_az*1000.0f, 2);  Serial.println(" mg");
    
    Serial.print("rate = "); Serial.print((float)sumCount/sum, 2); Serial.println(" Hz");
    }

// For plotting comma-delimited Euler angles in a spreadsheet
//     Serial.print(millis()/1000);Serial.print(",");
//     Serial.print(yaw, 2); Serial.print(","); Serial.print(pitch, 2); Serial.print(","); Serial.print(roll, 2); Serial.print(","); Serial.println(Pressure, 2);

    sumCount = 0;
    sum = 0;      

    digitalWrite(myLed, HIGH); delay(1); digitalWrite(myLed, LOW);  // toggle led
    }

    STM32.sleep(); // put MCU to sleep/stop while waiting for an interrupt.
}

/*  End of main loop */


void myinthandler1()
{
  newICM42688Data = true;
}

void myinthandler2()
{
  newMMC5983MAData = true;
}

void myinthandler3()
{
  newLPS22HData = true;
}

void alarmMatch()
{
  alarmFlag = true;
}

void SetDefaultRTC()  // Sets the RTC to the FW build date-time...Courtesy of Greg Tomasch
{
  char Build_mo[3];

  // Convert month string to integer

  Build_mo[0] = build_date[0];
  Build_mo[1] = build_date[1];
  Build_mo[2] = build_date[2];

  String build_mo = Build_mo;

  if(build_mo == "Jan")
  {
    month = 1;
  } else if(build_mo == "Feb")
  {
    month = 2;
  } else if(build_mo == "Mar")
  {
    month = 3;
  } else if(build_mo == "Apr")
  {
    month = 4;
  } else if(build_mo == "May")
  {
    month = 5;
  } else if(build_mo == "Jun")
  {
    month = 6;
  } else if(build_mo == "Jul")
  {
    month = 7;
  } else if(build_mo == "Aug")
  {
    month = 8;
  } else if(build_mo == "Sep")
  {
    month = 9;
  } else if(build_mo == "Oct")
  {
    month = 10;
  } else if(build_mo == "Nov")
  {
    month = 11;
  } else if(build_mo == "Dec")
  {
    month = 12;
  } else
  {
    month = 1;     // Default to January if something goes wrong...
  }

  // Convert ASCII strings to integers
  day     = (build_date[4] - 48)*10 + build_date[5] - 48;  // ASCII "0" = 48
  year    = (build_date[9] - 48)*10 + build_date[10] - 48;
  hours   = (build_time[0] - 48)*10 + build_time[1] - 48;
  minutes = (build_time[3] - 48)*10 + build_time[4] - 48;
  seconds = (build_time[6] - 48)*10 + build_time[7] - 48;

  // Set the date/time

  RTC.setDay(day);
  RTC.setMonth(month);
  RTC.setYear(year);
  RTC.setHours(hours);
  RTC.setMinutes(minutes);
  RTC.setSeconds(seconds);
}
  
