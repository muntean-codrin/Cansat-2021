#include <SPI.h>
#include <SD.h>
#include <Wire.h>
#include <SparkFunMPL3115A2.h>
#include <TinyGPS++.h>
#include "TLC59108.h"
#include "wiring_private.h"
#include "ICM_20948.h"
#include "helpFunctions.h"
#include <stdlib.h>
#include "motors.h"


//compare max altitude to current altitude; if the diff is bigger than this -> cansat has been launched
float LAUNCH_ALTITUDE_DIFFERENCE = 10; //10-50
int LAND_ALTITUDE_DIFFERENCE = 5;  //3-5
float ANGLE_TOLERANCE = 20;
float CONTROL_LOOP_DELAY = 15000; //ms
int MIN_DISTANCE_CONTROL = 50; //meters
int MOTOR_TARGET_PULL = 2000; //ticks


bool gpsInit = true;
bool mpuInit = true;
bool sdCardInit = true;

int mode = 0;
rgbColor ledColor;

TinyGPSPlus gps;
TLC59108 rgbLed(16);
ICM_20948_I2C myICM;
MPL3115A2 mplSensor;

File logFile;
missionState state;

Uart mySerial (&sercom1, 13, 11, SERCOM_RX_PAD_1, UART_TX_PAD_0);

char fileName[30] = "";
unsigned long startTime = 0, timeNow, landCheckTime = 0, parachuteControllTime = 0, stateTime = 0;

void setup()
{
  delay(5000);
  Serial.begin(115200);
  Wire.begin();

  pinMode(18, INPUT); // voltage
  pinMode(0, OUTPUT); // buzzer
  digitalWrite(0, LOW);
  pinMode(4, OUTPUT); // led1
  digitalWrite(4, LOW);
  pinMode(15, OUTPUT); // led2
  digitalWrite(15, LOW);

  rgbLed.init();
  rgbLed.setLedOutputMode(LED_MODE::PWM_IND);
  rgbLed.setAllBrightness(0);
  ledColor.setColor(0, 255, 0);

  SetupMotors();

  mySerial.begin(9600);
  pinPeripheral(11, PIO_SERCOM);
  pinPeripheral(13, PIO_SERCOM);

  mplSensor.begin();
  mplSensor.setModeBarometer();
  mplSensor.setOversampleRate(7);
  mplSensor.enableEventFlags();

  Wire.begin();
  Wire.setClock(400000);
  int noTries = 15, currentTries = 0;
  bool initialized = false;
  while ( !initialized && currentTries <= noTries) {
    myICM.begin(Wire, 0 );
    Serial.print( F("IMU INIT: ") );
    Serial.println( myICM.statusString() );
    if ( myICM.status != ICM_20948_Stat_Ok ) {
      Serial.println( "Trying again..." );
      delay(500);
      currentTries++;
      mpuInit = false;
    } else {
      initialized = true;
      mpuInit = true;
    }
  }

  if (!SD.begin(1)) {
    sdCardInit = false;
  }


  Serial.print("MPU INIT:");
  Serial.println(mpuInit);
  Serial.print("SD CARD INIT:");
  Serial.println(sdCardInit);



  if (mpuInit && sdCardInit)
  {
    rgbLed.setGreen();
    //change mode if shake
    long long timer2 = millis();
    while (millis() - timer2 < 10000)
    {
      if ( myICM.dataReady() )
      {
        myICM.getAGMT();
        float accelerationZ = (&myICM)->accZ();

        accelerationZ = abs(accelerationZ);
        if (accelerationZ > 1700)
        {
          mode++;
          if (mode == 3)
          {
            mode = 0;
          }

          if (mode == 0)
          {
            ledColor.setColor(0, 255, 0);
          }
          else if (mode == 1)
          {
            ledColor.setColor(255, 0, 255);
          }
          else if (mode == 2)
          {
            ledColor.setColor(255, 100, 0);
          }
          rgbLed.setRGB(ledColor);
          delay(1000);
        }
      }
    }


    mpuInit = configICM(myICM);
    Serial.print("Mode: ");
    Serial.println(mode);
    rgbLed.setAllBrightness(0);

    //wait for gps data
    bool ledFlick = false;
    long long timer1 = millis();
    while (false)//!gps.location.isValid()) //gps.time.second() == 0)
    {
      while (mySerial.available()) {
        gps.encode(mySerial.read());
      }

      if (millis() - timer1 > 1000)
      {
        if (ledFlick)
          rgbLed.setBlue();
        else
          rgbLed.setAllBrightness(0);
        ledFlick = !ledFlick;
        timer1 = millis();
      }


    }

    //init file name
    char s_hour[10] = "";
    char s_minute[10] = "";

    strcat(fileName, "f");

    if (gps.time.hour() + 3 < 10)
      strcat(fileName, "0");
    itoa(gps.time.hour() + 3, s_hour, 10);
    strcat(fileName, s_hour);

    if (gps.time.minute() < 10)
      strcat(fileName, "0");
    itoa(gps.time.minute(), s_minute, 10);
    strcat(fileName, s_minute);
    strcat(fileName, ".txt");


    Serial.print("Filename: ");
    Serial.println(fileName);

    logFile = SD.open(fileName, FILE_WRITE);
    if (!logFile)
    {
      rgbLed.setRed();
      while (1)
        delay(1000);
    }
    rgbLed.setRGB(ledColor);
    logFile.close();
  }
  else
  {
    rgbLed.setRed();
    while (1)
      delay(1000);

  }
  state = Waiting;
  startTime = millis() - 1000;
}

double yaw = 0, yawOffset = 0;
bool blinkLed = false;
int motorId = 0;

float maxAltitude = 0.0, landCheckAltitude = 0.0;
double referenceLat = 0.0, referenceLng = 0.0;
float distanceFromReference = 0;
bool pControl = false;
double targetAngle = 0;
int rotateDirection = 0;

void loop()
{
  if (mode == 2)
  {
    Serial.print(motors[0].GetEncoder());
    Serial.print(" ");
    Serial.print(motors[1].GetEncoder());
    Serial.print(" ");
    Serial.println(motors[2].GetEncoder());

    if (Serial.available())
    {
      String command = Serial.readStringUntil('\n');

      if (command.length() > 2)
      {
        //ticks
        motors[motorId].MoveTo(command.toInt());
      }
      else
      {
        //motor
        motorId = command.toInt();

      }

    }

  }
  else if (mode == 0)
  {
    readIMU();
    readGPS();

    timeNow = millis();
    if (timeNow - startTime > 500)
    {
      float pressure = getPressure(mplSensor);
      float altitude = getAltitude(mplSensor, pressure);
      float temperture = myICM.temp();
      double latitude = 0, longitude = 0, altitudeGps = 0;
      uint8_t hour = 0, minute = 0, second = 0;
      double voltage = analogRead(18) * 0.00726;


      if (gps.time.isValid())
      {
        hour = gps.time.hour() + 3;
        minute = gps.time.minute();
        second = gps.time.second();
      }
      if (gps.location.isValid())
      {
        latitude = gps.location.lat();
        longitude = gps.location.lng();
      }
      if (gps.altitude.isValid())
      {
        altitudeGps = gps.altitude.meters();
      }

      if (state == Waiting)
      {
        if (altitude > maxAltitude)
        {
          maxAltitude = altitude;
        }
        if (altitude < maxAltitude - LAUNCH_ALTITUDE_DIFFERENCE)
        {
          state = Launched;
          referenceLat = latitude;
          referenceLng = longitude;
        }
      }
      if (state == Launched)
      {
        distanceFromReference = calculatePositionOffset(latitude, longitude);

        if (millis() - landCheckTime > 20000)
        {
          if (abs(altitude - landCheckAltitude) < LAND_ALTITUDE_DIFFERENCE)
          {
            state = Landed;
            digitalWrite(0, HIGH);
          }
          landCheckAltitude = altitude;
          landCheckTime = millis();
        }
      }

      logData(fileName, voltage, pressure, altitude, maxAltitude, temperture, latitude, longitude, altitudeGps, distanceFromReference,  hour, minute, second, yaw, motors[1].GetEncoder(), motors[2].GetEncoder(), state);

      if (blinkLed) rgbLed.setRGB(ledColor);
      else rgbLed.setAllBrightness(0);
      blinkLed = !blinkLed;

      startTime = timeNow;
    }
  }
  else if (mode == 1)
  {
    //gps calculations
    readIMU();
    readGPS();

    float pressure = 0, altitude = 0, temperture = 0;
    double latitude = 0, longitude = 0, altitudeGps = 0, voltage = 0;
    uint8_t hour = 0, minute = 0, second = 0;

    timeNow = millis();
    if (timeNow - startTime > 500)
    {
      pressure = getPressure(mplSensor);
      altitude = getAltitude(mplSensor, pressure);
      temperture = myICM.temp();
      voltage = analogRead(18) * 0.00726;

      if (gps.time.isValid())
      {
        hour = gps.time.hour() + 3;
        minute = gps.time.minute();
        second = gps.time.second();
      }
      if (gps.location.isValid())
      {
        latitude = gps.location.lat();
        longitude = gps.location.lng();
      }
      if (gps.altitude.isValid())
      {
        altitudeGps = gps.altitude.meters();
      }

      if (state == Waiting)
      {
        if (altitude > maxAltitude)
        {
          maxAltitude = altitude;
        }
        if (altitude < maxAltitude - LAUNCH_ALTITUDE_DIFFERENCE)
        {
          Serial.println();
          Serial.print(altitude);
          Serial.print(" ");
          Serial.print(maxAltitude);
          Serial.print(" ");
          Serial.println(LAUNCH_ALTITUDE_DIFFERENCE);
          Serial.print(" ");
          Serial.println(maxAltitude - LAUNCH_ALTITUDE_DIFFERENCE);
          Serial.println();
          
          state = Launched;
          referenceLat = latitude;
          referenceLng = longitude;
          parachuteControllTime = millis();
          yawOffset = yaw;
        }
      }



      logData(fileName, voltage, pressure, altitude, maxAltitude, temperture, latitude, longitude, altitudeGps, distanceFromReference,  hour, minute, second, yaw, motors[1].GetEncoder(), motors[2].GetEncoder(), state);

      if (blinkLed) rgbLed.setRGB(ledColor);
      else rgbLed.setAllBrightness(0);
      blinkLed = !blinkLed;

      startTime = timeNow;

    }

    if (millis() - stateTime > 100)
    {
      if (state == Launched)
      {
        distanceFromReference = calculatePositionOffset(latitude, longitude);

        //Begin turn
        if (!pControl && distanceFromReference > MIN_DISTANCE_CONTROL && millis() - parachuteControllTime > CONTROL_LOOP_DELAY)
        {
          double currentAngle = yaw - yawOffset;
          if (currentAngle < 0)
            currentAngle += 360;

          targetAngle = 0;

          //calculate target angle
          double coorX, coorY;
          coorX = longitude - referenceLng;
          coorY = latitude - referenceLat;

          if (coorX >= 0 && coorY >= 0) //cadranul 1
          {
            targetAngle = 225;
          }
          else if (coorX <= 0 && coorY >= 0) //cadranul 2
          {
            targetAngle = 135;
          }
          else if (coorX <= 0 && coorY <= 0) //cadranul 3
          {
            targetAngle = 45;
          }
          else if (coorX >= 0 && coorY <= 0) // cadranul 4
          {
            targetAngle = 315;
          }

          double deltaAngle = currentAngle - targetAngle;
          if (deltaAngle < 0)
            deltaAngle += 360;

          if (deltaAngle < 180)
          {
            //stanga
            motors[1].MoveTo(MOTOR_TARGET_PULL);
            rotateDirection = -1;
          }
          else
          {
            //dreapta
            motors[2].MoveTo(MOTOR_TARGET_PULL);
            rotateDirection = 1;
          }

          pControl = true;
        }

        //Turn loop
        if (pControl)
        {
          double currentAngle = yaw - yawOffset;
          if (currentAngle < 0)
            currentAngle += 360;

          double remainingAngle;
          if (rotateDirection == 1) //rotire drepta => target > current
          {
            remainingAngle = targetAngle - currentAngle;
            if (remainingAngle < 0) remainingAngle += 360;

          }
          else
          {
            remainingAngle = currentAngle - targetAngle;
            if (remainingAngle < 0) remainingAngle += 360;
          }
          Serial.print(rotateDirection); Serial.print(" ");
          Serial.print(currentAngle); Serial.print(" ");
          Serial.print(targetAngle); Serial.print(" ");
          Serial.print(remainingAngle); Serial.println(" ");

          if (remainingAngle < ANGLE_TOLERANCE)
          {
            motors[1].MoveTo(0);
            motors[2].MoveTo(0);
            pControl = false;
            parachuteControllTime = millis();
          }

        }

        if (millis() - landCheckTime > 20000)
        {
          if (abs(altitude - landCheckAltitude) < LAND_ALTITUDE_DIFFERENCE)
          {
            state = Landed;
            digitalWrite(0, HIGH);
          }
          landCheckAltitude = altitude;
          landCheckTime = millis();
        }
      }

      stateTime = millis();
    }



  }
}


void readGPS()
{
  while (mySerial.available()) {
    gps.encode(mySerial.read());
  }
}

void readIMU()
{

  if (myICM.dataReady())
  {
    myICM.getAGMT();
  }

  icm_20948_DMP_data_t data;
  myICM.readDMPdataFromFIFO(&data);

  if (( myICM.status == ICM_20948_Stat_Ok ) || ( myICM.status == ICM_20948_Stat_FIFOMoreDataAvail )) // Was valid data available?
  {
    if ((data.header & DMP_header_bitmap_Quat6) > 0 ) // We have asked for GRV data so we should receive Quat6
    {
      double q1 = ((double)data.Quat6.Data.Q1) / 1073741824.0; // Convert to double. Divide by 2^30
      double q2 = ((double)data.Quat6.Data.Q2) / 1073741824.0; // Convert to double. Divide by 2^30
      double q3 = ((double)data.Quat6.Data.Q3) / 1073741824.0; // Convert to double. Divide by 2^30
      double q0 = sqrt( 1.0 - ((q1 * q1) + (q2 * q2) + (q3 * q3)));

      double q2sqr = q2 * q2;

      // roll (x-axis rotation)
      //double t0 = +2.0 * (q0 * q1 + q2 * q3);
      //double t1 = +1.0 - 2.0 * (q1 * q1 + q2sqr);
      //double roll = atan2(t0, t1) * 180.0 / PI;

      // pitch (y-axis rotation)
      //double t2 = +2.0 * (q0 * q2 - q3 * q1);
      //t2 = t2 > 1.0 ? 1.0 : t2;
      //t2 = t2 < -1.0 ? -1.0 : t2;
      //double pitch = asin(t2) * 180.0 / PI;

      // yaw (z-axis rotation)
      double t3 = +2.0 * (q0 * q3 + q1 * q2);
      double t4 = +1.0 - 2.0 * (q2sqr + q3 * q3);
      yaw = atan2(t3, t4) * 180.0 / PI;

      if (yaw < 0)
        yaw += 360;
    }
  }
}

float calculatePositionOffset(double lat, double lng)
{
  float earth_radius = 6371000;
  float epsilon1 = lat * PI / 180;
  float epsilon2 = referenceLat * PI / 180;
  float delta1 = lng * PI / 180;
  float delta2 = referenceLng * PI / 180;

  float x = (delta2 - delta1) * cos((epsilon1 + epsilon2) / 2);
  float y = epsilon2 - epsilon1;

  float distance = sqrt(x * x + y * y) * earth_radius;
  return distance;
}



void SERCOM1_Handler()
{
  mySerial.IrqHandler();
}
