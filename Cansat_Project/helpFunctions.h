enum missionState
{
  Waiting = 1,
  Launched = 2,
  Landed = 3,
};


void reverse(char* str, int len) 
{ 
    int i = 0, j = len - 1, temp; 
    while (i < j) { 
        temp = str[i]; 
        str[i] = str[j]; 
        str[j] = temp; 
        i++; 
        j--; 
    } 
} 

bool configICM(ICM_20948_I2C myICM)
{
    bool success = true; // Use success to show if the configuration was successful

  // Configure clock source through PWR_MGMT_1
  // ICM_20948_Clock_Auto selects the best available clock source – PLL if ready, else use the Internal oscillator
  success &= (myICM.setClockSource(ICM_20948_Clock_Auto) == ICM_20948_Stat_Ok); // This is shorthand: success will be set to false if setClockSource fails

  // Enable accel and gyro sensors through PWR_MGMT_2
  // Enable Accelerometer (all axes) and Gyroscope (all axes) by writing zero to PWR_MGMT_2
  success &= (myICM.setBank(0) == ICM_20948_Stat_Ok); // Select Bank 0
  uint8_t pwrMgmt2 = 0x40; // Set the reserved bit 6
  success &= (myICM.write(AGB0_REG_PWR_MGMT_2, &pwrMgmt2, 1) == ICM_20948_Stat_Ok); // Write one byte to the PWR_MGMT_2 register

  // Configure I2C_Master/Gyro/Accel in Low Power Mode (cycled) with LP_CONFIG
  success &= (myICM.setSampleMode( (ICM_20948_Internal_Mst | ICM_20948_Internal_Acc | ICM_20948_Internal_Gyr), ICM_20948_Sample_Mode_Cycled ) == ICM_20948_Stat_Ok);

  // Disable the FIFO
  success &= (myICM.enableFIFO(false) == ICM_20948_Stat_Ok);

  // Disable the DMP
  success &= (myICM.enableDMP(false) == ICM_20948_Stat_Ok);

  // Set Gyro FSR (Full scale range) to 2000dps through GYRO_CONFIG_1
  // Set Accel FSR (Full scale range) to 4g through ACCEL_CONFIG
  ICM_20948_fss_t myFSS;  // This uses a "Full Scale Settings" structure that can contain values for all configurable sensors
  myFSS.a = gpm4;         // (ICM_20948_ACCEL_CONFIG_FS_SEL_e)
                          // gpm2
                          // gpm4
                          // gpm8
                          // gpm16
  myFSS.g = dps2000;       // (ICM_20948_GYRO_CONFIG_1_FS_SEL_e)
                          // dps250
                          // dps500
                          // dps1000
                          // dps2000
  success &= (myICM.setFullScale( (ICM_20948_Internal_Acc | ICM_20948_Internal_Gyr), myFSS ) == ICM_20948_Stat_Ok);

  // Enable interrupt for FIFO overflow from FIFOs through INT_ENABLE_2
  // If we see this interrupt, we'll need to reset the FIFO
  //success &= (myICM.intEnableOverflowFIFO( 0x1F ) == ICM_20948_Stat_Ok); // Enable the interrupt on all FIFOs

  // Turn off what goes into the FIFO through FIFO_EN_1, FIFO_EN_2
  // Stop the peripheral data from being written to the FIFO by writing zero to FIFO_EN_1
  success &= (myICM.setBank(0) == ICM_20948_Stat_Ok); // Select Bank 0
  uint8_t zero = 0;
  success &= (myICM.write(AGB0_REG_FIFO_EN_1, &zero, 1) == ICM_20948_Stat_Ok);
  // Stop the accelerometer, gyro and temperature data from being written to the FIFO by writing zero to FIFO_EN_2
  success &= (myICM.write(AGB0_REG_FIFO_EN_2, &zero, 1) == ICM_20948_Stat_Ok);

  // Turn off data ready interrupt through INT_ENABLE_1
  success &= (myICM.intEnableRawDataReady(false) == ICM_20948_Stat_Ok);

  // Reset FIFO through FIFO_RST
  success &= (myICM.resetFIFO() == ICM_20948_Stat_Ok);

  // Set gyro sample rate divider with GYRO_SMPLRT_DIV
  // Set accel sample rate divider with ACCEL_SMPLRT_DIV_2
  ICM_20948_smplrt_t mySmplrt;
  mySmplrt.g = 19; // ODR is computed as follows: 1.1 kHz/(1+GYRO_SMPLRT_DIV[7:0]). 19 = 55Hz. InvenSense Nucleo example uses 19 (0x13).
  mySmplrt.a = 19; // ODR is computed as follows: 1.125 kHz/(1+ACCEL_SMPLRT_DIV[11:0]). 19 = 56.25Hz. InvenSense Nucleo example uses 19 (0x13).
  myICM.setSampleRate( (ICM_20948_Internal_Acc | ICM_20948_Internal_Gyr), mySmplrt ); // ** Note: comment this line to leave the sample rates at the maximum **
  
  // Setup DMP start address through PRGM_STRT_ADDRH/PRGM_STRT_ADDRL
  success &= (myICM.setDMPstartAddress() == ICM_20948_Stat_Ok); // Defaults to DMP_START_ADDRESS

  // Now load the DMP firmware
  success &= (myICM.loadDMPFirmware() == ICM_20948_Stat_Ok);

  // Write the 2 byte Firmware Start Value to ICM PRGM_STRT_ADDRH/PRGM_STRT_ADDRL
  success &= (myICM.setDMPstartAddress() == ICM_20948_Stat_Ok); // Defaults to DMP_START_ADDRESS

  // Set the Hardware Fix Disable register to 0x48
  success &= (myICM.setBank(0) == ICM_20948_Stat_Ok); // Select Bank 0
  uint8_t fix = 0x48;
  success &= (myICM.write(AGB0_REG_HW_FIX_DISABLE, &fix, 1) == ICM_20948_Stat_Ok);
  
  // Set the Single FIFO Priority Select register to 0xE4
  success &= (myICM.setBank(0) == ICM_20948_Stat_Ok); // Select Bank 0
  uint8_t fifoPrio = 0xE4;
  success &= (myICM.write(AGB0_REG_SINGLE_FIFO_PRIORITY_SEL, &fifoPrio, 1) == ICM_20948_Stat_Ok);
  
  // Configure Accel scaling to DMP
  // The DMP scales accel raw data internally to align 1g as 2^25
  // In order to align internal accel raw data 2^25 = 1g write 0x04000000 when FSR is 4g
  const unsigned char accScale[4] = {0x04, 0x00, 0x00, 0x00};
  success &= (myICM.writeDMPmems(ACC_SCALE, 4, &accScale[0]) == ICM_20948_Stat_Ok); // Write accScale to ACC_SCALE DMP register
  // In order to output hardware unit data as configured FSR write 0x00040000 when FSR is 4g
  const unsigned char accScale2[4] = {0x00, 0x04, 0x00, 0x00};
  success &= (myICM.writeDMPmems(ACC_SCALE2, 4, &accScale2[0]) == ICM_20948_Stat_Ok); // Write accScale2 to ACC_SCALE2 DMP register

  // Configure Compass mount matrix and scale to DMP
  // The mount matrix write to DMP register is used to align the compass axes with accel/gyro.
  // This mechanism is also used to convert hardware unit to uT. The value is expressed as 1uT = 2^30.
  // Each compass axis will be converted as below:
  // X = raw_x * CPASS_MTX_00 + raw_y * CPASS_MTX_01 + raw_z * CPASS_MTX_02
  // Y = raw_x * CPASS_MTX_10 + raw_y * CPASS_MTX_11 + raw_z * CPASS_MTX_12
  // Z = raw_x * CPASS_MTX_20 + raw_y * CPASS_MTX_21 + raw_z * CPASS_MTX_22
  // The AK09916 produces a 16-bit signed output in the range +/-32752 corresponding to +/-4912uT. 1uT = 6.66 ADU.
  // 2^30 / 6.66666 = 161061273 = 0x9999999
  const unsigned char mountMultiplierZero[4] = {0x00, 0x00, 0x00, 0x00};
  const unsigned char mountMultiplierPlus[4] = {0x09, 0x99, 0x99, 0x99}; // Value taken from InvenSense Nucleo example
  const unsigned char mountMultiplierMinus[4] = {0xF6, 0x66, 0x66, 0x67}; // Value taken from InvenSense Nucleo example
  success &= (myICM.writeDMPmems(CPASS_MTX_00, 4, &mountMultiplierPlus[0]) == ICM_20948_Stat_Ok);
  success &= (myICM.writeDMPmems(CPASS_MTX_01, 4, &mountMultiplierZero[0]) == ICM_20948_Stat_Ok);
  success &= (myICM.writeDMPmems(CPASS_MTX_02, 4, &mountMultiplierZero[0]) == ICM_20948_Stat_Ok);
  success &= (myICM.writeDMPmems(CPASS_MTX_10, 4, &mountMultiplierZero[0]) == ICM_20948_Stat_Ok);
  success &= (myICM.writeDMPmems(CPASS_MTX_11, 4, &mountMultiplierMinus[0]) == ICM_20948_Stat_Ok);
  success &= (myICM.writeDMPmems(CPASS_MTX_12, 4, &mountMultiplierZero[0]) == ICM_20948_Stat_Ok);
  success &= (myICM.writeDMPmems(CPASS_MTX_20, 4, &mountMultiplierZero[0]) == ICM_20948_Stat_Ok);
  success &= (myICM.writeDMPmems(CPASS_MTX_21, 4, &mountMultiplierZero[0]) == ICM_20948_Stat_Ok);
  success &= (myICM.writeDMPmems(CPASS_MTX_22, 4, &mountMultiplierMinus[0]) == ICM_20948_Stat_Ok);

  // Configure the B2S Mounting Matrix
  const unsigned char b2sMountMultiplierZero[4] = {0x00, 0x00, 0x00, 0x00};
  const unsigned char b2sMountMultiplierPlus[4] = {0x40, 0x00, 0x00, 0x00}; // Value taken from InvenSense Nucleo example
  success &= (myICM.writeDMPmems(B2S_MTX_00, 4, &b2sMountMultiplierPlus[0]) == ICM_20948_Stat_Ok);
  success &= (myICM.writeDMPmems(B2S_MTX_01, 4, &b2sMountMultiplierZero[0]) == ICM_20948_Stat_Ok);
  success &= (myICM.writeDMPmems(B2S_MTX_02, 4, &b2sMountMultiplierZero[0]) == ICM_20948_Stat_Ok);
  success &= (myICM.writeDMPmems(B2S_MTX_10, 4, &b2sMountMultiplierZero[0]) == ICM_20948_Stat_Ok);
  success &= (myICM.writeDMPmems(B2S_MTX_11, 4, &b2sMountMultiplierPlus[0]) == ICM_20948_Stat_Ok);
  success &= (myICM.writeDMPmems(B2S_MTX_12, 4, &b2sMountMultiplierZero[0]) == ICM_20948_Stat_Ok);
  success &= (myICM.writeDMPmems(B2S_MTX_20, 4, &b2sMountMultiplierZero[0]) == ICM_20948_Stat_Ok);
  success &= (myICM.writeDMPmems(B2S_MTX_21, 4, &b2sMountMultiplierZero[0]) == ICM_20948_Stat_Ok);
  success &= (myICM.writeDMPmems(B2S_MTX_22, 4, &b2sMountMultiplierPlus[0]) == ICM_20948_Stat_Ok);

  // Configure the DMP Gyro Scaling Factor
  // @param[in] gyro_div Value written to GYRO_SMPLRT_DIV register, where
  //            0=1125Hz sample rate, 1=562.5Hz sample rate, ... 4=225Hz sample rate, ...
  //            10=102.2727Hz sample rate, ... etc.
  // @param[in] gyro_level 0=250 dps, 1=500 dps, 2=1000 dps, 3=2000 dps
  success &= (myICM.setGyroSF(19, 3) == ICM_20948_Stat_Ok); // 19 = 55Hz (see above), 3 = 2000dps (see above)
  
  // Configure the Gyro full scale
  // 2000dps : 2^28
  // 1000dps : 2^27
  //  500dps : 2^26
  //  250dps : 2^25
  const unsigned char gyroFullScale[4] = {0x10, 0x00, 0x00, 0x00}; // 2000dps : 2^28
  success &= (myICM.writeDMPmems(GYRO_FULLSCALE, 4, &gyroFullScale[0]) == ICM_20948_Stat_Ok);

  // Configure the Accel Only Gain: 15252014 (225Hz) 30504029 (112Hz) 61117001 (56Hz)
  const unsigned char accelOnlyGain[4] = {0x03, 0xA4, 0x92, 0x49}; // 56Hz
  //const unsigned char accelOnlyGain[4] = {0x00, 0xE8, 0xBA, 0x2E}; // InvenSense Nucleo example uses 225Hz
  success &= (myICM.writeDMPmems(ACCEL_ONLY_GAIN, 4, &accelOnlyGain[0]) == ICM_20948_Stat_Ok);
  
  // Configure the Accel Alpha Var: 1026019965 (225Hz) 977872018 (112Hz) 882002213 (56Hz)
  const unsigned char accelAlphaVar[4] = {0x34, 0x92, 0x49, 0x25}; // 56Hz
  //const unsigned char accelAlphaVar[4] = {0x06, 0x66, 0x66, 0x66}; // Value taken from InvenSense Nucleo example
  success &= (myICM.writeDMPmems(ACCEL_ALPHA_VAR, 4, &accelAlphaVar[0]) == ICM_20948_Stat_Ok);
  
  // Configure the Accel A Var: 47721859 (225Hz) 95869806 (112Hz) 191739611 (56Hz)
  const unsigned char accelAVar[4] = {0x0B, 0x6D, 0xB6, 0xDB}; // 56Hz
  //const unsigned char accelAVar[4] = {0x39, 0x99, 0x99, 0x9A}; // Value taken from InvenSense Nucleo example
  success &= (myICM.writeDMPmems(ACCEL_A_VAR, 4, &accelAVar[0]) == ICM_20948_Stat_Ok);
  
  // Configure the Accel Cal Rate
  const unsigned char accelCalRate[4] = {0x00, 0x00}; // Value taken from InvenSense Nucleo example
  success &= (myICM.writeDMPmems(ACCEL_CAL_RATE, 2, &accelCalRate[0]) == ICM_20948_Stat_Ok);

  // Configure the Compass Time Buffer. The compass (magnetometer) is set to 100Hz (AK09916_mode_cont_100hz)
  // in startupMagnetometer. We need to set CPASS_TIME_BUFFER to 100 too.
  const unsigned char compassRate[2] = {0x00, 0x64}; // 100Hz
  success &= (myICM.writeDMPmems(CPASS_TIME_BUFFER, 2, &compassRate[0]) == ICM_20948_Stat_Ok);
  
  // Enable DMP interrupt
  // This would be the most efficient way of getting the DMP data, instead of polling the FIFO
  //success &= (myICM.intEnableDMP(true) == ICM_20948_Stat_Ok);

  // DMP sensor options are defined in ICM_20948_DMP.h
  //    INV_ICM20948_SENSOR_ACCELEROMETER               (16-bit accel)
  //    INV_ICM20948_SENSOR_GYROSCOPE                   (16-bit gyro + 32-bit calibrated gyro)
  //    INV_ICM20948_SENSOR_RAW_ACCELEROMETER           (16-bit accel)
  //    INV_ICM20948_SENSOR_RAW_GYROSCOPE               (16-bit gyro + 32-bit calibrated gyro)
  //    INV_ICM20948_SENSOR_MAGNETIC_FIELD_UNCALIBRATED (16-bit compass)
  //    INV_ICM20948_SENSOR_GYROSCOPE_UNCALIBRATED      (16-bit gyro)
  //    INV_ICM20948_SENSOR_STEP_DETECTOR               (Pedometer Step Detector)
  //    INV_ICM20948_SENSOR_STEP_COUNTER                (Pedometer Step Detector)
  //    INV_ICM20948_SENSOR_GAME_ROTATION_VECTOR        (32-bit 6-axis quaternion)
  //    INV_ICM20948_SENSOR_ROTATION_VECTOR             (32-bit 9-axis quaternion + heading accuracy)
  //    INV_ICM20948_SENSOR_GEOMAGNETIC_ROTATION_VECTOR (32-bit Geomag RV + heading accuracy)
  //    INV_ICM20948_SENSOR_GEOMAGNETIC_FIELD           (32-bit calibrated compass)
  //    INV_ICM20948_SENSOR_GRAVITY                     (32-bit 6-axis quaternion)
  //    INV_ICM20948_SENSOR_LINEAR_ACCELERATION         (16-bit accel + 32-bit 6-axis quaternion)
  //    INV_ICM20948_SENSOR_ORIENTATION                 (32-bit 9-axis quaternion + heading accuracy)

  // Enable the DMP Game Rotation Vector sensor
  success &= (myICM.enableDMPSensor(INV_ICM20948_SENSOR_GAME_ROTATION_VECTOR) == ICM_20948_Stat_Ok);

  // Enable any additional sensors / features
  //success &= (myICM.enableDMPSensor(INV_ICM20948_SENSOR_RAW_GYROSCOPE) == ICM_20948_Stat_Ok);
  //success &= (myICM.enableDMPSensor(INV_ICM20948_SENSOR_RAW_ACCELEROMETER) == ICM_20948_Stat_Ok);
  //success &= (myICM.enableDMPSensor(INV_ICM20948_SENSOR_MAGNETIC_FIELD_UNCALIBRATED) == ICM_20948_Stat_Ok);

  // Configuring DMP to output data at multiple ODRs:
  // DMP is capable of outputting multiple sensor data at different rates to FIFO.
  // Setting value can be calculated as follows:
  // Value = (DMP running rate / ODR ) - 1
  // E.g. For a 5Hz ODR rate when DMP is running at 55Hz, value = (55/5) - 1 = 10.
  success &= (myICM.setDMPODRrate(DMP_ODR_Reg_Quat6, 0) == ICM_20948_Stat_Ok); // Set to the maximum
  //success &= (myICM.setDMPODRrate(DMP_ODR_Reg_Accel, 0) == ICM_20948_Stat_Ok); // Set to the maximum
  //success &= (myICM.setDMPODRrate(DMP_ODR_Reg_Gyro, 0) == ICM_20948_Stat_Ok); // Set to the maximum
  //success &= (myICM.setDMPODRrate(DMP_ODR_Reg_Gyro_Calibr, 0) == ICM_20948_Stat_Ok); // Set to the maximum
  //success &= (myICM.setDMPODRrate(DMP_ODR_Reg_Cpass, 0) == ICM_20948_Stat_Ok); // Set to the maximum
  //success &= (myICM.setDMPODRrate(DMP_ODR_Reg_Cpass_Calibr, 0) == ICM_20948_Stat_Ok); // Set to the maximum

  // Enable the FIFO
  success &= (myICM.enableFIFO() == ICM_20948_Stat_Ok);

  // Enable the DMP
  success &= (myICM.enableDMP() == ICM_20948_Stat_Ok);

  // Reset DMP
  success &= (myICM.resetDMP() == ICM_20948_Stat_Ok);

  // Reset FIFO
  success &= (myICM.resetFIFO() == ICM_20948_Stat_Ok);


  return success;
}

double getYaw(ICM_20948_I2C myICM)
{
  icm_20948_DMP_data_t data;
  myICM.readDMPdataFromFIFO(&data);
  
  if(( myICM.status == ICM_20948_Stat_Ok ) || ( myICM.status == ICM_20948_Stat_FIFOMoreDataAvail )) // Was valid data available?
  {

    
    if ( (data.header & DMP_header_bitmap_Quat6) > 0 ) // We have asked for GRV data so we should receive Quat6
    {
      double q1 = ((double)data.Quat6.Data.Q1) / 1073741824.0; // Convert to double. Divide by 2^30
      double q2 = ((double)data.Quat6.Data.Q2) / 1073741824.0; // Convert to double. Divide by 2^30
      double q3 = ((double)data.Quat6.Data.Q3) / 1073741824.0; // Convert to double. Divide by 2^30

      // Convert the quaternions to Euler angles (roll, pitch, yaw)
      // https://en.wikipedia.org/w/index.php?title=Conversion_between_quaternions_and_Euler_angles&section=8#Source_code_2
      
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
      double yawAngle = atan2(t3, t4) * 180.0 / PI;

      //Serial.print(F("Roll:"));
      //Serial.print(roll, 1);
      //Serial.print(F(" Pitch:"));
      //Serial.print(pitch, 1);
      //Serial.print(F(" Yaw:"));
      if(yawAngle < 0)
        yawAngle+=360;
        
      return yawAngle;
    }
  }
}

int intToStr(int x, char str[], int d) 
{ 
    int i = 0;
    if(x < 0)
    {
      str[i++]='-';
      x = -x; 
    }
    while (x) { 
        str[i++] = (x % 10) + '0'; 
        x = x / 10; 
    } 
    while (i < d) 
        str[i++] = '0'; 
  
    reverse(str, i); 
    str[i] = '\0'; 
    return i; 
} 
  
void ftoa(float n, char* res, int afterpoint) 
{ 
    int ipart = (int)n; 
    float fpart = n - (float)ipart; 
  

    int i = intToStr(ipart, res, 0); 
  
    if (afterpoint != 0) { 
        res[i] = '.';
        fpart = fpart * pow(10, afterpoint); 
        intToStr((int)fpart, res + i + 1, afterpoint); 
    } 
}


void printPaddedInt16b( int16_t val ){
  if(val > 0){
    Serial.print(" ");
    if(val < 10000){ Serial.print("0"); }
    if(val < 1000 ){ Serial.print("0"); }
    if(val < 100  ){ Serial.print("0"); }
    if(val < 10   ){ Serial.print("0"); }
  }else{
    Serial.print("-");
    if(abs(val) < 10000){ Serial.print("0"); }
    if(abs(val) < 1000 ){ Serial.print("0"); }
    if(abs(val) < 100  ){ Serial.print("0"); }
    if(abs(val) < 10   ){ Serial.print("0"); }
  }
  Serial.print(abs(val));
}

void printRawAGMT( ICM_20948_AGMT_t agmt){
  Serial.print("RAW. Acc [ ");
  printPaddedInt16b( agmt.acc.axes.x );
  Serial.print(", ");
  printPaddedInt16b( agmt.acc.axes.y );
  Serial.print(", ");
  printPaddedInt16b( agmt.acc.axes.z );
  Serial.print(" ], Gyr [ ");
  printPaddedInt16b( agmt.gyr.axes.x );
  Serial.print(", ");
  printPaddedInt16b( agmt.gyr.axes.y );
  Serial.print(", ");
  printPaddedInt16b( agmt.gyr.axes.z );
  Serial.print(" ], Mag [ ");
  printPaddedInt16b( agmt.mag.axes.x );
  Serial.print(", ");
  printPaddedInt16b( agmt.mag.axes.y );
  Serial.print(", ");
  printPaddedInt16b( agmt.mag.axes.z );
  Serial.print(" ], Tmp [ ");
  printPaddedInt16b( agmt.tmp.val );
  Serial.print(" ]");
  Serial.println();
}


void printFormattedFloat(float val, uint8_t leading, uint8_t decimals){
  float aval = abs(val);
  if(val < 0){
    Serial.print("-");
  }else{
    Serial.print(" ");
  }
  for( uint8_t indi = 0; indi < leading; indi++ ){
    uint32_t tenpow = 0;
    if( indi < (leading-1) ){
      tenpow = 1;
    }
    for(uint8_t c = 0; c < (leading-1-indi); c++){
      tenpow *= 10;
    }
    if( aval < tenpow){
      Serial.print("0");
    }else{
      break;
    }
  }
  if(val < 0){
    Serial.print(-val, decimals);
  }else{
    Serial.print(val, decimals);
  }
}


void printScaledAGMT( ICM_20948_I2C *sensor ){
  Serial.print("Scaled. Acc (mg) [ ");
  printFormattedFloat( sensor->accX(), 5, 2 );
  Serial.print(", ");
  printFormattedFloat( sensor->accY(), 5, 2 );
  Serial.print(", ");
  printFormattedFloat( sensor->accZ(), 5, 2 );
  Serial.print(" ], Gyr (DPS) [ ");
  printFormattedFloat( sensor->gyrX(), 5, 2 );
  Serial.print(", ");
  printFormattedFloat( sensor->gyrY(), 5, 2 );
  Serial.print(", ");
  printFormattedFloat( sensor->gyrZ(), 5, 2 );
  Serial.print(" ], Mag (uT) [ ");
  printFormattedFloat( sensor->magX(), 5, 2 );
  Serial.print(", ");
  printFormattedFloat( sensor->magY(), 5, 2 );
  Serial.print(", ");
  printFormattedFloat( sensor->magZ(), 5, 2 );
  Serial.print(" ], Tmp (C) [ ");
  printFormattedFloat( sensor->temp(), 5, 2 );
  Serial.print(" ]");
  Serial.println();
}

void displayGpsInfo(TinyGPSPlus gps)
{
  Serial.print(F("Location: ")); 
  if (gps.location.isValid())
  {
    Serial.print(gps.location.lat(), 6);
    Serial.print(F(","));
    Serial.print(gps.location.lng(), 6);
  }
  else
  {
    Serial.print(F("INVALID"));
  }

  Serial.print(F("  Date/Time: "));
  if (gps.date.isValid())
  {
    Serial.print(gps.date.month());
    Serial.print(F("/"));
    Serial.print(gps.date.day());
    Serial.print(F("/"));
    Serial.print(gps.date.year());
  }
  else
  {
    Serial.print(F("INVALID"));
  }

  Serial.print(F(" "));
  if (gps.time.isValid())
  {
    if (gps.time.hour() < 10) Serial.print(F("0"));
    Serial.print(gps.time.hour());
    Serial.print(F(":"));
    if (gps.time.minute() < 10) Serial.print(F("0"));
    Serial.print(gps.time.minute());
    Serial.print(F(":"));
    if (gps.time.second() < 10) Serial.print(F("0"));
    Serial.print(gps.time.second());
    Serial.print(F("."));
    if (gps.time.centisecond() < 10) Serial.print(F("0"));
    Serial.print(gps.time.centisecond());
  }
  else
  {
    Serial.print(F("INVALID"));
  }

  Serial.println();
}

float getPressure(MPL3115A2 mplSensor)
{
  return mplSensor.readPressure();
}

float getTemp(MPL3115A2 mplSensor)
{
  return mplSensor.readTemp();
}

float getAltitude(MPL3115A2 mplSensor, float pressure)
{
  //local pressure at sea level in hPa
  float lPressure = 101325;
  //magic formula from https://www.weather.gov/media/epz/wxcalc/pressureAltitude.pdf
  float altitude = ((1 - pow(pressure / lPressure, 0.1902632)) * 44330.77);
  return altitude;
}


void logData(char *fileName, double voltage, float pressure, float altitude, float maxAltitude, float temperture, double latitude, double longitude, double altitudeGps, double distanceFromReference, uint8_t hour, uint8_t minute, uint8_t second, double yaw, int encoder1, int encoder2, missionState state)
{
  File logFile = SD.open(fileName, FILE_WRITE);
  
  if (hour < 10) Serial.print("0");
  Serial.print(hour);
  Serial.print(":");
  if (minute < 10) Serial.print("0");
  Serial.print(minute);
  Serial.print(":");
  if (second < 10) Serial.print("0");
  Serial.print(second);
  Serial.print(",");
  Serial.print(voltage);
  Serial.print(",");
  Serial.print(encoder1);
  Serial.print(",");
  Serial.print(encoder2);
  Serial.print(",");
  Serial.print(pressure);
  Serial.print(",");
  Serial.print(altitude);
  Serial.print(",");
  Serial.print(maxAltitude);
  Serial.print(",");
  Serial.print(temperture);
  Serial.print(",");
  Serial.print(latitude,6);
  Serial.print(",");
  Serial.print(longitude,6);
  Serial.print(",");
  Serial.print(altitudeGps);
  Serial.print(",");
  Serial.print(distanceFromReference);
  Serial.print(",");
  Serial.print(yaw);
  Serial.print(",");
  Serial.println(state);

  if(logFile)
  {
    if (hour < 10) logFile.print("0");
    logFile.print(hour);
    logFile.print(":");
    if (minute < 10) logFile.print("0");
    logFile.print(minute);
    logFile.print(":");
    if (second < 10) logFile.print("0");
    logFile.print(second);
    logFile.print(",");
    logFile.print(voltage);
    logFile.print(",");
    logFile.print(encoder1);
    logFile.print(",");
    logFile.print(encoder2);
    logFile.print(",");
    logFile.print(pressure);
    logFile.print(",");
    logFile.print(altitude);
    logFile.print(",");
    logFile.print(maxAltitude);
    logFile.print(",");
    logFile.print(temperture);
    logFile.print(",");
    logFile.print(latitude,6);
    logFile.print(",");
    logFile.print(longitude,6);
    logFile.print(",");
    logFile.print(altitudeGps);
    logFile.print(",");
    logFile.print(distanceFromReference);
    logFile.print(",");
    logFile.print(yaw);
    logFile.print(",");
    logFile.println(state);
    
    logFile.close();
    
  }


  
}
