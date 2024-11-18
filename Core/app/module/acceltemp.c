/**
 * @file acceltemp.c
 *
 * @date Created on: Oct 14, 2024
 * @author Jerry Ukwela (jeu6@case.edu)
 */

#include "main.h"
#include "acceltemp.h"
#include "ObjDict.h"

#include "app.h"
// -------- DEFINITIONS ----------


// --------   DATA   ------------


// -------- PROTOTYPES ----------

static void enableISPU();
static void configISPU( UNS8 memsel, UNS16 addr, const UNS8* data, UNS16 len);
static void confirmISPU( UNS8 memsel, UNS16 addr, const UNS8* data, UNS16 len);

static UNS8 atanT( UNS16 x);
static UNS8 intsqrt( UNS16 x);

static UNS8 modeIMU = IMU_ACC_ENABLED;
static UNS8 configureISPU = 1; //0=don't configure, 1=configure data and program, 2=configure data

//float Gx, Gy, Gz, Ax, Ay, Az; // Values for accelerometer and gyro

//============================
//    GLOBAL CODE
//============================

void putAccelRegSingle(UNS8 regNo, UNS8 data){
	HAL_I2C_Mem_Write(&hi2c2, ACC_ADDR, regNo, 1, &data, 1, 25);
}

UNS8 getAccelRegSingle(UNS8 regNo){
	UNS8 data;

	HAL_I2C_Mem_Read(&hi2c2, ACC_ADDR, regNo, 1, &data, 1, 25);
	return data;
}

void putAccelReg(UNS8 regNo, UNS8 data, UNS8 length){
	HAL_I2C_Mem_Write(&hi2c2, ACC_ADDR, regNo, 1, &data, length, 25);
}

void getAccelReg(UNS8 regNo, UNS8* data, UNS8 length){
	HAL_I2C_Mem_Read(&hi2c2, ACC_ADDR, regNo, 1, data, length, 25);
}


/**
 * @ingroup accelerometer
 * @brief puts Accelerometer to sleep
 */
void sleepAccelerometer( void )
{
	putAccelRegSingle(CTRL1_XL, 0x00);	// Accel
	putAccelRegSingle(CTRL2_G, 0x00);	// Gyro
    putAccelRegSingle(CTRL9_C, 0); 		// ISPU
}

/**
 * @ingroup accelerometer
 * @brief Changes functional mode of ISM330IS accelerometer
 * @param mode Mode to configure the IMU into
 */
void changeModeIMU (UNS8 mode){
	 if(mode & IMU_ISPU_ENABLED)
	  {
	    if(mode & IMU_RELOAD_ISPU)
	    {
	      configureISPU = 1;
	    }
	    else if(mode & IMU_RELOAD_ISPU_DATA)
	    {
	      configureISPU = 2;
	    }
	    else if(mode & IMU_ISPU_RST)
	    {
	      configureISPU = 3;
	    }
	    enableISPU();
	  }
	  else
	  {
	    putAccelRegSingle( FUNC_CFG_ACCESS, 0x00 ); //restore normal register access

	    putAccelRegSingle( CTRL9_C, 0 ); //powerdown ISPU
	    putAccelRegSingle( CTRL6_C, 0x10 ); //low power mode Acc
	    putAccelRegSingle( CTRL7_G, 0x80 ); //low power mode Gyro
	    if(mode & IMU_ACC_ENABLED)
	    {
	      putAccelRegSingle( CTRL1_XL, 0x20); //26Hz, 2g, Acc
	    }
	    else
	    {
	      putAccelRegSingle( CTRL1_XL, 0x00); //Powerdown Acc
	    }
	    if(mode & IMU_GYRO_ENABLED)
	    {
	      putAccelRegSingle( CTRL2_G, 0x20); //26Hz, +/-250dps, Gyro
	    }
	    else
	    {
	      putAccelRegSingle( CTRL2_G, 0x00); //Powerdown Gyro
	    }
	  }
	  modeIMU = mode;
}

/**
 * @ingroup accelerometer
 * @brief Initializes Accelerometer
 */
void initAccelerometer(void){
	changeModeIMU(modeIMU);
}

/**
 * @ingroup accelerometer
 * @brief update Accelerometer
 */
void updateAccelerometer( void )
{
  //takes 2.9ms without average depth
  //      4.0ms average depth =1
  //      6.1ms average depth=20
  //      8.4ms tilt calculation with average depth = 1
	static UNS8 taskDelayMSecs = 50;
	static UNS32 tDelayRef = 0; //JML: set to 0, so timesout first time through

	if( !isTimedOut( &tDelayRef, TIMEOUT_ms( taskDelayMSecs ) ) )
	{
		return;
	}

	//ELSE.. reading delay is over

	resetTimeOut( &tDelayRef );
	/* read accelerometer and gyro x,y,z */

	UNS8 accelData[6];
	UNS8 tempData[2];
	UNS8 gyroData[6];
	UNS8 ispuData[16];

    static INTEGER8 xBuf[ACCEL_BUFFER_LENGTH];
    static INTEGER8 yBuf[ACCEL_BUFFER_LENGTH];
    static INTEGER8 zBuf[ACCEL_BUFFER_LENGTH];
    static UNS8 cnt = 0;

    UNS8 depth = AccelerometerSettings & ACCEL_AVERAGE_DEPTH_BITS;

    if ((modeIMU & IMU_ISPU_ENABLED) && ((modeIMU & IMU_ACC_ENABLED) || (modeIMU & IMU_GYRO_ENABLED) || (modeIMU & IMU_TEMP_ENABLED)))
     {
       /* restore normal register access */
       putAccelRegSingle( FUNC_CFG_ACCESS, 0x00 );
     }
     if (modeIMU & IMU_ACC_ENABLED)
     {
       /* read accelerometer x,y,z (lowbyte highbyte each)*/
       getAccelReg( OUTX_L_A, accelData, 6 );
     }
     if (modeIMU & IMU_GYRO_ENABLED)
     {
       /* read gyro x,y,z (lowbyte highbyte each)*/
       getAccelReg( OUTX_L_G, gyroData, 6 );
     }
     if (modeIMU & IMU_TEMP_ENABLED)
     {
       /* read gyro x,y,z (lowbyte highbyte each)*/
       getAccelReg( OUT_TEMP_L, tempData, 2 );
     }

     if ((modeIMU & IMU_ISPU_ENABLED) && ((modeIMU & IMU_ACC_ENABLED) || (modeIMU & IMU_GYRO_ENABLED) || (modeIMU & IMU_TEMP_ENABLED)))
     {
       /* ISPU register access */
       putAccelRegSingle( FUNC_CFG_ACCESS, 0x80 );
     }
     if (modeIMU & IMU_ISPU_ENABLED)
     {
       /* read quaternion from ISPU (4 byte each x,y,z,w)*/
       getAccelReg( ISPU_DOUT_00_L, ispuData, 16 );
     }

	//JML TODO: use all 16-bits - currently just high bytefor backwards compatibility
     if (modeIMU & IMU_ACC_ENABLED)
     {
       Accelerometers[0] = accelData[1] * (modeIMU & IMU_ISPU_ENABLED ? 4 : 1) + 128; //ISPU Changes from 2g range to 8g range, so the output needs to be multiplied by 4 (loses precision)
       Accelerometers[1] = accelData[3] * (modeIMU & IMU_ISPU_ENABLED ? 4 : 1) + 128;
       Accelerometers[2] = accelData[5] * (modeIMU & IMU_ISPU_ENABLED ? 4 : 1) + 128;
       Accelerometers[3]++;
     }
     if (modeIMU & IMU_GYRO_ENABLED)
     {
         memcpy(&Gyroscopes, gyroData, sizeof(gyroData));
     }
     if (modeIMU & IMU_TEMP_ENABLED)
     {
         memcpy(&TemperatureIMU, tempData, sizeof(tempData));
     }
     if (modeIMU & IMU_ISPU_ENABLED)
     {
        memcpy(&Quaternion, ispuData, sizeof(ispuData));
     }

     if (depth > 0)
     {
       UNS8 i;
       INTEGER16 sumX=0, sumY=0, sumZ=0;
       INTEGER8 x=0,y=0,z=0;

       if(depth > ACCEL_BUFFER_LENGTH)
         depth = ACCEL_BUFFER_LENGTH;

       //Shift values in buffer
       //destination, source
       memmove( &xBuf[1], &xBuf[0], depth-1);
       memmove( &yBuf[1], &yBuf[0], depth-1);
       memmove( &zBuf[1], &zBuf[0], depth-1);

       //accelData is 0 to 256
       //Add new value at beginning of buffer
       //ignore bottom two bits and convert to signed (-32 to 31 range)
       xBuf[0]=(INTEGER8)(accelData[0]>>2) - 32;
       yBuf[0]=(INTEGER8)(accelData[1]>>2) - 32;
       zBuf[0]=(INTEGER8)(accelData[2]>>2) - 32;

       if(cnt<depth) //if we haven't yet reached sampled depth
       {
         cnt++;
         depth = cnt;
       }

       for(i=0; i<depth; i++)
       {
         sumX += xBuf[i];
         sumY += yBuf[i];
         sumZ += zBuf[i];
       }
       x =(INTEGER8)((sumX<<2)/depth);
       y =(INTEGER8)((sumY<<2)/depth);
       z =(INTEGER8)((sumZ<<2)/depth);
       //in order to make the division more accurate we shift the sum left two bits before dividing
       //outcome is now -128 to 127
       AccelerometersFiltered[0] = (UNS8)(x + 128);
       AccelerometersFiltered[1] = (UNS8)(y + 128);
       AccelerometersFiltered[2] = (UNS8)(z + 128);
       AccelerometersFiltered[3] = Accelerometers[3];



       if(AccelerometerSettings & ACCEL_CALC_TILT_BIT)
       {
         //calculate squares in advance
         UNS16 xx = x*x;
         UNS16 yy = y*y;
         UNS16 zz = z*z;

         UNS16 yyzz = yy+zz;
         UNS16 xxzz = xx+zz;
         UNS16 xxyy = xx+yy;


         if( x < 0 )
         {
           if(yyzz==0) //avoid divide by zero
             AccelerometersTilt[0] = -90;
           else
             AccelerometersTilt[0] = - (INTEGER8) atanT(    ((UNS16)(-x)<<8)          /  intsqrt(yyzz));
         }
         else
         {
           if(yyzz==0) //avoid divide by zero
             AccelerometersTilt[0] = 90;
           else
             AccelerometersTilt[0] =   (INTEGER8) atanT(    ((UNS16)( x)<<8)          /  intsqrt(yyzz));
         }
         if( y < 0 )
         {
           if(xxzz==0) //avoid divide by zero
             AccelerometersTilt[1] = -90;
           else
             AccelerometersTilt[1] = - (INTEGER8) atanT(    ((UNS16)(-y)<<8)          /  intsqrt(xxzz));
         }
         else
         {
           if(xxzz==0) //avoid divide by zero
             AccelerometersTilt[1] = 90;
           else
             AccelerometersTilt[1] =  (INTEGER8) atanT(    ((UNS16)( y)<<8)          /  intsqrt(xxzz));
         }
         if(z == 0) //avoid divide by zero
           AccelerometersTilt[2] = 90;
         else if(z < 0)
           AccelerometersTilt[2] = - (INTEGER8) atanT( ((UNS16)intsqrt(xxyy)<<8)   /    (UNS8)(-z) );
         else
           AccelerometersTilt[2] =   (INTEGER8) atanT( ((UNS16)intsqrt(xxyy)<<8)   /    (UNS8)( z) );

         AccelerometersTilt[3] = Accelerometers[3];
       }
     }
}

void configISPU( UNS8 memsel, UNS16 addr, const UNS8* data, UNS16 len)
{
	UNS8 d;

	//TODO: confirm ISPU access, and combine writes below into single write
	putAccelRegSingle( ISPU_MEM_SEL, memsel ); //select data (0) or program mem (1)
	putAccelRegSingle( ISPU_MEM_ADDR1, (UNS8) (addr >> 8)); //high byte
	putAccelRegSingle( ISPU_MEM_ADDR0, (UNS8) (addr & 0xFF) ); //low byte

	while(len--)
	{
		d = *data++;
		putAccelRegSingle( ISPU_MEM_DATA, d );
	}
}

void confirmISPU( UNS8 memsel, UNS16 addr, const UNS8* data, UNS16 len)
{
	return; //XXX: Currently returns becaue this function does nothing useful

	UNS8 d = 0;
	//TODO: confirm ISPU access, and combine writes below into single write
	putAccelRegSingle( ISPU_MEM_SEL, 0x40+memsel ); //select data (0) or program mem (1) and read mem enabled
	putAccelRegSingle( ISPU_MEM_ADDR1, (UNS8) (addr >> 8)); //high byte
	putAccelRegSingle( ISPU_MEM_ADDR0, (UNS8) (addr & 0xFF) ); //low byte

	while(len--)
	{
	    d = getAccelRegSingle( ISPU_MEM_DATA);  //TODO: do something with the output
	}
}


void enableISPU(){

	if(configureISPU == 1)
	{
	  if(getAccelRegSingle( WHO_AM_I) == 0x22) //
	  {
		  putAccelRegSingle( CTRL3_C, 0x04 ); //enable BDU, IF_INC
		  putAccelRegSingle( CTRL3_C, 0x44 ); //enable BDU, IF_INC
		  getAccelRegSingle( CTRL3_C); //expect 0x44

		  //clear
		  putAccelRegSingle( CTRL6_C, 0 );    //reg0x15
		  putAccelRegSingle( CTRL1_XL, 0);   //reg0x10
		  putAccelRegSingle( CTRL2_G, 0);   //reg0x11
		  putAccelRegSingle( CTRL7_G, 0);   //reg0x17
		  putAccelRegSingle( INT1_CTRL, 0 );  //reg0x0D

		  putAccelRegSingle( CTRL2_G, 0x48); //104Hz, 1000dps  Gyro
		  putAccelRegSingle( CTRL1_XL, 0x4C ); //104Hz, 8g  Accel

		  getAccelRegSingle( CTRL1_XL ); //expect 0x4C
		  getAccelRegSingle( CTRL6_C ); //expect 0
		  getAccelRegSingle( CTRL2_G ); //expect 0x48
		  getAccelRegSingle( CTRL7_G);  //expect 0
	  }
	  else
	  {
		configureISPU = 0;
	  }
	}
	if(configureISPU == 1 || configureISPU == 2 || configureISPU == 3)
	{
		//ISPU Config
		putAccelRegSingle( FUNC_CFG_ACCESS, 2 ); //reset  ISPU
	}
	if(configureISPU == 1 || configureISPU == 2)
	{
		putAccelRegSingle( FUNC_CFG_ACCESS, 0 ); //clear
		putAccelRegSingle( FUNC_CFG_ACCESS, 0x80 ); //ISPU register access
		configISPU(0, 0x0000, ispu_data, sizeof(ispu_data));
	}
	if(configureISPU == 1)
	{
		configISPU(1, 0x0000, ispu_bank1, sizeof(ispu_bank1));
		configISPU(1, 0x2000, ispu_bank2, sizeof(ispu_bank2));
		configISPU(1, 0x4000, ispu_bank3, sizeof(ispu_bank3));

		//Read back all registers
		confirmISPU(0, 0x0000, ispu_data, sizeof(ispu_data));
		confirmISPU(1, 0x0000, ispu_bank1, sizeof(ispu_bank1));
		confirmISPU(1, 0x2000, ispu_bank2, sizeof(ispu_bank2));
		confirmISPU(1, 0x4000, ispu_bank3, sizeof(ispu_bank3));
	}

	configureISPU = 0;

	putAccelRegSingle( FUNC_CFG_ACCESS, 0); //restore normal register access
	putAccelRegSingle( CTRL9_C, 0 ); //powerdown ISPU
	putAccelRegSingle( CTRL10_C, 0 );
	putAccelRegSingle( CTRL1_XL, 0x10 );

	putAccelRegSingle( FUNC_CFG_ACCESS, 0x80 ); //ISPU register access
	putAccelRegSingle( ISPU_CONFIG,  0x01 ); //ISPU_RST_N

	putAccelRegSingle( FUNC_CFG_ACCESS, 0 );  //restore normal register access

	//Delay 5ms?

	putAccelRegSingle( CTRL1_XL, 0 ); //powerdown acc
	//putAccelRegSingle( INT2_CTRL, 0x80);  //ISPU sleep state
	putAccelRegSingle( INT2_CTRL, 0x00);  //ISPU sleep state  JML FIX for GND-INT2 errata
	putAccelRegSingle( CTRL6_C, 0 ); //high performance mode Acc
	putAccelRegSingle( CTRL7_G, 0 ); //high performance mode Gyro
	putAccelRegSingle( CTRL9_C, 0x40); //ISPU rate = 104Hz
	putAccelRegSingle( MD1_CFG, 0x02); //INT1_ISPU

	putAccelRegSingle( FUNC_CFG_ACCESS, 0x80); //ISPU register access
	putAccelRegSingle( ISPU_CONFIG, 0x01); //ISPU_RST_N
	//putAccelRegSingle( ISPU_INT1_CTRL0, 0x01); //#1
	putAccelRegSingle( ISPU_INT1_CTRL0, 0x00); //JML FIX for GND-INT2 errata
	putAccelRegSingle( ISPU_INT2_CTRL0, 0x00); //JML FIX for GND-INT2 errata
	putAccelRegSingle( ISPU_ALGO0, 0x01); //#1

	putAccelRegSingle( FUNC_CFG_ACCESS, 0 ); //restore normal register access
	putAccelRegSingle( CTRL1_XL, 0x4C); //104Hz, 8g, A
	putAccelRegSingle( CTRL2_G, 0x4C); //104Hz, 2000dps, G

	putAccelRegSingle( FUNC_CFG_ACCESS, 0x80); //ISPU register access
}

/**
 * @ingroup temp
 * @brief puts thermometer to sleep
 */

void sleepTemperature( void )
{
	UNS8 sendData = 1;
	HAL_I2C_Mem_Write(&hi2c2, TEMP_ADDR, T_CONF_REG, 1, &sendData, 1, 50);
	//TODO: Implement
}

/**
 * @ingroup temp
 * @brief initializes thermometer to default configuration
 */

void initTemperature( void )
{

}

/**
 * @ingroup temp
 * @brief Gets temperature reading via I2C
 */
void updateTemperature( void )
{
	static UNS8 taskDelaySecs = 1;		//^^fornow. later read from ObjDict; 0=off
    static UNS32 tDelayRef = 0;//JML: set to 0, so timesout first time through;
    UNS8 tempData[2];
	INTEGER16 degrC;

	if( !isTimedOut( &tDelayRef, TIMEOUT_sec( taskDelaySecs ) ) )
	{
		return;
	}

	//ELSE.. reading delay is over

	resetTimeOut( &tDelayRef );

	/* read temp */
	if(HAL_I2C_Mem_Read(&hi2c2, TEMP_ADDR, T_TEMP_REG, 1, tempData, 2, 25) != HAL_OK)
	{
		return;
	}

	/* temp is measured in 1/16 degC above 0deg C */

	/* save temperature reading in 10ths degC */

	//Temperature = ((UNS16)tempData[0] << 4) + ((UNS16)tempData[1] >> 4 ); //Raw Reading

	degrC = ( (  (INTEGER32)( ((UNS16)tempData[0] << 4) + ((UNS16)tempData[1] >> 4 ) ) )* 160 ) >> 8 ;

	if( degrC > MAX_TEMPR )
	{
		degrC = MAX_TEMPR;
	}
	else if( degrC < MIN_TEMPR )
	{
		degrC = MIN_TEMPR;
	}

	Temperature = (UNS8)degrC;
}

void initDiagnostics(void){

}


/**
 * @ingroup diagnostics
 * @brief If disgnostics is enabled, update their entries in the OD
 */
void updateDiagnostics( void )
{
	static UNS8 taskDelayMSecs = 100;
	static UNS32 tDelayRef = 0; //JML: set to 0, so timesout first time through
	static UNS8 ch = 0;
	uint8_t ADCres[4];

	if(DiagnosticsEnabled)
	{
	  if( !isTimedOut( &tDelayRef, TIMEOUT_ms( taskDelayMSecs ) ) )
	  {
		  return;
	  }

	  //ELSE.. reading delay is over
	  resetTimeOut( &tDelayRef );

	  HAL_ADC_Start_DMA(&hadc1, &ADCres, 4);	// Sample 4 enabled ADC channels and transfer results to ADCres through DMA
	  HAL_ADC_PollForConversion(&hadc1, 1);	// Need time for DMA to transfer

	  //set ch for next measurement
	  switch(ch)
	  {
		case 0:
			Diagnostic_VIC = ADCres[2]; // Channel 3
			ch=1;
			break;
		case 1:
			Diagnostic_VIN = ADCres[1]; // Channel 2
			ch=3; //skip VOS, not currently configured
			break;
		case 2:
			Diagnostic_VOS = ADCres[0]; // Channel 1
			ch=3;
			break;
		case 3:
			Diagnostic_VDD = ADCres[3]; // Channel 4
			ch=0;
			break;
		default: ch=0;
	  }
	}
}

//============================
//    LOCAL CODE
//============================

static UNS8 atanT( UNS16 x)
{
    UNS8 imax=NTAN;
    UNS8 imin=0;
    UNS8 i=0;
    UNS8 delta=NTAN;

    while(delta>1)
    {
        i=imin+(delta>>1);

        if(x==TAN[i])
            return i;//-TAN_IOFFSET;
        else if(x<TAN[i])
            imax = i;
        else
            imin = i;

        delta = imax-imin;
    }
    return imin;//-TAN_IOFFSET;
}

static UNS8 intsqrt(UNS16 val) {
    UNS8 mulMask = 0x0040;
    UNS8 retVal = 0;
    if (val > 8100)
      return 90;

    while (mulMask != 0) {
        retVal |= mulMask;
        if ((retVal * retVal) > val) {
            retVal &= ~mulMask;
        }

        mulMask >>= 1;
    }
    return retVal;
}
