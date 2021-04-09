#ifndef TLC59108_H
#define TLC59108_H
#include <Wire.h>

struct rgbColor
{
  int r, g, b;
  void setColor(int red, int green, int blue)
  {
    r = red;
    g = green;
    b = blue;
  }
  
};


// default I2C addresses
// datasheet, pp 12-13
struct I2C_ADDR
{
	static const byte BASE = 0x40;
	static const byte SWRESET = 0x4b;
	static const byte ALLCALL = 0x48;
	static const byte SUB1 = 0x49;
	static const byte SUB2 = 0x4a;
	static const byte SUB3 = 0x4c;
};

// register auto-increment modes for setting multiple registers
// datasheet, p 13
struct AUTO_INCREMENT
{
	static const byte ALL = 0x80; // increment through all registers (for initial setup)
	static const byte IND = 0xa0; // increment through individual brightness registers
	static const byte GLOBAL = 0xc0; // increment through global control registers
	static const byte INDGLOBAL = 0xe0; // increment through individual and global registers
};

struct LED_MODE
{
	static const byte OFF = 0;
	static const byte FULL_ON = 1;
	static const byte PWM_IND = 2;
	static const byte PWM_INDGRP = 3;
};

  struct REGISTER
  {
	struct MODE1
	{
	  static const byte ADDR = 0x00;

	  static const byte OSC_OFF = 0x10;
	  static const byte SUB1 = 0x08;
	  static const byte SUB2 = 0x04;
	  static const byte SUB3 = 0x02;
	  static const byte ALLCALL = 0x01;
	};

	struct MODE2
	{
	  static const byte ADDR = 0x01;

	  static const byte EFCLR = 0x80;
	  static const byte DMBLNK = 0x20;
	  static const byte OCH = 0x08;
	};

	struct PWM0
	{
	  static const byte ADDR = 0x02;
	};

	struct PWM1
	{
	  static const byte ADDR = 0x03;
	};

	struct PWM2
	{
	  static const byte ADDR = 0x04;
	};

	struct PWM3
	{
	  static const byte ADDR = 0x05;
	};

	struct PWM4
	{
	  static const byte ADDR = 0x06;
	};

	struct PWM5
	{
	  static const byte ADDR = 0x07;
	};

	struct PWM6
	{
	  static const byte ADDR = 0x08;
	};

	struct PWM7
	{
	  static const byte ADDR = 0x09;
	};

	struct GRPPWM
	{
	  static const byte ADDR = 0x0a;
	};

	struct GRPFREQ
	{
	  static const byte ADDR = 0x0b;
	};

	struct LEDOUT0
	{
	  static const byte ADDR = 0x0c;
	};

	struct LEDOUT1
	{
	  static const byte ADDR = 0x0d;
	};

	struct SUBADR1
	{
	  static const byte ADDR = 0x0e;
	};

	struct SUBADR2
	{
	  static const byte ADDR = 0x0f;
	};

	struct SUBADR3
	{
	  static const byte ADDR = 0x10;
	};

	struct ALLCALLADR
	{
	  static const byte ADDR = 0x11;
	};

	struct IREF
	{
	  static const byte ADDR = 0x12;

	  static const byte CM = 0x80; // current multiplier
	  static const byte HC = 0x40; // subcurrent
	};

	struct EFLAG
	{
	  static const byte ADDR = 0x13;
	};
  };

  struct ERROR
  {
	static const uint8_t EINVAL = 2;
		  };

class TLC59108
{
	public:
	
		byte const addr = I2C_ADDR::BASE;
		static const byte NUM_CHANNELS = 8;
		uint8_t resetPin;

		


	
	
		TLC59108(uint8_t hwResetPin)
		{
			resetPin = hwResetPin;
		}
		uint8_t setRegister(const uint8_t reg, const uint8_t value2)
		{
		   Wire.beginTransmission(addr);
		   Wire.write(reg);
		   Wire.write(value2);
		   return Wire.endTransmission();
		}

		uint8_t setRegisters(const uint8_t startReg, const uint8_t values[], const uint8_t numValues)
		{
		  Wire.beginTransmission(addr);
		  Wire.write(startReg | AUTO_INCREMENT::ALL);
		  for(uint8_t i = 0; i < numValues; i++)
			Wire.write(values[i]);
		  return Wire.endTransmission();
		}

		int readRegister(const uint8_t reg)
		{
		   Wire.beginTransmission(addr);
		   Wire.write(reg);
		   if(!Wire.endTransmission())
			 return -1;

		   Wire.requestFrom(addr, (uint8_t) 1);
		   if(Wire.available())
			 return Wire.read();
		   else
			 return -1;
		}

		uint8_t readRegisters(uint8_t *dest, const uint8_t startReg, const uint8_t num){
		  Serial.println("in readRegisters");
		  Wire.beginTransmission(addr);
		  Wire.write(startReg | AUTO_INCREMENT::ALL);
		  if(Wire.endTransmission())
			return 0;

		  uint8_t bytesRead = 0;
		  Wire.requestFrom(addr, num);
		  while(Wire.available() && (bytesRead < num)) {
			(*dest) = (uint8_t) Wire.read();
			dest++;
			bytesRead++;
		  }

		  return bytesRead;
		}

		bool getAllBrightness(uint8_t dutyCycles[]){
		  return (readRegisters(dutyCycles, REGISTER::PWM0::ADDR, NUM_CHANNELS) == NUM_CHANNELS);
		}

		uint8_t init()
		{

		  pinMode(resetPin, OUTPUT);
		  digitalWrite(resetPin, LOW);
		  delay(1);
		  digitalWrite(resetPin, HIGH);
		  delay(1);

		  return setRegister(REGISTER::MODE1::ADDR, REGISTER::MODE1::ALLCALL);
		}

		uint8_t setBrightness(const uint8_t pwmChannel, const uint8_t dutyCycle)
		{
		   if(pwmChannel > 7)
			 return ERROR::EINVAL;

		   return setRegister(pwmChannel + 2, dutyCycle);
		}

		uint8_t setLedOutputMode(const uint8_t outputMode)
		{
		   if(outputMode & 0xfc)
			 return ERROR::EINVAL;

		   byte regValue = (outputMode << 6) | (outputMode << 4) | (outputMode << 2) | outputMode;

		   uint8_t retVal = setRegister(REGISTER::LEDOUT0::ADDR, regValue);
		   retVal &= setRegister(REGISTER::LEDOUT1::ADDR, regValue);
		   return retVal;
		}

		uint8_t setAllBrightness(const uint8_t dutyCycle)
		{
		   Wire.beginTransmission(addr);
		   Wire.write(REGISTER::PWM0::ADDR | AUTO_INCREMENT::IND);
		   for(uint8_t i=0; i<NUM_CHANNELS; i++)
			 Wire.write(dutyCycle);
		   return Wire.endTransmission();
		}
		
		void setRed()
		{
			setAllBrightness(0);
			setBrightness(1, 255);
		}
		
		void setGreen()
		{
			setAllBrightness(0);
			setBrightness(2, 255);
		}
		
		void setBlue()
		{
			setAllBrightness(0);
			setBrightness(3, 255);
		}
		
		void setPurple()
		{
			setAllBrightness(0);
			setBrightness(1, 255);
			setBrightness(3, 255);
		}
		
		void setYellow()
		{
			setAllBrightness(0);
			setBrightness(1, 255);
			setBrightness(2, 100);
		}

   void setRGB(rgbColor color)
   {
      setBrightness(1, color.r);
      setBrightness(2, color.g);
      setBrightness(3, color.b);
      
   }
		


	

};
#endif
