/*****************************************************************
* This code is design to interface with the MLX90621 sensor.
* This sensor is a Low noise High Speed 16x4 Far Infrared array.
* Here is used to obtain data for making a thermal sensor.
* More information is available here: http://www.melexis.com/Infrared-Thermometer-Sensors/Infrared-Thermometer-Sensors/Low-noise-high-speed-16x4-Far-Infrared-array-823.aspx
*
* Pixel position:
* The array consists of 64 IR sensors (also called pixels). Each pixel is identified with its row and
* column position as Pix(i,j) where i is its row number (from 0 to 3) and j is its column number (from 0 to 15)
*
* RAM:
* The on chip 146x16 RAM is accessible for reading via I2C. The RAM is used for storing the results of
* measurements of pixels and Ta sensor and is distributes as follows:
* 64 words for IR sensors. The data is in 2¡¯s complement format
* 1 word for measurement result of PTAT sensor. The data is 16 bit without sign.
* The memory map of the RAM is shown below:
* 0x00=IR Sensor(0,0) result
* 0x01=IR Sensor(1,0) result
* 0x02=IR Sensor(2,0) result
* 0x03=IR Sensor(3,0) result
* 0x04=IR Sensor(0,1) result
* 0x05=IR Sensor(1,1) result
* ...
* 0x3B=IR Sensor(3,14) result
* 0x3C=IR Sensor(0,15) result
* 0x3D=IR Sensor(1,15) result
* 0x3E=IR Sensor(2,15) result
* 0x3F=IR Sensor(3,15) result
*
* Device Addressing:
* For accessing to internal EEPROM: 0x50
* For accessing to IR array data:   0x60
*/

//Libraries to be included
#include <Arduino.h>
#include <Wire.h>

//Begin registers
#define CAL_ACOMMON_L 0xD0
#define CAL_ACOMMON_H 0xD1
#define CAL_ACP_L 0xD3
#define CAL_ACP_H 0xD4
#define CAL_BCP 0xD5
#define CAL_alphaCP_L 0xD6
#define CAL_alphaCP_H 0xD7
#define CAL_TGC 0xD8
#define CAL_AI_SCALE 0xD9
#define CAL_BI_SCALE 0xD9

#define VTH_L 0xDA
#define VTH_H 0xDB
#define KT1_L 0xDC
#define KT1_H 0xDD
#define KT2_L 0xDE
#define KT2_H 0xDF
#define KT_SCALE 0xD2

//Common sensitivity coefficients
#define CAL_A0_L 0xE0
#define CAL_A0_H 0xE1
#define CAL_A0_SCALE 0xE2
#define CAL_DELTA_A_SCALE 0xE3
#define CAL_EMIS_L 0xE4
#define CAL_EMIS_H 0xE5

//Config register = 0xF5-F6
#define OSC_TRIM_VALUE 0xF7

/* Variables */
const byte refreshRate = 1; //Set this value to your desired refresh frequency. Possible values: 0=0.5Hz, 1=1Hz, 2=2Hz, 4=4Hz, 8=8Hz, 16=16Hz, 32=32Hz.   
int16_t irData[64]; //Contains the raw IR data from the sensor
float temperatures[64]; //Contains the calculated temperatures of each pixel in the array

byte eepromData[256]; //Contains the full EEPROM reading from the MLX90621
int16_t k_t1_scale = 0, k_t2_scale = 0, resolution = 0, configuration = 0, cpix = 0;
uint16_t  ptat = 0;
float Tambient = 0;//Tracks the changing ambient temperature of the sensor

void setup()
{
	Serial.begin(115200);
	Wire.begin();
	delay(5);
	readEEPROM();
	writeTrimmingValue();
	setConfiguration();
}

void loop()
{
	if (checkConfig())
	{
		readEEPROM();
		writeTrimmingValue();
		setConfiguration();
	}
	for (int i = 0; i < 16; i++)//Every 16 readings check that the POR flag is not set
	{
		readPTAT(); Serial.print(" PTAT:"); Serial.println(ptat); delay(100);
		readIR();
		calculateTA();
		Serial.print("\nAmbient: "); Serial.println(Tambient);
		readCPIX(); Serial.print("CPIX:"); Serial.println(cpix); delay(100);
		calculateTO();
		Serial.println("Temperatures: ");
		for (int n = 0; n < 64; n++)
		{
			float Temperature = getTemperature(n);
			Serial.print(Temperature);
			if (n == 15 || n == 31 || n == 47 || n == 63) Serial.println();
			else Serial.print(",");
		}
		Serial.println();
		delay(10000);
	}
}

void readEEPROM()
{
	int i = 0;
	Serial.print("\nReading EEPPROM...");
	for (int j = 0; j < 256; j = j + 32)
	{
		Wire.beginTransmission(0x50);
		Wire.write((byte)j);
		//Wire.endTransmission(false); // use repeated start to get answer   
		Wire.requestFrom(0x50, 32);
		i = j;
		Serial.print("\n["); Serial.print(j); Serial.print("] ");
		while (Wire.available())
		{ // slave may send less than requested
			eepromData[i] = (byte)Wire.read();
			Serial.print(eepromData[i]); Serial.print(" ");
			i++;
		}
		Wire.endTransmission();
	}
	Serial.println(" ");
}

void writeTrimmingValue() 
{
	Serial.print("\nWriting Trimming Value...");
	Wire.beginTransmission(0x60);
	Wire.write(0x04);
	Wire.write((byte)eepromData[OSC_TRIM_VALUE] - 0xAA);
	Wire.write(eepromData[OSC_TRIM_VALUE]);//eepromData[0xF7]
	Wire.write(0x100 - 0xAA);//0x56
	Wire.write(0x00);
	I2Canswer(Wire.endTransmission());
}

void setConfiguration()
{
	Serial.print("\nSetting configuration...");
	byte Hz_LSB;
	switch (refreshRate)
	{
	case 0:
		Hz_LSB = 0b00111111;
		break;
	case 1:
		Hz_LSB = 0b00111110;
		break;
	case 2:
		Hz_LSB = 0b00111101;
		break;
	case 4:
		Hz_LSB = 0b00111100;
		break;
	case 8:
		Hz_LSB = 0b00111011;
		break;
	case 16:
		Hz_LSB = 0b00111010;
		break;
	case 32:
		Hz_LSB = 0b00111001;
		break;
	default:
		Hz_LSB = 0b00111110;
	}
	byte defaultConfig_H = 0b00000100; //0x04;//0b00000100;
	Wire.beginTransmission(0x60);
	Wire.write(0x03);
	Wire.write((byte)Hz_LSB - 0x55);
	Wire.write(Hz_LSB);
	Wire.write(defaultConfig_H - 0x55);//0x14-0x55
	Wire.write(defaultConfig_H);
	I2Canswer(Wire.endTransmission());

	//Read the resolution from the config register
	resolution = (readConfig() & 0x30) >> 4;
	Serial.print("Resolution: "); Serial.println(resolution); delay(100);
}


/**************************************************************
*Absolute ambient temperature data of the device can be read *
*by using the following function.                            *
**************************************************************/
void readPTAT()
{
	int i = 0;
	byte ptatLow = 0, ptatHigh = 0;
	Serial.print("Reading PTAT...");
	Wire.beginTransmission(0x60);//Sensor Address
	Wire.write(0x02);//CMD_Sensor_Read
	Wire.write(0x40);//PTAT Adress
	Wire.write(0x00);// address step (0)
	Wire.write(0x01);// number of reads (1)
	I2Canswer(Wire.endTransmission(false)); // use repeated start to get answer   )
	Wire.requestFrom(0x60, 2);// technically the 1 read takes up 2 bytes
	while (Wire.available())
	{
		i++;
		ptatLow = Wire.read();
		ptatHigh = Wire.read();
		if (i > 2) Serial.print("Error reading PTAT");
	}
	I2Canswer(Wire.endTransmission()); // use repeated start to get answer   )
	ptat = ((uint16_t)(ptatHigh << 8) | ptatLow); // rearrange int to account for endian difference (TODO: check)
}

/*****************************************************
*Compensation pixel  data of the device can be read *
*by using the following function.                   *
*****************************************************/
void readCPIX()
{
	int i = 0;
	byte cpixLow = 0, cpixHigh = 0;
	Serial.print("Reading CPIX...");
	Wire.beginTransmission(0x60);//Sensor Address
	Wire.write(0x02);//CMD_Sensor_Read
	Wire.write(0x41);//PTAT Adress
	Wire.write(0x00);// address step (0)
	Wire.write(0x01);// number of reads (1)
	I2Canswer(Wire.endTransmission(false)); // use repeated start to get answer   )
	Wire.requestFrom(0x60, 2);// technically the 1 read takes up 2 bytes
	while (Wire.available())
	{
		i++;
		cpixLow = Wire.read();
		cpixHigh = Wire.read();
		if (i > 2) Serial.print("Error reading CPIX");
	}
	I2Canswer(Wire.endTransmission()); // use repeated start to get answer   )
	cpix = ((int16_t)(cpixHigh << 8) | cpixLow);
	if (cpix >= 32768)		cpix -= 65536;
}

/*****************************************************
*IR  data of the device that it is read by lines    *
*by using the following function.                   *
*****************************************************/
void readIR()
{
	int i = 0;
	Serial.print("Reading IRsensor:");
	// Due to wire library buffer limitations, we can only read up to 32 bytes at a time
	// Thus, the request has been split into multiple different requests to get the full 128 values
	// Each pixel value takes up two bytes (???) thus NUM_PIXELS * 2
	for (int line = 0; line < 4; line++) //4 pixels lines
	{
		Serial.print("\n["); Serial.print(line); Serial.print("] ");
		Wire.beginTransmission(0x60);//Sensor Address
		Wire.write(0x02);//CMD_Sensor_Read
		Wire.write(line);//lines:0x00,0x01,0x02,0x03
		Wire.write(0x04);//0x04
		Wire.write(0x10);//0x10
		Wire.endTransmission(false);
		Wire.requestFrom(0x60, 32);//16 pixels per column x 2 bytes per pixel
		for (int j = 0; j < 16; j++)
		{
			// We read two bytes
			byte pixelDataLow = Wire.read();
			byte pixelDataHigh = Wire.read();
			irData[i] = (int16_t)((pixelDataHigh << 8) | pixelDataLow);
			Serial.print(irData[i]); Serial.print(" ");
			i++;
		}
	}
	I2Canswer(Wire.endTransmission());
}

void I2Canswer(int opcion)
{
	switch (opcion)
	{
	case 0:
		Serial.print(" Success. ");
		break;
	case 1:
		Serial.print(" Data too long to fit in transmit buffer. ");
		break;
	case 2:
		Serial.print(" Received NACK on transmit of address. ");
		break;
	case 3:
		Serial.print(" Received NACK on transmit of data. ");
		break;
	default:
		Serial.print(" Other error. ");
	}
}

float getTemperature(int num)
{
	if ((num >= 0) && (num < 64)) return temperatures[num];
	else 		                return NULL;
}

void calculateTA(void)
{
	//Declare local variables
	float v_th = 0, k_t1 = 0, k_t2 = 0;

	//Calculate variables from EEPROM
	k_t1_scale = (int16_t)(eepromData[KT_SCALE] & 0xF0) >> 4;//KT_SCALE=0xD2[7:4]
	Serial.print("\nk_t1_scale :"); Serial.println(k_t1_scale);
	k_t2_scale = (int16_t)(eepromData[KT_SCALE] & 0x0F) + 10;//KT_SCALE=0xD2[3:0]+10
	Serial.print("k_t2_scale :"); Serial.println(k_t2_scale);

	v_th = (float)256 * eepromData[VTH_H] + eepromData[VTH_L];
	if (v_th >= 32768.0)   v_th -= 65536.0;
	v_th = v_th / pow(2, (3 - resolution));
	Serial.print("v_th :"); Serial.println(v_th);

	k_t1 = (float)256 * eepromData[KT1_H] + eepromData[KT1_L];
	if (k_t1 >= 32768.0)   k_t1 -= 65536.0;
	k_t1 /= (pow(2, k_t1_scale) * pow(2, (3 - resolution)));
	Serial.print("k_t1 :"); Serial.println(k_t1);

	k_t2 = (float)256 * eepromData[KT2_H] + eepromData[KT2_L];
	//Serial.print("k_t2_A :");Serial.println(k_t2);
	if (k_t2 >= 32768.0)   k_t2 -= 65536.0;
	k_t2 /= (pow(2, k_t2_scale) * pow(2, (3 - resolution)));//0.000768
	Tambient = ((-k_t1 + sqrt(sq(k_t1) - (4 * k_t2 * (v_th - (float)ptat)))) / (2 * k_t2)) + 25.0;
}

void calculateTO()
{
	//Declare local variables
	float a_ij[64], b_ij[64], alpha_ij[64];
	float emissivity, tgc, alpha_cp, a_cp, b_cp;
	int16_t a_common, a_i_scale, b_i_scale;
	//Calculate variables from EEPROM
	emissivity = (256 * eepromData[CAL_EMIS_H] + eepromData[CAL_EMIS_L]) / 32768.0;
	a_common = (int16_t)256 * eepromData[CAL_ACOMMON_H] + eepromData[CAL_ACOMMON_L];
	if (a_common >= 32768)		a_common -= 65536;
	alpha_cp = (256 * eepromData[CAL_alphaCP_H] + eepromData[CAL_alphaCP_L]) / (pow(2, CAL_A0_SCALE) * pow(2, (3 - resolution)));
	a_i_scale = (int16_t)(eepromData[CAL_AI_SCALE] & 0xF0) >> 4;
	b_i_scale = (int16_t)eepromData[CAL_BI_SCALE] & 0x0F;
	a_cp = (float)256 * eepromData[CAL_ACP_H] + eepromData[CAL_ACP_L];
	if (a_cp >= 32768.0)		a_cp -= 65536.0;
	a_cp /= pow(2, (3 - resolution));
	b_cp = (float)eepromData[CAL_BCP];
	if (b_cp > 127.0)  		b_cp -= 256.0;
	b_cp /= (pow(2, b_i_scale) * pow(2, (3 - resolution)));
	tgc = (float)eepromData[CAL_TGC];
	if (tgc > 127.0)  tgc -= 256.0;
	tgc /= 32.0;
	float v_cp_off_comp = (float)cpix - (a_cp + b_cp * (Tambient - 25.0));
	float v_ir_off_comp, v_ir_tgc_comp, v_ir_norm, v_ir_comp;
	for (int i = 0; i < 64; i++)
	{
		a_ij[i] = ((float)a_common + eepromData[i] * pow(2, a_i_scale)) / pow(2, (3 - resolution));
		b_ij[i] = eepromData[0x40 + i];
		if (b_ij[i] > 127)  			b_ij[i] -= 256;
		b_ij[i] /= (pow(2, b_i_scale) * pow(2, (3 - resolution)));
		v_ir_off_comp = irData[i] - (a_ij[i] + b_ij[i] * (Tambient - 25.0));
		v_ir_tgc_comp = v_ir_off_comp - tgc * v_cp_off_comp;
		alpha_ij[i] = ((256 * eepromData[CAL_A0_H] + eepromData[CAL_A0_L]) / pow(2, eepromData[CAL_A0_SCALE]));
		alpha_ij[i] += (eepromData[0x80 + i] / pow(2, eepromData[CAL_DELTA_A_SCALE]));
		alpha_ij[i] /= pow(2, 3 - resolution);
		v_ir_norm = v_ir_tgc_comp / (alpha_ij[i] - tgc * alpha_cp);
		v_ir_comp = v_ir_norm / emissivity;
		temperatures[i] = sqrt(sqrt((v_ir_comp + pow((Tambient + 273.15), 4)))) - 273.15;
	}
}

//Poll the MLX90621 for its current status
//Returns true if the POR/Brown out bit is set
boolean checkConfig() //Every 16 readings check that the POR flag is not set
{
	bool check = !((readConfig() & 0x0400) >> 10);
	return check;
}

/****************************************************
*Configuration Registers of the device can be read *
*by using the following function.                  *
****************************************************/
int16_t readConfig()
{
	int i = 0;
	byte configLow = 0, configHigh = 0;
	Serial.print("\nReading Configuration...");
	Wire.beginTransmission(0x60);
	Wire.write(0x02);
	Wire.write(0x92);
	Wire.write(0x00);
	Wire.write(0x01);
	I2Canswer(Wire.endTransmission(false)); // use repeated start to get answer   )
	Wire.requestFrom(0x60, 2);
	while (Wire.available())
	{
		i++;
		byte configLow = Wire.read();
		byte configHigh = Wire.read();
		if (i > 2) Serial.print("Error reading Config");
	}
	I2Canswer(Wire.endTransmission()); // use repeated start to get answer   )
	configuration = ((int16_t)(configHigh << 8) | configLow);
	Serial.print("Configuration:"); Serial.print(configHigh); Serial.println(configLow);
	return configuration;
}




