/**********************************************************************
 Raymarinewind.h by Bernd Cirotzki

 **********************************************************************/

#ifndef _RAYMARINEWIND_H_
#define _RAYMARINEWIND_H_

#include <Arduino.h>
#include <EEPROM.h>
#include <NMEA2000.h>
#include "N2kMessages.h"
#include <math.h>

#define EEPROM_SIZE 15
#define ADR_MAX_WIND_COS 0  // 2 bytes Int
#define ADR_MIN_WIND_COS 2
#define ADR_MAX_WIND_SIN 4 
#define ADR_MIN_WIND_SIN 6
#define ANGELCORRECTION  8  // Adress Winkelkorrektur Raymarine Windmesser
#define ADR_WINDSPEEDCORRECTION 10
#define ADR_N2kSENDTIME 12
#define ADR_LEARN 14

#define COS_PIN 36   // Green wire
#define SIN_PIN 39   // Blue wire
#define IMPULSPIN 23 // Windspeed Wellow wire
#define ANGLESAMPLERATE 10

void eepromWriteInt(int adr, short int wert, bool co = true);
short int eepromReadInt(int adr);

class Raymarinewind
{
	public:
	  Raymarinewind(tNMEA2000 *_pNMEA2000);
		~Raymarinewind(){};
		void GetWindSpeedImpulse();
    bool GetWindAngle();
    void Handle();  // Use this Funktion in Loop
    short int WindAngleCorrection;
    short int WindSpeedCorrection;    
    short int Max_Windangle_cos;
    short int Min_Windangle_cos;
    short int Max_Windangle_sin;
    short int Min_Windangle_sin;
    short int NMEA200sendtime;
    bool SenttoBluetoothandSerial;
    bool learn;
    bool autolearn;
	private:
    void SendNMEA2000data();
    double GetWindSpeed();   // Returns in m/s
    unsigned long SampleTimer;
	  unsigned long LastReceivedImpulse;
    unsigned long LastSendNMEA2000;
    unsigned long LastWriteAngleValues;
    bool AktivImpuls;
		double WindAngle;  // in Deg
    double Windspeed;  // in m/s
    unsigned long WindImpuse[3];
    uint8_t Impulsplace;
    tNMEA2000 *pNMEA2000;
    double cos_value;
    double sin_value;    
    short int SID;
    bool NewAngleValues;    
};

#endif // _RAYMARINEWIND_H_
