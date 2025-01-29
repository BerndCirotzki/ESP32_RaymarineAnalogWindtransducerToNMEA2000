/**********************************************************************
 Raymarinewind.cpp by Bernd Cirotzki

 **********************************************************************/
#include "Raymarinewind.h"
#include "BluetoothStream.h"

extern BluetoothStream *pBlueTooth;

const double radToDeg = 180.0 / M_PI;
const double degToRad = M_PI / 180.0;

void eepromWriteInt(int adr, short int wert, bool co) {
byte low, high;
  low=wert&0xFF;
  high=(wert>>8)&0xFF;
  EEPROM.write(adr, low); // dauert 3,3ms 
  EEPROM.write(adr+1, high);
  if (co)
    EEPROM.commit();
  return;
}

short int eepromReadInt(int adr) {
byte low, high;
  low=EEPROM.read(adr);
  high=EEPROM.read(adr+1);
  return low + ((high << 8)&0xFF00);
}
 
Raymarinewind::Raymarinewind(tNMEA2000 *_pNMEA2000)
{
  Serial.println("Create Raymarine Wind Object");
  SenttoBluetoothandSerial = false;
  AktivImpuls = false;
  NewAngleValues = false;
  SampleTimer = millis();
  LastReceivedImpulse = millis();
  LastWriteAngleValues = millis();
  LastSendNMEA2000 = millis();
  Windspeed = 0;
  WindAngle = -1;
  cos_value = 1500;  // the middel
  sin_value = 1500;  
  for (uint8_t Impulsplace = 0;Impulsplace < 3;Impulsplace++) {  WindImpuse[Impulsplace] = 5000; }
  Impulsplace = 0;
  pNMEA2000 = _pNMEA2000;
    if (EEPROM.read(ADR_MAX_WIND_COS) == -1) // first use of ESP
  {
      eepromWriteInt(ANGELCORRECTION, 0, false);
      eepromWriteInt(ADR_WINDSPEEDCORRECTION, 0, false);
      eepromWriteInt(ADR_N2kSENDTIME, 200, false);
      EEPROM.write(ADR_LEARN, 1);
      EEPROM.commit();
  }
  autolearn = false;
  learn = EEPROM.read(ADR_LEARN);
  if (learn)
  {
     Serial.println("Is in learn mode");
     pBlueTooth->SendString("Is in learn mode\n");
  } else {
    Serial.println("No learn mode");
    pBlueTooth->SendString("No learn mode\n");
  }
  NMEA200sendtime = eepromReadInt(ADR_N2kSENDTIME); 
  WindAngleCorrection =  eepromReadInt(ANGELCORRECTION);           // In Ardurino set up it is -1 ... that is OK !
  if((Max_Windangle_cos = eepromReadInt(ADR_MAX_WIND_COS)) < 2000) // Not Set yet
  {     
     Max_Windangle_cos = 2000; // Should grov up when finding real Values.
  }  
  if((Min_Windangle_cos = eepromReadInt(ADR_MIN_WIND_COS)) < 900)
  {    
     Min_Windangle_cos = 1500; // Should go down
  }  
  if((Max_Windangle_sin = eepromReadInt(ADR_MAX_WIND_SIN)) < 2000)
  {
     Max_Windangle_sin = 2000;
  }
  if((Min_Windangle_sin = eepromReadInt(ADR_MIN_WIND_SIN)) < 900)
  {
     Min_Windangle_sin = 1500;
  }   
  WindSpeedCorrection = eepromReadInt(ADR_WINDSPEEDCORRECTION);
  if(WindSpeedCorrection < -1000 || WindSpeedCorrection > 1000)
     WindSpeedCorrection = 0; // Default No WindSpeedcorrection. 
  Serial.println("Enter \"help\" to get commandlist");
  pBlueTooth->SendString("Enter \"help\" to get commandlist\n");
}

void Raymarinewind::Handle()
{
    if (LastReceivedImpulse + 4500 < millis() )  // 4.5 Second no Impuls
    { 
       LastReceivedImpulse = millis();
       WindImpuse[Impulsplace] = 5000;
       Impulsplace++;
       if(Impulsplace > 2) Impulsplace = 0;
    }
    if(true == GetWindAngle()) // Gültige Werte vorhanden.
         SendNMEA2000data();
}

void Raymarinewind::GetWindSpeedImpulse()
{
  if (AktivImpuls) return;
  AktivImpuls = true;
  unsigned long Impuls = millis();
	if (LastReceivedImpulse + 5000 < millis() )  // 6 Second no Impulse
  { 
	   LastReceivedImpulse = Impuls;
     WindImpuse[Impulsplace] = 5000;
     Impulsplace++;
     if(Impulsplace > 2) Impulsplace = 0;     
     AktivImpuls = false;
     return; // To long Time from Last Impulse
  }
  WindImpuse[Impulsplace] = Impuls - LastReceivedImpulse;
  LastReceivedImpulse = Impuls;
  Impulsplace++;
  if(Impulsplace > 2) Impulsplace = 0;
  AktivImpuls = false;
}

double Raymarinewind::GetWindSpeed()
{
   unsigned long Impulswidth = 0;
   double Speed;

   for (uint8_t i = 0; i < 3; i++)
   {
      Impulswidth = Impulswidth + WindImpuse[i];
   }
   Speed = (double)((60 / ((double)Impulswidth / ((1000 + WindSpeedCorrection) * 3) * 2))) / 59; // RPM  ... (/59 in m/s Correction (compare Messurements with ST60+ Wind)) 3 = 3 Impulses.
   if(Speed <= 0.1 || Impulswidth == 15000)
      Speed = 0;
   if (Speed > 40) Speed = 0; // this could be the switch on moment
   if (Speed < 0.2)
      Windspeed = Speed * 0.8 + Windspeed * 0.2; // Go more faster to Zero
   else if (Speed < 0.5)
      Windspeed = Speed * 0.6 + Windspeed * 0.4; // Go faster to Zero
   else   
      Windspeed = Speed * 0.25 + Windspeed * 0.75;
   return Windspeed;
}

bool Raymarinewind::GetWindAngle()
{  
   if (SampleTimer + ANGLESAMPLERATE > millis())      // Only every 10ms
       return false;
   SampleTimer = millis();  
   double cos_temp = (double) analogRead(COS_PIN);
   double sin_temp = (double) analogRead(SIN_PIN); 
      
   if(cos_temp < 800 || sin_temp < 800 || cos_temp > 3000 || sin_temp > 3000 || isnan(cos_temp) || isnan(sin_temp))
      return false;  // No Windsensor mounted  

   cos_value = 0.03 * cos_temp + 0.97 * cos_value;
   sin_value = 0.03 * sin_temp + 0.97 * sin_value;   
   if (learn)
   {    
     if(cos_value > Max_Windangle_cos && cos_value < 4000)
     {
       Max_Windangle_cos = (short int) cos_value;
       eepromWriteInt(ADR_MAX_WIND_COS, Max_Windangle_cos, false);
       LastWriteAngleValues = millis();
       NewAngleValues = true;
     }
     if(cos_value < Min_Windangle_cos && cos_value > 900)
     {       
       Min_Windangle_cos = (short int) cos_value;
       eepromWriteInt(ADR_MIN_WIND_COS, Min_Windangle_cos, false);
       LastWriteAngleValues = millis();
       NewAngleValues = true;
     } 
     if(sin_value > Max_Windangle_sin && sin_value < 4000)
     {
       Max_Windangle_sin = (short int) sin_value;
       eepromWriteInt(ADR_MAX_WIND_SIN, Max_Windangle_sin, false);
       LastWriteAngleValues = millis();
       NewAngleValues = true;
     }
     if(sin_value < Min_Windangle_sin && sin_value > 900)
     {
       Min_Windangle_sin = (short int) sin_value;
       eepromWriteInt(ADR_MIN_WIND_SIN, Min_Windangle_sin, false);
       LastWriteAngleValues = millis();
       NewAngleValues = true;
    }  
    if (NewAngleValues == true && (LastWriteAngleValues + 60000) < millis()) // Only write the Max an min Values every 60 Seconds and when no changes since 60 seconds
    {
       NewAngleValues = false;
       Serial.println("Write new EEPROM windangle values");
       if (pBlueTooth)
           pBlueTooth->SendString("Write new EEPROM windangle values\n");
       EEPROM.commit();
       if (autolearn)
       {
          autolearn = false;
          learn = false;
          Serial.println("learn new Value was out of range");
          if (pBlueTooth)
             pBlueTooth->SendString("learn new Value was out of range\n");
       }
    }
    if (!NewAngleValues)
        LastWriteAngleValues = millis();
   }
   // ACOS : (Messwert - MinWert) / (Maxwert - MinWert)
   double Value = (2 * ((double)(cos_value - Min_Windangle_cos) / (double)(Max_Windangle_cos - Min_Windangle_cos)) -1);
   if (Value > 1)
   {     
     Value = 1;
     if (!learn)
     {
       learn = true;
       autolearn = true;
     }
   }
   if (Value < -1)
   {     
     Value = -1;
     if (!learn)
     {
       learn = true;
       autolearn = true;
     }
   }
   double CosWinkel = acos(Value) * radToDeg;
   Value = (2 * ((double)(sin_value - Min_Windangle_sin) / (double)(Max_Windangle_sin - Min_Windangle_sin)) -1);
   if (Value > 1)
   {
     Value = 1;
     if (!learn)
     {
       learn = true;
       autolearn = true;
     }
   }
   if (Value < -1)
   {     
     Value = -1;
     if (!learn)
     {
       learn = true;
       autolearn = true;
     }
   }
   double SinWinkel = asin(Value) * radToDeg;   
   // VB = 4 + 2 * sin(Winkel) ... Port or Starboard
   // VG = 4 + 2 * cos(Winkel) ... Downwind or Upwind
   if(SinWinkel >= 0)  // Wind is from Starboard
   {      
      if(CosWinkel >= 90) // Down Area to Starboard
          SinWinkel = 180 - SinWinkel;  // dieses erzeugt einen Sprung !!-((
   }
   else // Wind from Port
   {
      if(CosWinkel > 90) // Down Area to Port)
          SinWinkel = 180 - SinWinkel;  
      else                // Up Area to Port
          SinWinkel = 360 + SinWinkel;
      CosWinkel = 360 - CosWinkel;
   }
   if ( (CosWinkel > 67 && CosWinkel < 112) || (CosWinkel > 248 && CosWinkel < 292)) // using Cos is better
      WindAngle = CosWinkel;
   else if ( (SinWinkel > 338 || SinWinkel < 22) || (SinWinkel > 158 && SinWinkel < 202)) // using Sin is better
      WindAngle = SinWinkel; 
   else // Take the middel   
      WindAngle = (SinWinkel + CosWinkel) / 2; 
   WindAngle += WindAngleCorrection;
   while(WindAngle >= 360) WindAngle -= 360;
   while(WindAngle < 0) WindAngle += 360;
   return true;
}

void Raymarinewind::SendNMEA2000data()
{
   if ((LastSendNMEA2000 + NMEA200sendtime) > millis())
       return;  // Do not send too often
   LastSendNMEA2000 = millis();
   if (pNMEA2000!=0)
   {
      tN2kMsg N2kMsg;
      double WS = GetWindSpeed();
      // PNG 130306
      SID++;
      SID++; if (SID > 250) SID = 1;
      SetN2kWindSpeed(N2kMsg, SID, WS, WindAngle * degToRad, N2kWind_Apparent);
      pNMEA2000->SendMsg(N2kMsg);
      if(SenttoBluetoothandSerial)
      {
           char dummy[6];
           Serial.print("Windspeed : ");
           Serial.print(WS);
           Serial.print("m/s Windangle : ");
           Serial.print(WindAngle);
           Serial.println("deg");
           sprintf(dummy,"%3.1f", WS);
           pBlueTooth->SendString("Windspeed : ");
           pBlueTooth->SendString(dummy);
           sprintf(dummy,"%4.1f", WindAngle);
           pBlueTooth->SendString("m/s  Windangle : ");
           pBlueTooth->SendString(dummy);
           pBlueTooth->SendString("deg\n");
      }
   }   
}
