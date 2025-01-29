// Raymarine Windsensor Signals to NMEA 2000
// Bernd Cirotzki 01/2025
//
// Features :
// - Config over bluetooth or Serial
// - sending N2k Wind data (PGN 130306) converted from analog Raymarine Wind transducer
// - Best is to make a reset after first use.
// - could be mounted parallel to ST60 Instrument.

#include "BluetoothStream.h"
#include "Raymarinewind.h"

#define N2K_SOURCE 54  // NMEA2000 Sensor Adress
#define ESP32_CAN_TX_PIN GPIO_NUM_32  // Set CAN TX port TX to TX !!
#define ESP32_CAN_RX_PIN GPIO_NUM_34  // Set CAN RX port RX to RX !!

//8 seconds WDT127237
#define WDT_TIMEOUT 8

#include <Arduino.h>
#include <esp_task_wdt.h>
#include <Time.h>
#include <N2kMsg.h>
#include <NMEA2000.h>
#include <N2kMessages.h>

#include <NMEA2000_CAN.h>  // This will automatically choose right CAN library and create suitable NMEA2000 object

Raymarinewind *pWind;
BluetoothStream *pBlueTooth;

void IRAM_ATTR ReceiveWindSpeedImpulse()
{
  pWind->GetWindSpeedImpulse();
}

// Set the information for other bus devices, which messages we support
const unsigned long TransmitMessages[] PROGMEM = {130306L, // Wind
                                                  0
                                                  };


/* setup function */
void setup(void)
{
  Serial.begin(115200);
  Serial.println("Raymarine Wind Transducer V1.0");  
  NMEA2000.SetProductInformation("Raymarine Wind Transducer", // Manufacturer's Model serial code
                                 001, // Manufacturer's product code
                                 "Raymarine Wind Transducer",  // Manufacturer's Model ID
                                 "1.0",  // Manufacturer's Software version code
                                 "E22078" // Manufacturer's Model version
                                 );
  // Det device information
  NMEA2000.SetDeviceInformation(1, // Unique number. Use e.g. Serial number.
                                130, // Atmospheric. See codes on http://www.nmea.org/Assets/20120726%20nmea%202000%20class%20%26%20function%20codes%20v%202.00.pdf
                                85, // External Environment. See codes on http://www.nmea.org/Assets/20120726%20nmea%202000%20class%20%26%20function%20codes%20v%202.00.pdf
                                1851 // Raymarine .. Just choosen free from code list on http://www.nmea.org/Assets/20121020%20nmea%202000%20registration%20list.pdf                               
                               );

  NMEA2000.SetN2kCANMsgBufSize(20);
  NMEA2000.SetN2kCANReceiveFrameBufSize(250);
  NMEA2000.SetN2kCANSendFrameBufSize(250);
  NMEA2000.SetForwardSystemMessages(false);
  NMEA2000.SetMode(tNMEA2000::N2km_NodeOnly,N2K_SOURCE);
  NMEA2000.EnableForward(false);
  NMEA2000.Open();
  NMEA2000.SendProductInformation();
  // Wind Speed Impulse
  if (!EEPROM.begin(EEPROM_SIZE))
  {
      Serial.println("EEPROM start failed");      
  }
  pWind = new Raymarinewind(&NMEA2000);
  pinMode(IMPULSPIN, INPUT_PULLUP);
  attachInterrupt(IMPULSPIN, ReceiveWindSpeedImpulse, RISING);
  pBlueTooth = new BluetoothStream();
  esp_task_wdt_init(WDT_TIMEOUT, false); //enable panic so ESP32 restarts  ... muss false Sein, damit wirklicher Neustart... kein Panic. auf treue zum debuggen.
  esp_task_wdt_add(NULL); //add current thread to WDT watch  ... ESP.restart()  
}

void loop(void)
{  
  CheckConfig();
  NMEA2000.ParseMessages();
  pWind->Handle();
  // Watchdog reset !
  esp_task_wdt_reset();
}

void CheckConfig()
{
  if (Serial.available() || pBlueTooth->available())
  {
      String Val;
      char dummy[6];  
  
      if (Serial.available())
      {
        Val = Serial.readString();
        pBlueTooth->ConfigBluetooth(Val);
      }
      else
      {
        pBlueTooth->ParseMessages(Val);        
      }
      if(Val.endsWith("\n"))
            Val.replace("\n","");
      if(Val.endsWith("\r"))
            Val.replace("\r","");
      pBlueTooth->SendString("\n");
      if(Val == String("help") )
      {        
         Serial.println("help      :  show this help side");
         Serial.println("shconf    :  show the config-values");
         Serial.println("reset     :  Set all Values to default. New windangle learn must be done");
         Serial.println("restart   :  Restart the Windtransducer");
         Serial.println("shvalue   :  show the Windtransducer Values");
         Serial.println("setac [v] :  Set the Windangle correction Value in deg. -50 to +50 deg. def is 0");
         Serial.println("setsp [v] :  Set the Windspeed correction. Value from -1000 to +1000 . default is 0 (normaly it must not be changed)");
         Serial.println("setn2k [v]:  Set the sending time of N2k message in ms (default is 200 ms)");
         Serial.println("stoplearn :  Do not learn the Windangle Min and Max Values any more. This must not be done, but could be better.");
         pBlueTooth->SendString("help      :  show this help side\n");
         pBlueTooth->SendString("shconf    :  show the config-values\n");
         pBlueTooth->SendString("reset     :  Set all Values to default. New windangle learn must be done\n");
         pBlueTooth->SendString("shvalue   :  show the Windtransducer Values\n");
         pBlueTooth->SendString("restart   :  Restart the Windtransducer\n");
         pBlueTooth->SendString("setac [v] :  Set the Windangle correction Value in deg. -50 to +50 deg. def is 0\n");
         pBlueTooth->SendString("setsp [v] :  Set the windspeed correction. Value from -1000 to +1000. default is 0 (normaly it must not be changed)\n");
         pBlueTooth->SendString("setn2k [v]:  Set the sending time of N2k message in ms (default is 200 ms)\n");
         pBlueTooth->SendString("stoplearn :  Do not learn the Windangle Min and Max Values any more. This must not be done, but could be better.\n");
      }
      if(Val == String("stoplearn"))      
      {
          pWind->learn = 0;
          EEPROM.write(ADR_LEARN, 0);
          EEPROM.commit();
          Serial.println("Learnmod is stopped.");
          pBlueTooth->SendString("Learnmod is stopped.\n");
      }
      if(Val == String("shconf"))
      {
          Serial.print("Windangle correction : ");  Serial.print(pWind->WindAngleCorrection);
          Serial.print(" Windspeed correction : ");  Serial.println(pWind->WindSpeedCorrection);
          Serial.print("Angle Cos-Max : ");  Serial.print(pWind->Max_Windangle_cos);
          Serial.print(" Angle Cos-Min : ");  Serial.println(pWind->Min_Windangle_cos);
          Serial.print("Angle Sin-Max : ");  Serial.print(pWind->Max_Windangle_sin);
          Serial.print(" Angle Sin-Min : ");  Serial.println(pWind->Min_Windangle_sin);
          Serial.print("NMEA2000 Sendtime : ");  Serial.println(pWind->NMEA200sendtime);
          sprintf(dummy,"%i", pWind->WindAngleCorrection);
          pBlueTooth->SendString("Windangle correction : ");
          pBlueTooth->SendString(dummy);
          sprintf(dummy,"%i", pWind->WindSpeedCorrection);
          pBlueTooth->SendString(" Windspeed correction : ");
          pBlueTooth->SendString(dummy);
          pBlueTooth->SendString("\n");          
          sprintf(dummy,"%i", pWind->Max_Windangle_cos);
          pBlueTooth->SendString("Max_Windangle_cos : ");
          pBlueTooth->SendString(dummy);
          sprintf(dummy,"%i", pWind->Min_Windangle_cos);
          pBlueTooth->SendString(" Min_Windangle_cos : ");
          pBlueTooth->SendString(dummy);
          pBlueTooth->SendString("\n");
          sprintf(dummy,"%i", pWind->Max_Windangle_sin);
          pBlueTooth->SendString("Max_Windangle_sin : ");
          pBlueTooth->SendString(dummy);
          sprintf(dummy,"%i", pWind->Min_Windangle_sin);
          pBlueTooth->SendString(" Min_Windangle_sin : ");
          pBlueTooth->SendString(dummy);
          pBlueTooth->SendString("\n");
          printf(dummy,"%i", pWind->NMEA200sendtime);
          pBlueTooth->SendString(" NMEA2000 Sendtime : ");
          pBlueTooth->SendString(dummy);
          pBlueTooth->SendString(" m/s\n");
          if (pWind->learn)
          {
               Serial.println("Is in learn mode");
               pBlueTooth->SendString("Is in learn mode\n");
          } else {
              Serial.println("No learn mode");
              pBlueTooth->SendString("No learn mode\n");
          }           
      }
      if(Val == String("reset"))
      {
          Serial.println("Set all values to default. Learn new Windangles");
          pBlueTooth->SendString("Set all values to default. Learn new Windangles\n");
          pWind->WindAngleCorrection = 0; 
          pWind->WindSpeedCorrection = 0;
          pWind->Max_Windangle_cos = 1000;
          pWind->Min_Windangle_cos = 2000;
          pWind->Max_Windangle_sin = 1000;
          pWind->Min_Windangle_sin = 2000;
          pWind->NMEA200sendtime = 200;
          pWind->learn = 1;
          pWind->autolearn = false;
          eepromWriteInt(ADR_MAX_WIND_COS, 1000, false);
          eepromWriteInt(ADR_MIN_WIND_COS, 2000, false);
          eepromWriteInt(ADR_MAX_WIND_SIN, 1000, false);
          eepromWriteInt(ADR_MIN_WIND_SIN, 2000, false);
          eepromWriteInt(ANGELCORRECTION, 0, false);
          eepromWriteInt(ADR_WINDSPEEDCORRECTION, 0,false);
          EEPROM.write(ADR_LEARN, 1); 
          eepromWriteInt(ADR_N2kSENDTIME, 200);         
      }
      if(Val == String("restart"))
      {
        Serial.println("Windtransducer restart.");
        pBlueTooth->SendString("Windtransducer restart.\n");        
        delay(400);
        ESP.restart(); //call reset 
      }
      if(Val.substring(0, 5) == String("setsp"))      
      {
         pWind->WindSpeedCorrection = Val.substring(6, Val.length()).toInt();
         if(pWind->WindSpeedCorrection < -1000 || pWind->WindSpeedCorrection > 1000)
         {
            pWind->WindSpeedCorrection = 0; // Default No WindSpeedcorrection.
            Serial.println("Error Value out of range");
            pBlueTooth->SendString("Error Value out of range\n");
         }
         else
         {
            sprintf(dummy,"%i",pWind->WindSpeedCorrection);
            Serial.print("New WindSpeedCorrection: ");Serial.println(pWind->WindSpeedCorrection);
            pBlueTooth->SendString("New WindSpeedCorrection: ");pBlueTooth->SendString(dummy);pBlueTooth->SendString("\n");
            eepromWriteInt(ADR_WINDSPEEDCORRECTION, pWind->WindSpeedCorrection); 
         }
      }
      if(Val.substring(0, 5) == String("setac"))      
      {
         pWind->WindAngleCorrection = Val.substring(6, Val.length()).toInt();
         if(pWind->WindAngleCorrection < -50 || pWind->WindAngleCorrection > 50)
         {
            pWind->WindAngleCorrection = 0;
            Serial.println("Error Value out of range");
            pBlueTooth->SendString("Error Value out of range\n");
         }
         else
         {         
            sprintf(dummy,"%i",pWind->WindAngleCorrection);
            Serial.print("New WindAngleCorrection: ");Serial.println(pWind->WindAngleCorrection);
            pBlueTooth->SendString("New WindAngleCorrection: ");pBlueTooth->SendString(dummy);pBlueTooth->SendString("\n");
            eepromWriteInt(ANGELCORRECTION, pWind->WindAngleCorrection); 
         }
      }
      if(Val.substring(0, 6) == String("setn2k"))      
      {
         pWind->NMEA200sendtime = Val.substring(7, Val.length()).toInt();
         if(pWind->NMEA200sendtime < 50 || pWind->NMEA200sendtime > 5000)
         {
            pWind->NMEA200sendtime = 200;
            Serial.println("Error Value out of range. set to 200ms");
            pBlueTooth->SendString("Error Value out of range. set to 200ms\n");
         }
         else
         {         
            sprintf(dummy,"%i",pWind->NMEA200sendtime);
            Serial.print("New NMEA200sendtime: ");Serial.println(pWind->NMEA200sendtime);
            pBlueTooth->SendString("New NMEA200sendtime: ");pBlueTooth->SendString(dummy);pBlueTooth->SendString("\n");
            eepromWriteInt(ADR_N2kSENDTIME, pWind->NMEA200sendtime); 
         }
      }
      if(Val == String("shvalue"))
      {         
          pWind->SenttoBluetoothandSerial = true;
      }
   }
}
