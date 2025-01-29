# Raymarine analog Windtransducer (E22078) to NMEA2000 Output
This Code converts the analog Winddransducer Data to NMEA2000.
The Setting can be done via Bluetooth.
- calibrition
- Windangle correction
- Windspeedcorrection
- could be mounted parallel to Raymarine Instuments (p.E ST60)
- send out NMEA2000 Data
 
## used Librarys
- NMEA2000 Library https://github.com/ttlappalainen/NMEA2000
- NMEA2000_esp32  https://github.com/ttlappalainen/NMEA2000_esp32
- BluetoothSerial

## Bluetooth
- Configuration can be done via Bluetooth using free App SerialBluetooth. Enter "help" to get the commandlist

### NMEA2000 PGN sending
  130306 - Winddata
  
### Circuit diagram
    
![Raymarine Windtransducer to NMEA2000](https://github.com/user-attachments/assets/3da0d31f-4c62-4c38-938b-760570098d73)
