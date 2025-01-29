# Raymarine analog Windtransducer (E22078) to NMEA2000 Output
This Code converts the analog Winddransducer Data to NMEA2000.
The Setting can be done via Bluetooth.
- calibrition
- Windangle correction
- Windspeedcorrectin
 
## used Librarys
- NMEA2000 Library https://github.com/ttlappalainen/NMEA2000
- NMEA2000_esp32  https://github.com/ttlappalainen/NMEA2000_esp32
- BluetoothSerial

## Bluetooth
- Configuration can be done via Bluetooth using free App SerialBluetooth. Enter "help" to get the commandlist

### NMEA2000 PGN sending
  130306 - Winddata
  
### Circuit diagram
    
