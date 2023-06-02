# small battery powered LoRaWAN temperature sensor. 
This small LoRaWAN temperature sensor measures the temperature with a Dalla DS18B20 temperature sensor. In freely selectable transmission intervals the temperature and also the voltage data of the CR2032 are sent to the TTS / TTN network. The frequency of 868 MHz, which is common in Europe, is used as the transmission frequency. The software uses the LoRaWAN protocol version 1.0.3 with a secure activation via OTAA.
The transmission interval can be preset between 1 and 1440 minutes directly before compiling the C software. But much more comfortable is the function that you can set the send interval remotely via a downlink. The send interval in minutes is set either as 1 byte value (0x01 - 0xFF) corresponding to 1 to 255 minutes, or as 2 byte values (e.g. 0x02 , 0x58) corresponding to 600 minutes. The new transmission interval is taken over from the next transmission process.
The very compact SMD sensor is populated on both sides, is smaller than a matchbox and weighs only 11g. The quiescent current requirement of less than 6 µA allows the sensor to operate for a very long time with the used CR20232 button cell with 230 mAh. How many transmissions are possible with one coin cell, I will report at a later time.
The small PCB has beside the usual serial port for programming via Arduino DIE also a still unused I2C port. Here further sensors could be connected if necessary. There is no direct ISP port for programming the ATMEGA328P due to space limitations. Because the ISP port has to be used only once after the setup (Arduino hex firmware) I marked the 6 connection points on the layout. A simple 2x3 pin list with 6 short cables serves this purpose once.

# kleiner batteriebetriebener LoRaWAN Temperatursensor 
Dieser kleine LoRaWAN Temperatursensor misst die Temperatur mit einem Dalla DS18B20 Temperaturfühler. In frei wählbaren Sendeintervallen werden die Temperatur- und auch die Spannungsdaten der CR2032 an das TTS / TTN Netzwerk gesendet. Als Sendefrequenz wird die in Europa übliche Frequenz von 868 MHz genutzt. Die Software verwendet das LoRaWAN Protokoll Version 1.0.3 mit einer sicheren Aktivierung über OTAA.
Der Sendeintervall kann direkt vor dem Kompilieren der C Software zwischen 1 und 1440 Minuten voreingestellt werden. Viel komfortabler ist aber die Funktion, dass man über einen Downlink den Sendeintervall auch aus der Ferne einstellen kann. Der Sendeintervall in Minuten wird dafür entweder als 1 Byte Wert (0x01 – 0xFF) entspricht 1 bis 255 Minuten, oder als 2 Byte Werte (z.B. 0x02 , 0x58) entspricht 600 Minuten. Der neue Sendeintervall wird ab dem nächsten Sendevorgang übernommen.
Der sehr kompakt aufgebaute SMD Sensor ist auf beiden Seiten bestückt, ist kleiner als eine Streichholzschachtel und wiegt nur 11 g. Der Ruhestrombedarf von weniger als 6 µA lässt den Sensor mit der verwendeten CR20232 Knopfzelle mit 230 mAh sehr lange Zeit betreiben. Wie viele Sendevorgänge mit einer Knopfzelle möglich sind, werde ich zu einer späteren Zeit berichten.
Die kleine Leiterplatte hat neben dem üblichen seriellen Anschluss zur Programmierung per Arduino DIE auch einen noch ungenutzten I2C Port. Hier könnten bei Bedarf weitere Sensoren angeschlossen werden. Einen direkten ISP Anschluss zur Programmierung des ATMEGA328P gibt es aus Platzgründen nicht. Da der ISP Anschluss nur einmal nach dem Aufbau verwendet werden muss (Arduino Hex-Firmware) habeich die 6 Anschlusspunkte auf dem Layout markiert. Eine einfache 2x3 Steckliste mit 6 kurzen Kabel erfüllt einmalig diesen Zweck. 
