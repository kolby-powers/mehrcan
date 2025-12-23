# mehrcan
Arduino Märklin digital Steuerung für CAN-Bus System (Digitale Modelleisenbahn H0 / Model Railroad)

## Allgemein
Die aktuelle Version ist für einen einfachen Pendelverkehr zwischen zwei Punkten / Bahnhöfen vorgesehen. Als Erkennung werden 2 Sensoren für die Endpunkte verwendet sowie eine Zeitspanne für den schnellen Fahrabschnitt. Der Code innerhalb des "Track" Abschnitt kann an die individuellen Abläufe angepasst werden und dient lediglich als Beispiel.

# Hardware
* [Arduino NANO](https://store.arduino.cc/collections/nano-family/products/arduino-nano)
* [CAN Shield AZ-Delivery](https://www.az-delivery.de/en/products/mcp2515-can-bus-modul)
* [Märklin digital Gleisbox](https://www.maerklin.de/de/produkte/details/article/60116)

## Sensoren
Für den Pendelverkehr im Beispiel werden je ein Reed-Sensor unter dem Gleis als Endpunkt verwendet. Die Sensoren schalten dabei gegen Masse und verwenden einen internen Pullup des Arduino für die Erkennung. (Konfiguration der Pins und Pullup bei Bedarf im "Track" Abschnitt des Code)

## Pinout
Der Anschluss des CAN-Shield erfolgt ganz klassisch per SPI-Header des Arduino (6-Pin-Header). Der CS-Pin (ChipSelect) ist bei auf den **Pin 10 (D10)** des Arduino zu verbinden. Der INT (Interupt) Pin des CAN-Shield wird in der aktuellen Konfiguration nicht benötigt. Zusätzlich zu CAN-H und CAN-L sollte auch die Masse der Gleisbox mit dem Arduino verbunden werden um die Signalqualität zu verbessern. Werden keine weiteren Geräte in diesen CAN-Bus Teil eingebunden, so ist die Brücke für den Abschlusswiderstand am CAN-Shield zu schließen.

Der Anschluss der Sensoren erfolgt jeweils einmal mit Masse des Arduino, sowie gegen **Pin 2 (D2)** und **Pin 4 (D4)** für die Signalleitung. Es können unterschiedliche Sensoren verwendet werden, solange diese bei Kontakt die Masse auf den entsprechenden Pin schalten (Schließer).

**Zusammenfassung**

* SPI-Header Pins für CAN-Shield
* D10 für CS des CAN-Shield
* Zusätzliche Masse (GND) für Gleisbox
* D2 + GND für Sensor 1
* D4 + GND für Sensor 2
