## **Practica 5: Buses de comunicación I (introducción y I2c)**
## **Ejercicio Practico 1 ESCÁNER I2C:**
**Codigo main.cpp:**
```
#include <Arduino.h>
#include <Wire.h>

void setup() {
  Wire.begin(21,20);
  Serial.begin(115200);
  while (!Serial);
  Serial.println("\nI2C Scanner");
}

void loop() {
  byte error, address;
  int nDevices = 0;

  Serial.println("Scanning...");

  for(address = 1; address < 127; address++) {
    Wire.beginTransmission(address);
    error = Wire.endTransmission();

    if (error == 0) {
      Serial.print("I2C device found at address 0x");
      if (address < 16) Serial.print("0");
      Serial.println(address, HEX);
      nDevices++;
    } else if (error == 4) {
      Serial.print("Unknown error at address 0x");
      if (address < 16) Serial.print("0");
      Serial.println(address, HEX);
    }     
  }

  if (nDevices == 0) Serial.println("No I2C devices found\n");
  else Serial.println("done\n");

  delay(5000);
}


```
la salida

```
Scanning...
I2C device found at address 0x57 !
done
```

## **Ejercicio Practico 2:**
## **Ejercicio de subida de nota ( muy valorado):**

### **Parte 1:**
### **Parte 2:**
