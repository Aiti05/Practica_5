## **Practica 5: Buses de comunicación I (introducción y I2c)**
## **Ejercicio Practico 1 ESCÁNER I2C:**
**Codigo main.cpp:**
```
#include <Arduino.h>
#include <Wire.h>
void setup()
{
  Wire.begin(21,20);
  Serial.begin(115200);
  while (!Serial);             // Leonardo: wait for serial monitor
  Serial.println("\nI2C Scanner");
}
void loop()
{
  byte error, address;
  int nDevices;
  Serial.println("Scanning...");
  nDevices = 0;
  for(address = 1; address < 127; address++ )
  {
    // The i2c_scanner uses the return value of
    // the Write.endTransmisstion to see if
    // a device did acknowledge to the address.
    Wire.beginTransmission(address);
    error = Wire.endTransmission();
    if (error == 0)
    {
      Serial.print("I2C device found at address 0x");
      if (address<16)
        Serial.print("0");
      Serial.print(address,HEX);
      Serial.println("  !");
      nDevices++;
    }
    else if (error==4)
    {
      Serial.print("Unknown error at address 0x");
      if (address<16)
        Serial.print("0");
      Serial.println(address,HEX);
    }
  }
  if (nDevices == 0)
    Serial.println("No I2C devices found\n");
  else
    Serial.println("done\n");
  delay(5000);           // wait 5 seconds for next scan
}


```
la salida

```
Scanning...
I2C device found at address 0x57 !
done
```

## **Ejercicio Practico 2:**
**Codigo main.cpp para que por la pantalla salga el mensaje del ejercicio practico 1:**

```
#include <Wire.h>
#include <Adafruit_GFX.h>
#include <Adafruit_SSD1306.h>

// Definir las dimensiones del OLED y la dirección I2C
#define SCREEN_WIDTH 128
#define SCREEN_HEIGHT 64
#define OLED_RESET -1 // No utilizamos un pin de reset
Adafruit_SSD1306 display(SCREEN_WIDTH, SCREEN_HEIGHT, &Wire, OLED_RESET);

void setup() {
  Serial.begin(115200); // Inicializar comunicación serie

  // Inicializar la comunicación I2C en los pines SDA (21) y SCL (20) para ESP32
  Wire.begin(21, 20); // Usa los pines 21 (SDA) y 20 (SCL)

  // Inicializar el display OLED con la dirección I2C 0x3C
  if (!display.begin(SSD1306_SWITCHCAPVCC, 0x3C)) {
    Serial.println(F("No se pudo encontrar un display OLED"));
    for (;;); // Detener el programa si no se encuentra el display
  }

  // Limpiar la pantalla
  display.clearDisplay();
  display.setTextSize(1);
  display.setTextColor(SSD1306_WHITE);
  display.setCursor(0, 0);

  // Mostrar mensaje inicial
  display.println(F("Escaneando dispositivos I2C..."));
  display.display();
  delay(1000);
}

void loop() {
  byte error, address;
  int nDevices = 0;

  display.clearDisplay(); // Limpiar la pantalla antes de mostrar resultados
  display.setCursor(0, 0); // Ubicar el cursor en la parte superior

  // Escanear direcciones I2C de 1 a 126
  for (address = 1; address < 127; address++) {
    Wire.beginTransmission(address);
    error = Wire.endTransmission();

    if (error == 0) {
      // Si se encuentra un dispositivo, mostrar su dirección en hexadecimal
      display.print(F("Dispositivo I2C en: 0x"));
      if (address < 16) display.print("0"); // Asegurarse de que la dirección tiene 2 dígitos
      display.println(address, HEX); // Mostrar dirección en formato hexadecimal
      nDevices++;
    } else if (error == 4) {
      // Error desconocido
      display.print(F("Error en: 0x"));
      if (address < 16) display.print("0");
      display.println(address, HEX);
    }
  }

  // Si no se encontraron dispositivos, mostrar mensaje correspondiente
  if (nDevices == 0) {
    display.println(F("No se encontraron dispositivos"));
  } else {
    display.println(F("Escaneo completado"));
  }

  display.display(); // Actualizar pantalla con los resultados
  delay(5000); // Esperar 5 segundos antes de hacer otro escaneo
}
```
![WhatsApp Image 2025-03-22 at 14 20 37](https://github.com/user-attachments/assets/5d35003f-8d13-4add-a36b-c785d50dee20)


**Codigo main.cpp para que por la pantalla salga un mensaje personalizado:**
```
#include <Wire.h>
#include <Adafruit_GFX.h>
#include <Adafruit_SSD1306.h>


#define SCREEN_WIDTH 128
#define SCREEN_HEIGHT 64
#define OLED_RESET -1 // No estamos usando un pin de reset
Adafruit_SSD1306 display(SCREEN_WIDTH, SCREEN_HEIGHT, &Wire, OLED_RESET);


void setup() {
  Serial.begin(115200);


  // Inicializar la comunicación I2C en los pines 21 (SDA) y 20 (SCL)
  Wire.begin(21, 20);
  
  // Inicializar el display con la dirección I2C 0x3C (o 0x3D dependiendo de tu pantalla)
  if (!display.begin(SSD1306_SWITCHCAPVCC, 0x3C)) {
    Serial.println(F("No se pudo encontrar un display OLED"));
    for (;;); // Detener el programa si no se encuentra el display
  }


  // Limpiar la pantalla
  display.clearDisplay();


  // Configurar el texto en la pantalla
  display.setTextSize(2);
  display.setTextColor(SSD1306_WHITE);
  display.setCursor(0, 0);
  display.print(F("Hola Palma"));
  display.display();
}


void loop() {
  // Lo que quieras actualizar en la pantalla
}
```
![WhatsApp Image 2025-03-22 at 14 20 37 (1)](https://github.com/user-attachments/assets/89b3db85-b111-4d9f-8972-c5c95b7642db)


## **Ejercicio de subida de nota ( muy valorado):**

### **Parte 1:**
```
#include <Arduino.h>
#include <Wire.h>
#include <Adafruit_GFX.h>
#include <Adafruit_SSD1306.h>
#include "MAX30105.h"
#include "spo2_algorithm.h"

#define SCREEN_WIDTH 128
#define SCREEN_HEIGHT 32
#define OLED_RESET -1
#define SCREEN_ADDRESS 0x3C

Adafruit_SSD1306 display(SCREEN_WIDTH, SCREEN_HEIGHT, &Wire, OLED_RESET);
MAX30105 particleSensor;

void setup() {
  Serial.begin(115200);
  
  if(!display.begin(SSD1306_SWITCHCAPVCC, SCREEN_ADDRESS)) {
    Serial.println(F("SSD1306 allocation failed"));
    for(;;);
  }
  
  if (!particleSensor.begin(Wire, I2C_SPEED_STANDARD)) {
    Serial.println("MAX30102 not found");
    for(;;);
  }

  particleSensor.setup(60, 4, 2, 100, 411, 4096);
  
  display.clearDisplay();
  display.setTextSize(1);
  display.setTextColor(SSD1306_WHITE);
  display.setCursor(0,0);
  display.println("Sensor MAX30102");
  display.display();
}

void loop() {
  int32_t irValue = particleSensor.getIR();

  display.clearDisplay();
  display.setCursor(0, 0);
  display.print("IR Value: ");
  display.println(irValue);
  display.display();

  delay(500);
}

```
### **Parte 2:**
