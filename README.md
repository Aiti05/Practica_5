## **Practica 5: Buses de comunicación I (introducción y I2c)**
El objetivo de esta práctica es comprender el funcionamiento de los buses de comunicación entre periféricos, en particular el protocolo I2C (Inter-Integrated Circuit). Este protocolo serie permite la conexión de múltiples dispositivos utilizando solo dos líneas: SDA (Serial Data Line), que transporta los datos, y SCL (Serial Clock Line), que proporciona la señal de sincronización. Su arquitectura maestro-esclavo permite que un dispositivo principal controle varios periféricos mediante direcciones únicas, facilitando la comunicación con sensores, pantallas y otros módulos electrónicos.

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

![WhatsApp Image 2025-03-22 at 14 20 37 (2)](https://github.com/user-attachments/assets/e191d201-d824-4221-b746-d856c0fc8b69)
Para iniciar, en el primer ejercicio práctico se implementa un escáner I2C que permite detectar qué dispositivos están conectados al bus y muestra sus direcciones en el monitor serie. El código en C++ utiliza la librería Wire.h para inicializar el bus I2C en el ESP32 con los pines GPIO 21 (SDA) y GPIO 20 (SCL). Se realiza un bucle donde se recorren todas las direcciones posibles (del 1 al 126) e intenta comunicarse con cada una. Si la transmisión se completa sin errores, significa que un dispositivo está presente en esa dirección, por lo que se imprime su dirección en formato hexadecimal. Si no hay respuesta, se informa que no se encontró ningún dispositivo. 

La salida esperada en el monitor serie es una lista de dispositivos detectados, mostrando su dirección en formato hexadecimal.
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
En el segundo ejercicio práctico se modifica el código del escáner I2C para que, en lugar de imprimir las direcciones en el monitor serie, estas se muestren en una pantalla OLED SSD1306 utilizando la librería Adafruit_SSD1306.h. Se inicializa la pantalla OLED en la dirección I2C 0x3C y se establece un texto de bienvenida indicando que se está realizando un escaneo. Luego, en el bucle principal, se recorren las direcciones I2C como en el primer ejercicio, pero en lugar de imprimir los resultados en el monitor serie, estos se dibujan en la pantalla OLED. Si no se encuentran dispositivos, se muestra el mensaje “No hay dispositivos I2C”. Este código permite visualizar directamente en la pantalla OLED la lista de dispositivos detectados.

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
Además, se incluye un tercer ejercicio donde se utiliza la pantalla OLED para mostrar un mensaje personalizado. Este código simplemente configura la pantalla OLED y muestra un texto fijo de bienvenida con un tamaño de fuente más grande. Este ejemplo demuestra cómo se puede utilizar la pantalla OLED para mostrar información sin necesidad de escanear dispositivos I2C.

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
En este ejercicio de subida de nota se integra un sensor MAX30102 para medir la frecuencia cardíaca y la saturación de oxígeno en sangre, mostrando los resultados en la pantalla OLED. Se usa la librería MAX30105.h para inicializar el sensor y obtener los valores de infrarrojo (IR), que permiten calcular la frecuencia cardíaca. El código muestra estos valores en la pantalla OLED, actualizándolos cada 500 ms.


### **Parte 2:**
```
#include <Wire.h>
#include <Adafruit_GFX.h>
#include <Adafruit_SSD1306.h>
#include "MAX30105.h"
#include "spo2_algorithm.h"

#define SCREEN_WIDTH 128
#define SCREEN_HEIGHT 32
#define SCREEN_ADDRESS 0x3C

Adafruit_SSD1306 display(SCREEN_WIDTH, SCREEN_HEIGHT, &Wire, -1);
MAX30105 particleSensor;

uint32_t irBuffer[100], redBuffer[100];
int32_t spo2, heartRate;
int8_t validSPO2, validHeartRate;

void setup() {
  Serial.begin(115200);
  display.begin(SSD1306_SWITCHCAPVCC, SCREEN_ADDRESS);
  display.clearDisplay();
  display.setTextSize(1);
  display.setTextColor(SSD1306_WHITE);
  display.setCursor(0, 0);
  display.println("Iniciando...");
  display.display();
  
  if (!particleSensor.begin(Wire, I2C_SPEED_STANDARD)) {
    Serial.println("Sensor MAX30102 no encontrado");
    for (;;);
  }

  particleSensor.setup(60, 4, 2, 100, 411, 4096);
}

void loop() {
  for (byte i = 0; i < 100; i++) {
    while (!particleSensor.available()) particleSensor.check();
    redBuffer[i] = particleSensor.getRed();
    irBuffer[i] = particleSensor.getIR();
    particleSensor.nextSample();
  }

  maxim_heart_rate_and_oxygen_saturation(irBuffer, 100, redBuffer, &spo2, &validSPO2, &heartRate, &validHeartRate);

  display.clearDisplay();
  display.setCursor(0, 0);
  display.print("HR: ");
  display.println(validHeartRate ? String(heartRate) + " BPM" : "N/A");
  display.print("SpO2: ");
  display.println(validSPO2 ? String(spo2) + " %" : "N/A");
  display.display();
  
  delay(1000);
}

```
Este codigo permite medir la frecuencia cardíaca y la saturación de oxígeno en sangre con el sensor MAX30102 y mostrar los valores en una pantalla OLED SSD1306. Utiliza comunicación I2C para conectar el ESP32 con ambos dispositivos y presenta los resultados en tiempo real. Primero, se inicializa la pantalla OLED y se muestra un mensaje de inicio. Luego, el sensor MAX30102 es configurado para capturar datos de luz roja e infrarroja, necesarios para calcular la frecuencia cardíaca y el SpO2. En el bucle principal, se leen 100 muestras del sensor, que luego se procesan con la función de cálculo para determinar los valores de HR (Heart Rate) y SpO2. Finalmente, los resultados se muestran en la pantalla OLED con un formato claro y actualizado cada segundo.
