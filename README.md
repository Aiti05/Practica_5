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
#include <Wire.h>
#include <Adafruit_GFX.h>
#include <Adafruit_SSD1306.h>
#include "MAX30105.h"
#include "spo2_algorithm.h"

// Definir los pines I2C personalizados para ESP32-S3
#define SDA_PIN 21  
#define SCL_PIN 22

// Configuración OLED
#define SCREEN_WIDTH 128
#define SCREEN_HEIGHT 64
Adafruit_SSD1306 display(SCREEN_WIDTH, SCREEN_HEIGHT, &Wire, -1);

// Sensor MAX30105
MAX30105 particleSensor;
uint32_t irBuffer[100], redBuffer[100];
int32_t spo2, heartRate;
int8_t validSPO2, validHeartRate;

void setup() {
  Serial.begin(115200);
  Wire.begin(SDA_PIN, SCL_PIN);  // Configurar I2C

  // Iniciar pantalla OLED
  if (!display.begin(SSD1306_SWITCHCAPVCC, SCREEN_ADDRESS)) {
    Serial.println("Fallo en OLED SSD1306");
    while (1);
  }
  display.clearDisplay();
  display.setTextSize(1);
  display.setTextColor(SSD1306_WHITE);
  display.setCursor(0, 0);
  display.println("Iniciando...");
  display.display();

  // Iniciar sensor MAX30105
  if (!particleSensor.begin(Wire, I2C_SPEED_STANDARD)) {
    Serial.println("Sensor MAX30105 no encontrado");
    while (1);
  }

  // Configuración optimizada para MAX30105
  particleSensor.setup(69, 4, 2, 100, 411, 4096);  // Configuración optimizada para HR + SpO2
}

void loop() {
  // Leer 100 muestras del sensor
  for (byte i = 0; i < 100; i++) {
    while (!particleSensor.available()) particleSensor.check();
    redBuffer[i] = particleSensor.getRed();
    irBuffer[i] = particleSensor.getIR();
    particleSensor.nextSample();
  }

  // Calcular SpO2 y HR
  maxim_heart_rate_and_oxygen_saturation(irBuffer, 100, redBuffer, &spo2, &validSPO2, &heartRate, &validHeartRate);

  // Mostrar en pantalla OLED
  display.clearDisplay();
  display.setCursor(0, 0);
  display.print("HR: ");
  display.println(validHeartRate ? String(heartRate) + " BPM" : "N/A");
  display.print("SpO2: ");
  display.println(validSPO2 ? String(spo2) + " %" : "N/A");
  display.display();

  Serial.print("HR: ");
  Serial.print(validHeartRate ? String(heartRate) + " BPM" : "N/A");
  Serial.print(" | SpO2: ");
  Serial.println(validSPO2 ? String(spo2) + " %" : "N/A");

  delay(1000);
}


```
El código configura un ESP32-S3 para medir la frecuencia cardíaca y la saturación de oxígeno (SpO₂) utilizando un sensor MAX30105 y mostrar los valores en un display OLED SSD1306. Se inicializa la comunicación I2C con los pines SDA 21 y SCL 22, y se verifica que tanto el sensor como la pantalla funcionen correctamente. En cada iteración del loop(), se recogen 100 muestras de los valores de luz roja e infrarroja del sensor y se procesan con la función maxim_heart_rate_and_oxygen_saturation(), que calcula la frecuencia cardíaca y el SpO₂. Los datos se imprimen en el OLED y en el puerto serie, actualizándose cada segundo.


### **Parte 2:**
```
#include <WiFi.h>
#include <Wire.h>
#include <Adafruit_GFX.h>
#include <Adafruit_SSD1306.h>
#include "MAX30105.h"
#include "spo2_algorithm.h"

// Configuración WiFi
const char* ssid = "Nautilus";     
const char* password = "20000Leguas"; 

// Servidor Web
WiFiServer server(80);

// Configuración OLED
#define SCREEN_WIDTH 128
#define SCREEN_HEIGHT 64
Adafruit_SSD1306 display(SCREEN_WIDTH, SCREEN_HEIGHT, &Wire, -1);

// Sensor MAX30105
MAX30105 particleSensor;
uint32_t irBuffer[100], redBuffer[100];
int32_t spo2, heartRate;
int8_t validSPO2, validHeartRate;

// Pines I2C para ESP32-S3
#define SDA_PIN 21 
#define SCL_PIN 22 

void setup() {
  Serial.begin(115200);
  Wire.begin(SDA_PIN, SCL_PIN);

  // Conexión a WiFi
  WiFi.begin(ssid, password);
  Serial.print("Conectando a WiFi");
  while (WiFi.status() != WL_CONNECTED) {
    delay(500);
    Serial.print(".");
  }
  Serial.println("\nWiFi conectado!");
  Serial.print("IP: ");
  Serial.println(WiFi.localIP());
  server.begin();

  // Iniciar OLED
  if (!display.begin(SSD1306_SWITCHCAPVCC, SCREEN_ADDRESS)) {
    Serial.println("Fallo en OLED");
    while (1);
  }
  display.clearDisplay();
  display.setTextSize(1);
  display.setTextColor(SSD1306_WHITE);
  display.setCursor(0, 0);
  display.println("Iniciando...");
  display.display();

  // Iniciar sensor MAX30105
  if (!particleSensor.begin(Wire, I2C_SPEED_STANDARD)) {
    Serial.println("Sensor MAX30105 no encontrado");
    while (1);
  }
  particleSensor.setup(69, 4, 2, 100, 411, 4096);
}

void loop() {
  // Manejar clientes web
  WiFiClient client = server.available();
  if (client) {
    String request = client.readStringUntil('\r');
    client.flush();

    // Enviar datos al cliente
    String response = "HTTP/1.1 200 OK\r\nContent-Type: text/html\r\n\r\n";
    response += "<!DOCTYPE html><html><head><meta http-equiv='refresh' content='1'>";
    response += "<style>body{font-family:Arial;text-align:center;} .data{font-size:2em;color:blue;}</style>";
    response += "</head><body>";
    response += "<h2>Monitor de Frecuencia Cardiaca y SpO2</h2>";
    response += "<p>Frecuencia Cardiaca: <span class='data'>" + String(validHeartRate ? String(heartRate) + " BPM" : "N/A") + "</span></p>";
    response += "<p>SpO2: <span class='data'>" + String(validSPO2 ? String(spo2) + " %" : "N/A") + "</span></p>";
    response += "</body></html>";

    client.print(response);
    client.stop();
  }

  // Leer datos del sensor
  for (byte i = 0; i < 100; i++) {
    while (!particleSensor.available()) particleSensor.check();
    redBuffer[i] = particleSensor.getRed();
    irBuffer[i] = particleSensor.getIR();
    particleSensor.nextSample();
  }

  maxim_heart_rate_and_oxygen_saturation(irBuffer, 100, redBuffer, &spo2, &validSPO2, &heartRate, &validHeartRate);

  // Mostrar en OLED
  display.clearDisplay();
  display.setCursor(0, 0);
  display.print("HR: ");
  display.println(validHeartRate ? String(heartRate) + " BPM" : "N/A");
  display.print("SpO2: ");
  display.println(validSPO2 ? String(spo2) + " %" : "N/A");
  display.display();

  // Mostrar en Monitor Serie
  Serial.print("HR: ");
  Serial.print(validHeartRate ? String(heartRate) + " BPM" : "N/A");
  Serial.print(" | SpO2: ");
  Serial.println(validSPO2 ? String(spo2) + " %" : "N/A");

  delay(1000);
}


```
Se mantiene la funcionalidad del sensor y la pantalla OLED, pero se añade conectividad WiFi para permitir el acceso remoto a los datos. El ESP32 se conecta a una red WiFi definida por un SSID y contraseña y crea un servidor web en el puerto 80. En cada ciclo del loop(), se gestionan las solicitudes de clientes HTTP, generando una página web con HTML y CSS que muestra los valores en tiempo real y se actualiza automáticamente cada segundo. Así, esta parte permite monitorear la frecuencia cardíaca y el SpO₂ tanto en la pantalla OLED como desde cualquier dispositivo en la misma red.











