#include <Arduino.h>
#include <Adafruit_Sensor.h>
#include <DHT.h>

// Definir pines
#define SENSOR4466_PIN 34  // Pin analógico en ESP32 para el sensor de sonido MAX4466
#define DHT22_PIN 14       // Pin digital en ESP32 para el sensor DHT22
#define LuzLED_PIN 13      // Pin luz LED en ESP32

// Definir tipo de sensor DHT
#define DHTTYPE DHT22   // Definir que es un sensor DHT22

// Crear objeto DHT
DHT dht(DHT22_PIN, DHTTYPE);

const int sampleWindow = 30;  // Ancho de ventana de muestra en ms (30 ms = 33.33Hz) 
unsigned int sample;

void setup() {
  pinMode(SENSOR4466_PIN, INPUT); // Establecer el pin de SENSOR4466 como entrada
  pinMode(LuzLED_PIN, OUTPUT);    // Configurar el pin luz LED como salida
  digitalWrite(LuzLED_PIN, LOW);  // Apagar luz LED inicialmente
  Serial.begin(115200);           // Velocidad serial más alta para ESP32
  dht.begin();                    // Inicializar sensor DHT22
}

void loop() {
  // Lectura del sensor de sonido MAX4466
  unsigned long startMillis = millis();  // Start of sample window
  float peakToPeak = 0;  // Peak-to-peak level

  unsigned int signalMax = 0;   // Minimo valor
  unsigned int signalMin = 4095; // Maximo valor para ESP32 (12-bit ADC)

  // Recolectar datos durante la ventana de muestra
  while (millis() - startMillis < sampleWindow) {
    sample = analogRead(SENSOR4466_PIN);  // Obtener lectura del micrófono

    if (sample > signalMax) {
      signalMax = sample;  // Guardar el valor máximo
    }
    if (sample < signalMin) {
      signalMin = sample;  // Guardar el valor mínimo
    }
  }

  peakToPeak = signalMax - signalMin;  // Amplitud pico-pico
  int db = map(peakToPeak, 0, 4095, 40, 100);  // Ajustar según el sensor

  if (isnan(db)) {
    Serial.println("Error de lectura del sensor MAX4466");
  } else {
  Serial.print("Nivel de sonido: ");
  Serial.print(db);
  Serial.println(" dB");
  }

  // Si el nivel de decibeles es mayor a 80, hacer parpadear el LED
  if (db > 80) {
    Serial.println("Activando LED");  // Verificar si entra aquí
    digitalWrite(LuzLED_PIN, HIGH);
    delay(500);
    digitalWrite(LuzLED_PIN, LOW);
    delay(500);
  } else {
    Serial.println("LED apagado");  // Mensaje de verificación
    digitalWrite(LuzLED_PIN, LOW);
  }

  // Lectura del sensor DHT22 (Temperatura y Humedad)
  float temperatura = dht.readTemperature();
  float humedad = dht.readHumidity();

  // Verificar si la lectura fue exitosa o si hubo un error
  if (isnan(temperatura) || isnan(humedad)) {
    Serial.println("Error de lectura del sensor DHT22");
  } else {
    Serial.print("Temperatura: ");
    Serial.print(temperatura);
    Serial.println(" °C");

    Serial.print("Humedad: ");
    Serial.print(humedad);
    Serial.println(" %");
  }

  // Esperar 2 segundos antes de la siguiente lectura
  delay(2000);
}