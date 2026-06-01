#include <Wire.h>
#include <Adafruit_Sensor.h>
#include <Adafruit_GFX.h>
#include <Adafruit_SSD1306.h>
#include <BH1750.h>
#include <Adafruit_BME680.h>
#include <WiFi.h>
#include <ThingSpeak.h>
#include <math.h>


#define SCREEN_WIDTH 128
#define SCREEN_HEIGHT 64
#define OLED_RESET -1
Adafruit_SSD1306 display(SCREEN_WIDTH, SCREEN_HEIGHT, &Wire, OLED_RESET);


#define SENSOR_PIN_MOISTURE 33
#define SENSOR_PIN_MQ4      35
#define SENSOR_PIN_MQ137    32
#define SENSOR_PIN_DUST     34
#define DUST_LED_PIN        14
#define FLOW_SENSOR_PIN     13


#define RELAY_PUMP_PIN      12
#define FAN_PIN             27
#define BUZZER_PIN          26
#define RED_LED_PIN         25

#define SEALEVELPRESSURE_HPA (1013.25)
Adafruit_BME680 bme;


BH1750 lightMeter;

const char *ssid = "YOUR_WIFI_NAME";
const char *password = "YOUR_WIFI_PASSWORD";
unsigned long myChannelNumber = YOUR_CHANNEL_NUMBER;
const char *myWriteAPIKey = "YOUR_WRITE_API_KEY";
WiFiClient client;

const float TEMP_LIMIT_C   = 39.0;
const float HUM_LIMIT_RH   = 70.0;
const float NH3_LIMIT_PPM  = 50.0;
const float CH4_LIMIT_PPM  = 100.0;
const float DUST_LIMIT_MG  = 15.0;
const float LIGHT_LIMIT_LX = 60.0;


const float MIN_VALID_FLOW_LMIN = 0.20;   


float moistureValue = 0.0;
int litterValue = 0;  // 1=Wet, 2=Perfect, 3=Dry

float sensorValueMQ4 = 0.0;
float sensorValueMQ137 = 0.0;
float ppmCH4 = 0.0;
float ppmNH3 = 0.0;

float voMeasured = 0.0;
float calcVoltage = 0.0;
float dustDensity = 0.0;

float temperatureC = 0.0;
float humidityRH = 0.0;
float lux = 0.0;

// Datasheet-style relation: Frequency (Hz) = 7.5 × Flow rate (L/min)
// Therefore: Flow rate (L/min) = Frequency / 7.5
// Also around 450 pulses per liter
volatile unsigned long flowPulseCount = 0;
unsigned long totalFlowPulses = 0;
float flowRateLMin = 0.0;
float flowRateLHour = 0.0;
float totalLiters = 0.0;
unsigned long oldTime = 0;
const float calibrationFactor = 7.5;
const float pulsesPerLiter = 450.0;

bool tempHumAlert = false;
bool gasAlert = false;
bool dustAlert = false;
bool lightAlert = false;
bool litterAlert = false;
bool flowFaultAlert = false;


void IRAM_ATTR pulseCounter() {
  flowPulseCount++;
}


// Replace with your calibrated equations if available
float estimateMQ4ppm(float rawADC) {
  if (rawADC <= 0) return 0;
  return pow((rawADC / 1000.0), 1.5) * 10.0;
}

float estimateMQ137ppm(float rawADC) {
  if (rawADC <= 0) return 0;
  return pow((rawADC / 1000.0), 1.5) * 10.0;
}

void setup() {
  Serial.begin(115200);

  pinMode(DUST_LED_PIN, OUTPUT);
  pinMode(RELAY_PUMP_PIN, OUTPUT);
  pinMode(FAN_PIN, OUTPUT);
  pinMode(BUZZER_PIN, OUTPUT);
  pinMode(RED_LED_PIN, OUTPUT);

  pinMode(SENSOR_PIN_MOISTURE, INPUT);
  pinMode(SENSOR_PIN_MQ4, INPUT);
  pinMode(SENSOR_PIN_MQ137, INPUT);
  pinMode(SENSOR_PIN_DUST, INPUT);

  pinMode(FLOW_SENSOR_PIN, INPUT_PULLUP);

  digitalWrite(RELAY_PUMP_PIN, LOW);
  digitalWrite(FAN_PIN, LOW);
  digitalWrite(BUZZER_PIN, LOW);
  digitalWrite(RED_LED_PIN, LOW);
  digitalWrite(DUST_LED_PIN, HIGH);

  Wire.begin();
  lightMeter.begin();

  if (!bme.begin()) {
    Serial.println("Could not find BME680 sensor!");
    while (1);
  }

  if (!display.begin(SSD1306_SWITCHCAPVCC, 0x3C)) {
    Serial.println("SSD1306 allocation failed");
    while (1);
  }

  display.clearDisplay();
  display.display();

  attachInterrupt(digitalPinToInterrupt(FLOW_SENSOR_PIN), pulseCounter, FALLING);
  oldTime = millis();

  WiFi.begin(ssid, password);
  Serial.print("Connecting to WiFi");
  while (WiFi.status() != WL_CONNECTED) {
    delay(1000);
    Serial.print(".");
  }
  Serial.println("\nConnected to WiFi");

  ThingSpeak.begin(client);
}

void loop() {
 
  moistureValue = analogRead(SENSOR_PIN_MOISTURE);

  if (moistureValue < 1500) {
    litterValue = 1;   // Wet
  } else if (moistureValue >= 1500 && moistureValue < 3000) {
    litterValue = 2;   // Perfect
  } else {
    litterValue = 3;   // Dry
  }


  sensorValueMQ4 = analogRead(SENSOR_PIN_MQ4);
  sensorValueMQ137 = analogRead(SENSOR_PIN_MQ137);

  ppmCH4 = estimateMQ4ppm(sensorValueMQ4);
  ppmNH3 = estimateMQ137ppm(sensorValueMQ137);

  
  digitalWrite(DUST_LED_PIN, LOW);
  delayMicroseconds(280);
  voMeasured = analogRead(SENSOR_PIN_DUST);
  delayMicroseconds(40);
  digitalWrite(DUST_LED_PIN, HIGH);
  delayMicroseconds(9680);

  calcVoltage = voMeasured * (3.3 / 4095.0);
  dustDensity = 0.17 * calcVoltage - 0.1;
  if (dustDensity < 0) dustDensity = 0.0;


  if (!bme.performReading()) {
    Serial.println("Failed to perform BME680 reading");
    delay(2000);
    return;
  }

  temperatureC = bme.temperature;
  humidityRH = bme.humidity;

 
  lux = lightMeter.readLightLevel();
  if (lux < 0) lux = 0.0;


  if ((millis() - oldTime) >= 1000) {
    noInterrupts();
    unsigned long pulseCount = flowPulseCount;
    flowPulseCount = 0;
    interrupts();

    totalFlowPulses += pulseCount;

  
    float frequency = (float)pulseCount;

   
    flowRateLMin = frequency / calibrationFactor;
    flowRateLHour = flowRateLMin * 60.0;

  
    totalLiters = totalFlowPulses / pulsesPerLiter;

    oldTime = millis();
  }


  tempHumAlert = (humidityRH > HUM_LIMIT_RH && temperatureC > TEMP_LIMIT_C);
  gasAlert = (ppmNH3 > NH3_LIMIT_PPM || ppmCH4 > CH4_LIMIT_PPM);
  dustAlert = (dustDensity >= DUST_LIMIT_MG);
  lightAlert = (lux > LIGHT_LIMIT_LX);
  litterAlert = (litterValue == 1);


  flowFaultAlert = (tempHumAlert && flowRateLMin < MIN_VALID_FLOW_LMIN);

  bool pumpOn = false;
  bool fanOn = false;
  bool buzzerOn = false;
  bool redLedOn = false;

 
  if (tempHumAlert) {
    pumpOn = true;
    fanOn = true;
    buzzerOn = true;
  }

 
  if (gasAlert) {
    fanOn = true;
    buzzerOn = true;
    redLedOn = true;
  }

  
  if (dustAlert || lightAlert || litterAlert) {
    buzzerOn = true;
    redLedOn = true;
  }

  
  if (flowFaultAlert) {
    buzzerOn = true;
    redLedOn = true;
    fanOn = true;
  }

  digitalWrite(RELAY_PUMP_PIN, pumpOn ? HIGH : LOW);
  digitalWrite(FAN_PIN, fanOn ? HIGH : LOW);
  digitalWrite(BUZZER_PIN, buzzerOn ? HIGH : LOW);
  digitalWrite(RED_LED_PIN, redLedOn ? HIGH : LOW);


  Serial.println("------ Poultry Farm Monitoring ------");
  Serial.print("Temperature: "); Serial.print(temperatureC); Serial.println(" C");
  Serial.print("Humidity: "); Serial.print(humidityRH); Serial.println(" %");
  Serial.print("NH3: "); Serial.print(ppmNH3); Serial.println(" ppm");
  Serial.print("CH4: "); Serial.print(ppmCH4); Serial.println(" ppm");
  Serial.print("Dust: "); Serial.print(dustDensity); Serial.println(" mg/m3");
  Serial.print("Light: "); Serial.print(lux); Serial.println(" lux");
  Serial.print("Flow Rate: "); Serial.print(flowRateLMin, 2); Serial.println(" L/min");
  Serial.print("Flow Rate: "); Serial.print(flowRateLHour, 2); Serial.println(" L/hour");
  Serial.print("Total Water: "); Serial.print(totalLiters, 3); Serial.println(" L");

  Serial.print("Litter: ");
  if (litterValue == 1) Serial.println("Wet");
  else if (litterValue == 2) Serial.println("Perfect");
  else Serial.println("Dry");

  if (flowFaultAlert) {
    Serial.println("ALERT: Pump ON but no/low water flow detected!");
  }


  ThingSpeak.setField(1, (int)ppmCH4);        // CH4
  ThingSpeak.setField(2, (int)ppmNH3);        // NH3
  ThingSpeak.setField(3, dustDensity);        // Dust
  ThingSpeak.setField(4, temperatureC);       // Temperature
  ThingSpeak.setField(5, humidityRH);         // Humidity
  ThingSpeak.setField(6, (int)lux);           // Light
  ThingSpeak.setField(7, litterValue);        // Litter
  ThingSpeak.setField(8, flowRateLHour);      // Real-time water flow in L/h

  String statusText = "T:" + String(temperatureC, 1) +
                      " H:" + String(humidityRH, 1) +
                      " NH3:" + String(ppmNH3, 1) +
                      " CH4:" + String(ppmCH4, 1);

  if (flowFaultAlert) {
    statusText += " FLOW_FAULT";
  }

  ThingSpeak.setStatus(statusText);

  int x = ThingSpeak.writeFields(myChannelNumber, myWriteAPIKey);
  if (x == 200) {
    Serial.println("Data uploaded successfully");
  } else {
    Serial.print("ThingSpeak upload failed. Code: ");
    Serial.println(x);
  }

  // ---------- OLED Display ----------
  display.clearDisplay();
  display.setTextColor(WHITE);
  display.setTextSize(1);

  display.setCursor(0, 0);
  display.print("T:");
  display.print(temperatureC, 1);
  display.print("C H:");
  display.print((int)humidityRH);
  display.print("%");

  display.setCursor(0, 10);
  display.print("NH3:");
  display.print((int)ppmNH3);
  display.print(" CH4:");
  display.print((int)ppmCH4);

  display.setCursor(0, 20);
  display.print("D:");
  display.print(dustDensity, 1);
  display.print(" Lx:");
  display.print((int)lux);

  display.setCursor(0, 30);
  display.print("F:");
  display.print(flowRateLMin, 2);
  display.print("L/m");

  display.setCursor(0, 40);
  display.print("TW:");
  display.print(totalLiters, 2);
  display.print("L");

  display.setCursor(0, 50);
  if (flowFaultAlert) {
    display.print("ALERT: FLOW LOW");
  } else if (litterValue == 1) {
    display.print("Litter: Wet");
  } else if (litterValue == 2) {
    display.print("Litter: Perfect");
  } else {
    display.print("Litter: Dry");
  }

  display.display();

  delay(10000);
}