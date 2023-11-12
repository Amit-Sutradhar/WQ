# WQ


#include "DFRobot_ESP_PH.h"
#include "EEPROM.h"
#include <OneWire.h>
#include <DallasTemperature.h>
#include <WiFi.h>
#include "ThingSpeak.h"  // always include thingspeak header file after other header files and custom macros


#define SECRET_SSID "IIIT-Guest"       // replace MySSID with your WiFi network name
#define SECRET_PASS "I%GR#*S@!"  // replace MyPassword with your WiFi password

// #define SECRET_SSID "IIIT-Guest"		// replace MySSID with your WiFi network name
// #define SECRET_PASS "I%GR#*S@!"	// replace MyPassword with your WiFi password

#define SECRET_CH_ID 2332306                    // replace 0000000 with your channel number
#define SECRET_WRITE_APIKEY "ZCPFB5TXD41L0IMW"  // replace XYZ with your channel write API Key

char ssid[] = SECRET_SSID;  // your network SSID (name)
char pass[] = SECRET_PASS;  // your network password
int keyIndex = 0;           // your network key Index number (needed only for WEP)
WiFiClient client;

unsigned long myChannelNumber = SECRET_CH_ID;
const char* myWriteAPIKey = SECRET_WRITE_APIKEY;
String myStatus = "";

int turbidityPin = 32;  // Turbidity sensor analog pin

// Pin configuration for TDS sensor
#define TdsSensorPin 34  // Replace with the corresponding pin on your ESP32 board
#define VREF 3.3         // Analog reference voltage(Volt) of the ADC
#define SCOUNT 30        // Sum of sample points

// Pin configuration for DS18B20
#define ONE_WIRE_BUS 23
OneWire oneWire(ONE_WIRE_BUS);
DallasTemperature sensors(&oneWire);

DFRobot_ESP_PH ph;
#define ESPADC 4096.0    // The ESP Analog Digital Conversion value
#define ESPVOLTAGE 3300  // The ESP voltage supply value
#define PH_PIN 35        // The ESP GPIO data pin number

int analogBuffer[SCOUNT];  // Store the analog value in the array, read from ADC
int analogBufferIndex = 0;
float tdsValue = 0, temperature = 25;
float waterTemperature=0;

// Declare the 'voltage' and 'phValue' , 'turbidity' variables globally
float voltage, phValue, turbidity;

// Function to get the median value
int getMedian(int bArray[], int iFilterLen)
{
    for (int i = 0; i < iFilterLen - 1; i++)
    {
        for (int j = 0; j < iFilterLen - i - 1; j++)
        {
            if (bArray[j] > bArray[j + 1])
            {
                int temp = bArray[j];
                bArray[j] = bArray[j + 1];
                bArray[j + 1] = temp;
            }
        }
    }
    return iFilterLen % 2 != 0 ? bArray[iFilterLen / 2] : (bArray[iFilterLen / 2 - 1] + bArray[iFilterLen / 2]) / 2;
}

// Function to get the turbidity value
float getTurbidity()
{
    int sensorValue = analogRead(turbidityPin);
    // Adjust the mapping to match the expected range and invert the values
    float turbidity = map(sensorValue, 0, 4095, 250, -105);
    // Print turbidity value
    Serial.print("Turbidity: ");
    Serial.print("   ");
    Serial.println(turbidity);

    // Determine water quality based on turbidity value
    if (turbidity < 30)
    {
        Serial.println("Water is CLEAN");
    }
    else if (turbidity >= 30 && turbidity < 60)
    {
        Serial.println("Water is MODERATE");
    }
    else
    {
        Serial.println("Water is DIRTY");
    }

    return turbidity;
}


// void set up
void setup() {
  Serial.begin(9600);  //Initialize serial
  while (!Serial) {
    ;  // wait for serial port to connect. Needed for Leonardo native USB port only
  }
  pinMode(TdsSensorPin, INPUT);
  WiFi.mode(WIFI_STA);
  sensors.begin();
  EEPROM.begin(32);  // Needed to permit storage of calibration value in EEPROM
  ph.begin();
  ThingSpeak.begin(client);  // Initialize ThingSpeak
}

// Void Loop
void loop() {

  // Connect or reconnect to WiFi
  if (WiFi.status() != WL_CONNECTED) {
    Serial.print("Attempting to connect to SSID: ");
    Serial.println(SECRET_SSID);
    while (WiFi.status() != WL_CONNECTED) {
      WiFi.begin(ssid, pass);  // Connect to WPA/WPA2 network. Change this line if using open or WEP network
      Serial.print(".");
      delay(5000);
    }
    Serial.println("\nConnected.");
  }


  // TDS Sensor
  static unsigned long analogSampleTimepoint = millis();
  if (millis() - analogSampleTimepoint > 40) {
    analogSampleTimepoint = millis();
    analogBuffer[analogBufferIndex] = analogRead(TdsSensorPin);  // Read the analog value and store it in the buffer
    analogBufferIndex = (analogBufferIndex + 1) % SCOUNT;
  }

  static unsigned long printTimepoint = millis();
  if (millis() - printTimepoint > 800) {
    printTimepoint = millis();
    float averageVoltage = getMedian(analogBuffer, SCOUNT) * VREF / 4095.0;                                                         // Convert the analog value to voltage
    float compensationCoefficient = 1.0 + 0.02 * (temperature - 25.0);                                                              // Temperature compensation formula
    float compensationVoltage = averageVoltage / compensationCoefficient;                                                           // Temperature compensation
    tdsValue = (133.42 * pow(compensationVoltage, 3) - 255.86 * pow(compensationVoltage, 2) + 857.39 * compensationVoltage) * 0.5;  // Convert voltage value to TDS value

    // Read temperature from DS18B20
    sensors.requestTemperatures();                        // Send the command to get temperature readings
    waterTemperature = sensors.getTempCByIndex(0);  // Read temperature in Celsius

    // Print TDS value
    Serial.print("TDS Value: ");
    Serial.print(tdsValue, 0);
    Serial.println(" ppm");

    // Print Water temperature
    Serial.print("Water Temperature: ");
    Serial.print(waterTemperature);
    Serial.println(" °C");

    // Turbidity Sensor
    turbidity = getTurbidity();
    // Print Turbidity value
    Serial.print("Turbidity: ");
    Serial.println(turbidity);
  }
  // pH Sensor
  static unsigned long timepoint = millis();
  if (millis() - timepoint > 1000U)  // Time interval: 1s
  {
    timepoint = millis();
    // Calculate voltage
    voltage = analogRead(PH_PIN) / ESPADC * ESPVOLTAGE;  // Read the voltage
    // Print voltage value
    Serial.print("Voltage:");
    Serial.println(voltage, 4);

    // Calculate pH value
    phValue = ph.readPH(voltage, temperature);  // Convert voltage to pH with temperature compensation
    // Print pH value
    Serial.print("pH:");
    Serial.println(phValue, 4);
  }
  // Calibration process by Serial CMD
  ph.calibration(voltage, temperature);

 
  // set the fields with the values
  ThingSpeak.setField(1, tdsValue);
  ThingSpeak.setField(2, waterTemperature);
  ThingSpeak.setField(3, voltage);
  ThingSpeak.setField(4, phValue);
  ThingSpeak.setField(5, turbidity);

  // set the status
  ThingSpeak.setStatus(myStatus);

  // write to the ThingSpeak channel
  int x = ThingSpeak.writeFields(myChannelNumber, myWriteAPIKey);
  if (x == 200) {
    Serial.println("Channel update successful.");
  } else {
    Serial.println("Problem updating channel. HTTP error code " + String(x));
  }

  delay(20000);  // Wait 20 seconds to update the channel again
}
