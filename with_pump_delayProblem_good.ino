#include "DFRobot_ESP_PH.h"
#include "EEPROM.h"
#include <OneWire.h>
#include <DallasTemperature.h>
#include <WiFi.h>
#include "ThingSpeak.h"

#define SECRET_SSID "IIIT-Guest"
#define SECRET_PASS "I%GR#*S@!"
#define SECRET_CH_ID 2332306
#define SECRET_WRITE_APIKEY "ZCPFB5TXD41L0IMW"

// Pin definitions pump
#define MOTOR1_IN1 12
#define MOTOR1_IN2 14
#define MOTOR2_IN3 26
#define MOTOR2_IN4 27


// Motor control function
void controlMotor(int in1, int in2, int duration) {
  digitalWrite(in1, HIGH);
  digitalWrite(in2, LOW);
  delay(duration);
  digitalWrite(in1, LOW);
  digitalWrite(in2, LOW);
}


char ssid[] = SECRET_SSID;
char pass[] = SECRET_PASS;
int keyIndex = 0;
WiFiClient client;

unsigned long myChannelNumber = SECRET_CH_ID;
const char* myWriteAPIKey = SECRET_WRITE_APIKEY;
String myStatus = "";

int turbidityPin = 32;
#define TdsSensorPin 34
#define VREF 3.3
#define SCOUNT 10  // Reduce the number of sample points for simplicity

#define ONE_WIRE_BUS 23
OneWire oneWire(ONE_WIRE_BUS);
DallasTemperature sensors(&oneWire);

DFRobot_ESP_PH ph;
#define ESPADC 4096.0
#define ESPVOLTAGE 3300
#define PH_PIN 35

int analogBuffer[SCOUNT];
float tdsValue = 0, temperature = 25;
float waterTemperature = 0;

float voltage, phValue, turbidity;

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

float getTurbidity()
{
    int sensorValue = analogRead(turbidityPin);
    float turbidity = map(sensorValue, 0, 4095, 250, -105);
    Serial.print("Turbidity: ");
    Serial.print("   ");
    Serial.println(turbidity);

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

void setup()
{
    Serial.begin(9600);
    while (!Serial)
    {
        ;
    }
    pinMode(TdsSensorPin, INPUT);
    WiFi.mode(WIFI_STA);
    sensors.begin();
    EEPROM.begin(32);
    ph.begin();
    ThingSpeak.begin(client);




  // Set motor control pins as OUTPUT
  pinMode(MOTOR1_IN1, OUTPUT);
  pinMode(MOTOR1_IN2, OUTPUT);
  pinMode(MOTOR2_IN3, OUTPUT);
  pinMode(MOTOR2_IN4, OUTPUT);
}

void loop()
{

  
    if (WiFi.status() != WL_CONNECTED)
    {
        Serial.print("Attempting to connect to SSID: ");
        Serial.println(SECRET_SSID);
        while (WiFi.status() != WL_CONNECTED)
        {
            WiFi.begin(ssid, pass);
            Serial.print(".");
            delay(5000);
        }
        Serial.println("\nConnected.");
    }





  
            // Motor 1 on for 10 seconds
                    Serial.println("Motor 1 ON for 10sec");
                    controlMotor(MOTOR1_IN1, MOTOR1_IN2, 10000);








    static unsigned long analogSampleTimepoint = millis();
    if (millis() - analogSampleTimepoint > 40)
    {
        analogSampleTimepoint = millis();
        analogBuffer[0] = analogRead(TdsSensorPin);
    }

    static unsigned long printTimepoint = millis();
    if (millis() - printTimepoint > 800)
    {
        printTimepoint = millis();
        float averageVoltage = analogBuffer[0] * VREF / 4095.0;
        float compensationCoefficient = 1.0 + 0.02 * (temperature - 25.0);
        float compensationVoltage = averageVoltage / compensationCoefficient;
        tdsValue = (133.42 * pow(compensationVoltage, 3) - 255.86 * pow(compensationVoltage, 2) + 857.39 * compensationVoltage) * 0.5;

        sensors.requestTemperatures();
        waterTemperature = sensors.getTempCByIndex(0);

        Serial.print("TDS Value: ");
        Serial.print(tdsValue, 0);
        Serial.println(" ppm");

        Serial.print("Water Temperature: ");
        Serial.print(waterTemperature);
        Serial.println(" °C");

        turbidity = getTurbidity();
        Serial.print("Turbidity: ");
        Serial.println(turbidity);
    }

    static unsigned long timepoint = millis();
    if (millis() - timepoint > 1000U)
    {
        timepoint = millis();
        voltage = analogRead(PH_PIN) / ESPADC * ESPVOLTAGE;
        Serial.print("Voltage:");
        Serial.println(voltage, 4);

        phValue = ph.readPH(voltage, temperature);
        Serial.print("pH:");
        Serial.println(phValue, 4);
    }

    ph.calibration(voltage, temperature);

    ThingSpeak.setField(1, tdsValue);
    ThingSpeak.setField(2, waterTemperature);
    ThingSpeak.setField(3, voltage);
    ThingSpeak.setField(4, phValue);
    ThingSpeak.setField(5, turbidity);

    ThingSpeak.setStatus(myStatus);

    int x = ThingSpeak.writeFields(myChannelNumber, myWriteAPIKey);
    if (x == 200)
    {
        Serial.println("Channel update successful.");
    }
    else
    {
        Serial.println("Problem updating channel. HTTP error code " + String(x));
    }

    delay(20000);


                       // Both motors off for 4m
                    Serial.println("Both Motors OFF for 20sec");
                    digitalWrite(MOTOR1_IN1, LOW);
                    digitalWrite(MOTOR1_IN2, LOW);
                    digitalWrite(MOTOR2_IN3, LOW);
                    digitalWrite(MOTOR2_IN4, LOW);
                    delay(240000); 

{
  // Motor 2 on for 10 seconds
  Serial.println("Motor 2 ON for 10 sec ");
  controlMotor(MOTOR2_IN3, MOTOR2_IN4, 10000);


    // Both motors off for 2 seconds
  Serial.println("Both Motors OFF for 2 sec");
  digitalWrite(MOTOR1_IN1, LOW);
  digitalWrite(MOTOR1_IN2, LOW);
  digitalWrite(MOTOR2_IN3, LOW);
  digitalWrite(MOTOR2_IN4, LOW);
  delay(1000);

  }


}
