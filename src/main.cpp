#include <Arduino.h>
#include <OneWire.h>
#include <DallasTemperature.h>

// OUTPUT
#define GREEN_LED_PIN 7 
#define RED_LED_PIN 8



// Temperature
float tempC;
#define ONE_WIRE_BUS 4
OneWire oneWire(ONE_WIRE_BUS);
DallasTemperature sensors(&oneWire);

// TDS
float tdsValue = 0;
#define TDS_PIN A0
#define TDS_VREF 5.0
#define SCOUNT 30
int analogBuffer[SCOUNT]; // store the analog value in the array, read from ADC
int analogBufferTemp[SCOUNT];
int analogBufferIndex = 0;
int copyIndex = 0;
float averageVoltage = 0;

// Turbidity
float turbidity;
#define TURBIDITY_PIN A1
float turb_volt;
float turb_ntu;

// median filtering algorithm
int getMedianNum(int bArray[], int iFilterLen)
{
    int bTab[iFilterLen];
    for (byte i = 0; i < iFilterLen; i++)
        bTab[i] = bArray[i];
    int i, j, bTemp;
    for (j = 0; j < iFilterLen - 1; j++)
    {
        for (i = 0; i < iFilterLen - j - 1; i++)
        {
            if (bTab[i] > bTab[i + 1])
            {
                bTemp = bTab[i];
                bTab[i] = bTab[i + 1];
                bTab[i + 1] = bTemp;
            }
        }
    }
    if ((iFilterLen & 1) > 0)
    {
        bTemp = bTab[(iFilterLen - 1) / 2];
    }
    else
    {
        bTemp = (bTab[iFilterLen / 2] + bTab[iFilterLen / 2 - 1]) / 2;
    }
    return bTemp;
}
// round off to decimal place
float round_to_dp(float in_value, int decimal_place)
{
    float multiplier = powf(10.0f, decimal_place);
    in_value = roundf(in_value * multiplier) / multiplier;
    return in_value;
}

void setup()
{
    // Mushy Setup
    Serial.begin(9600);

    // Digital Setup
    pinMode(LED_BUILTIN, OUTPUT);
    pinMode(TDS_PIN, INPUT);
    pinMode(GREEN_LED_PIN, OUTPUT);
    pinMode(RED_LED_PIN, OUTPUT);

    // Activations
    sensors.begin();
    if (sensors.getDeviceCount() == 0)
    {
        Serial.println("No DS18B20 sensors found. Check connections.");
    }
}

void loop()
{
    // Temperature ----------------------------------------------------------------------------------------------------
    sensors.requestTemperatures();
    digitalWrite(LED_BUILTIN, HIGH);
    tempC = sensors.getTempCByIndex(0);

    if (tempC == -127.00)
    {
        Serial.println("Error: Could not read temperature. Check connections. ()");
    }

    // TDS ------------------------------------------------------------------------------------------------------------
    static unsigned long analogSampleTimepoint = millis();
    if (millis() - analogSampleTimepoint > 40U)
    { /* every 40 milliseconds,read the analog value from the ADC */
        analogSampleTimepoint = millis();
        analogBuffer[analogBufferIndex] = analogRead(TDS_PIN); // read the analog value and store into the buffer
        analogBufferIndex++;
        if (analogBufferIndex == SCOUNT)
        {
            analogBufferIndex = 0;
        }
    }
    static unsigned long printTimepoint = millis();
    if (millis() - printTimepoint > 800U)
    {
        printTimepoint = millis();
        for (copyIndex = 0; copyIndex < SCOUNT; copyIndex++)
        {
            analogBufferTemp[copyIndex] = analogBuffer[copyIndex];

            // read the analog value more stable by the median filtering algorithm, and convert to voltage value
            averageVoltage = getMedianNum(analogBufferTemp, SCOUNT) * (float)TDS_VREF / 1024.0;

            // temperature compensation formula: fFinalResult(25^C) = fFinalResult(current)/(1.0+0.02*(fTP-25.0));
            float compensationCoefficient = 1.0 + 0.02 * (tempC - 25.0);
            float compensationVoltage = averageVoltage / compensationCoefficient; // temperature compensation

            tdsValue = (133.42 * compensationVoltage * compensationVoltage * compensationVoltage - 255.86 * compensationVoltage * compensationVoltage + 857.39 * compensationVoltage) * 0.5; // convert voltage value to tds value

        }
    }

    // Turbidity ------------------------------------------------------------------------------------------------------------
    turb_volt = analogRead(TURBIDITY_PIN) * (5.0 / 1024.0);
    if (turb_volt < 2.5)
    {
        turb_ntu = 3000;
    }
    else
    {
        turb_ntu = -1120.4 * square(turb_volt) + 5742.3 * turb_volt - 4353.8;
    }

    // OUTPUT values
    Serial.print("Temperature:");
    Serial.println(tempC);

    Serial.print("TDS Value:");
    Serial.print(tdsValue, 0);
    Serial.println("ppm");

    Serial.print("NTU:"); // Nephelometric Turbidity unit
    Serial.print(turb_ntu);
    Serial.print("\tVoltage:");
    Serial.println(turb_volt);


    // OUTPUT Interface
    if (tdsValue < 250)
    {
        digitalWrite(GREEN_LED_PIN, HIGH);
        digitalWrite(RED_LED_PIN, LOW);
    }else if (tdsValue < 300)
    {
        digitalWrite(GREEN_LED_PIN, HIGH);
        digitalWrite(RED_LED_PIN, HIGH);
    }else{
        digitalWrite(GREEN_LED_PIN, LOW);
        digitalWrite(RED_LED_PIN, HIGH);
    }
    


    digitalWrite(LED_BUILTIN, LOW);
    Serial.println(); // Marks the end of reading of 3 SENSORS

    // delay(0.5); // 0.5 ms
}