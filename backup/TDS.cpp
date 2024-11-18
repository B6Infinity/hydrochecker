#include <Arduino.h>
#include <OneWire.h>
#include <DallasTemperature.h>
#include <EEPROM.h>

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

void setup()
{
    // Mushy Setup
    Serial.begin(9600);

    // Digital Setup
    pinMode(LED_BUILTIN, OUTPUT);
    pinMode(TDS_PIN, INPUT);

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
    {   /* every 40 milliseconds,read the analog value from the ADC */
        analogSampleTimepoint = millis();
        analogBuffer[analogBufferIndex] = analogRead(TDS_PIN); // read the analog value and store into the buffer
        analogBufferIndex++;
        if (analogBufferIndex == SCOUNT)
        {
            analogBufferIndex = 0;
        }
    }
    static unsigned long printTimepoint = millis();
    if(millis()-printTimepoint > 800U){
        printTimepoint = millis();
        for(copyIndex=0; copyIndex<SCOUNT; copyIndex++){
            analogBufferTemp[copyIndex] = analogBuffer[copyIndex];
      
            // read the analog value more stable by the median filtering algorithm, and convert to voltage value
            averageVoltage = getMedianNum(analogBufferTemp,SCOUNT) * (float)TDS_VREF / 1024.0;
      
            //temperature compensation formula: fFinalResult(25^C) = fFinalResult(current)/(1.0+0.02*(fTP-25.0)); 
            float compensationCoefficient = 1.0+0.02*(tempC-25.0);
            float compensationVoltage=averageVoltage/compensationCoefficient; //temperature compensation
            
            tdsValue=(133.42*compensationVoltage*compensationVoltage*compensationVoltage - 255.86*compensationVoltage*compensationVoltage + 857.39*compensationVoltage)*0.5;//convert voltage value to tds value
            
            //Serial.print("voltage:");
            //Serial.print(averageVoltage,2);
            //Serial.print("V   ");
            // Serial.print("TDS Value:");
            // Serial.print(tdsValue,0);
            // Serial.println("ppm");
        }
    }



    // OUTPUT values
    Serial.print("Temperature:");
    Serial.println(tempC);

    Serial.print("TDS Value:");
    Serial.print(tdsValue,0);
    Serial.println("ppm");

    

    digitalWrite(LED_BUILTIN, LOW);
    // delay(0.5); // 0.5 ms
}