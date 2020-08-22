#include <Arduino.h>
#include <EEPROM.h>
#include "rotEncoder.h"
#include <Wire.h> 
#include <LiquidCrystal_I2C.h>
#include <Adafruit_ADS1015.h>
#include <SPI.h>
#include <LoRa.h>

String sensorAddr = "A1"; // sensor address 
String reqIn; // variable to store sensor address

#define LCD_PRINT_DELAY 1000
#define LCD_MSG_DELAY 2000
#define Backlight_ON_DELAY 15000

// EEPROM Addresses
#define INIT_CHK_ADDR 100
#define TEMP_CAL_MIN_ADDR 10
#define TEMP_CAL_MAX_ADDR 12
#define RH_CAL_MIN_ADDR 14
#define RH_CAL_MAX_ADDR 16
#define TEMP_ALARM_MIN_ADDR 18
#define TEMP_ALARM_MAX_ADDR 20
#define RH_ALARM_MIN_ADDR 22
#define RH_ALARM_MAX_ADDR 24
#define PASSCODE_STARTING_ADDR 26

// raw values from ads1117
#define SENSE_RAW_MIN 4680 // for 4mA current through 220Ohms resistor
#define SENSE_RAW_MAX 23270 // for 20mA current through 220Ohms resistor

float tempCalMin, tempCalMax, rhCalMin, rhCalMax, tempAlarmMin, tempAlarmMax, rhAlarmMin, rhAlarmMax;
float tempCalValue, rhCalValue;

LiquidCrystal_I2C lcd(0x3f, 20, 4); 
RotEncoder rotE1(3, 5, 4);// swPin , clkPin , dtPin
RotEncoder rotE2(6, 8, 7);// swPin , clkPin , dtPin
Adafruit_ADS1115 ads;

byte passCode[5] = {1, 1, 1, 1};
byte engCode[5] = {1, 6, 6, 6};
boolean lcdBlinker;

// delay settings for the lcd
unsigned long lcdPrintDelay, lcdClearDelay; 
unsigned long printDelay, bclightOnDelay, buzzerDelay;

uint16_t buzzerDly = 500;

// rot encoder variables
int prevCount1;
int prevRotVal1;

int prevCount2;
int prevRotVal2;

byte buzzerPin = A3;

void setup()
{
    Serial.begin(9600);
    delay(50);
    LoRa.begin(433E6);

    
    ads.begin(); // analog extension module
    lcd.init();
    lcd.backlight();
    pinMode(buzzerPin, OUTPUT);
    //------------------------------------------
    // ---If Passcode has not set yet user has to go through this part---
    if (EEPROM.read(INIT_CHK_ADDR) != 23)
    {
        setPassword();
        setTempAlarm();
        setRhAlarm();
        tempCal(1);
        rhCal(1);
        EEPROM.write(INIT_CHK_ADDR, 23);
    }
    //------------------------------------------
    //---read eeprom for initiation-------------
    
    tempCalMin = readFromEeprom(TEMP_CAL_MIN_ADDR);
    tempCalMax = readFromEeprom(TEMP_CAL_MAX_ADDR);
    tempCalValue = (float)(tempCalMin + tempCalMax - 75.00) / 2.00;

    rhCalMin = readFromEeprom(RH_CAL_MIN_ADDR);
    rhCalMax = readFromEeprom(RH_CAL_MAX_ADDR);
    rhCalValue = (float)(rhCalMin + rhCalMax - 115.00) / 2.00;

    tempAlarmMin = readFromEeprom(TEMP_ALARM_MIN_ADDR);
    tempAlarmMax = readFromEeprom(TEMP_ALARM_MAX_ADDR);
    rhAlarmMin = readFromEeprom(RH_ALARM_MIN_ADDR);
    rhAlarmMax = readFromEeprom(RH_ALARM_MAX_ADDR);
    
    for (int8_t i = 0; i < 4; i++)
    {
        passCode[i] = EEPROM.read(PASSCODE_STARTING_ADDR + i);
    }
    
    // Print to serial port
    Serial.print(tempCalMin);
    Serial.print(" | ");
    Serial.print(tempCalMax);
    Serial.print(" | ");
    Serial.print(rhCalMin);
    Serial.print(" | ");
    Serial.print(rhCalMax);
    Serial.print(" | ");
    Serial.print(tempAlarmMin);
    Serial.print(" | ");
    Serial.print(tempAlarmMax);
    Serial.print(" | ");
    Serial.print(rhAlarmMin);
    Serial.print(" | ");
    Serial.print(rhAlarmMax);
    Serial.print(" | ");
    Serial.print(passCode[0]);
    Serial.print(" | ");
    Serial.print(passCode[1]);
    Serial.print(" | ");
    Serial.print(passCode[2]);
    Serial.print(" | ");
    Serial.print(passCode[3]);
    Serial.println();
    
    bclightOnDelay = millis();
}

void loop()
{
    
    backLightCtrl(Backlight_ON_DELAY); // backlight off after defined time ends

    lcdClr(60000); // lcd clears after every defined time
    // show values on display
    normalDisplay(tempReadVal(), rhReadVal(), tempAlarmMin, tempAlarmMax, rhAlarmMin, rhAlarmMax);//tempInVal, rhInVal, tempMinVal, tempMaxVal, rhMinVal, rhMaxVal
    if (rotE1.pushCounter() >= 3) //access to menu
    {
        selectorMenu(accessToMenu(&passCode[0], "")); // prompt the PW
        bclightOnDelay = millis(); // on the backlight for some time
    }

    alarmCtrl(); // alarms
   
    // transfer data through Lora
    transferData(tempReadVal(), rhReadVal()); //temp, rh
    
}

void backLightCtrl(uint16_t inDelay) // back light on off control
{
    int count1 = rotE1.pushCounter();
    int rotVal1 = rotE1.rotVal() / 2;

    int count2 = rotE2.pushCounter();
    int rotVal2 = rotE2.rotVal() / 2;

    rotE1.pushCounterResetTimer(1000);
    rotE2.pushCounterResetTimer(1000);
    // backlight on if any interaction occures
    if (prevCount1 != count1 || prevCount2 != count2 || prevRotVal1 != rotVal1 || prevRotVal2 != rotVal2)
    {
        lcd.backlight();
        bclightOnDelay = millis();
    }
    if (millis() - bclightOnDelay > inDelay)
    {
        lcd.setBacklight(0);
    }

    prevCount1 = count1;
    prevRotVal1 = rotVal1;

    prevCount2 = count2;
    prevRotVal2 = rotVal2;
}

void setPassword() // set/ change password
{
    rotE1.pushCounterReset();
    rotE2.pushCounterReset();
    lcd.clear();
    lcd.setCursor(2, 0);
    lcd.print("Change the Code:");

    int8_t pCountMinus = 0;
    int8_t pCountPlus = 0;
    int8_t pCount = 0;

    while (pCount >= 0 && pCount < 4)
    {
        pCountPlus = rotE1.pushCounter();
        pCountMinus = rotE2.pushCounter();
        pCount = pCountPlus - pCountMinus;
        if (pCount == 0)
        {
            passCode[pCount] = rotE1.rotValBetween() / 2;
        }
        else if (pCount == 1) // if user presses the button  move to next charater
        {
            passCode[pCount] = rotE1.rotValBetween() / 2;
        }
        else if (pCount == 2)// if user presses the button  move to next charater
        {
            passCode[pCount] = rotE1.rotValBetween() / 2;
        }
        else if (pCount == 3)// if user presses the button  move to next charater
        {
            passCode[pCount] = rotE1.rotValBetween() / 2;
        }

        if (millis() - lcdPrintDelay > LCD_PRINT_DELAY / 2)// show on lcd every defined time
        {
            if (pCount == 0)
            {
                lcd.setCursor(6, 2);
                lcdBlinkChar(passCode[pCount]);

                lcd.print(" * * *");
            }
            else if (pCount == 1)
            {
                lcd.setCursor(6, 2);

                lcd.print("* ");
                lcdBlinkChar(passCode[pCount]);

                lcd.print(" * *");
            }
            else if (pCount == 2)
            {
                lcd.setCursor(6, 2);

                lcd.print("* * ");
                lcdBlinkChar(passCode[pCount]);

                lcd.print(" *");
            }
            else if (pCount == 3)
            {
                lcd.setCursor(6, 2);

                lcd.print("* * * ");
                lcdBlinkChar(passCode[pCount]);
            }
            lcdPrintDelay = millis();
        }
    }

    if (pCount == 4) 
    {
        lcd.setCursor(6, 2);
        lcd.print("* * * *");
        delay(1000);
        lcd.clear();
        lcd.setCursor(4, 1);
        lcd.print("Code Changed");
        lcd.setCursor(4, 2);
        lcd.print("Successfully");
        lcd.setCursor(4, 3);

        for (int8_t i = 0; i < 4; i++)
        {
            EEPROM.write(PASSCODE_STARTING_ADDR + i, passCode[i]);
        }
        
        delay(LCD_MSG_DELAY);
    }
    else if (pCount < 0) // if cancel button pressed => cancel
    {
        lcd.clear();
        lcd.setCursor(4, 1);
        lcd.print("Cancelled..");
        delay(LCD_MSG_DELAY);
    }

    rotE1.pushCounterReset();
    rotE2.pushCounterReset();
    lcd.clear();
}

// LCD display 
void normalDisplay(float tempInVal, float rhInVal, float tempMinVal, float tempMaxVal, float rhMinVal, float rhMaxVal)
{
    if (millis() - lcdPrintDelay > LCD_PRINT_DELAY)
    {
        lcd.setCursor(11, 0);
        lcd.print("Min  Max");

        lcd.setCursor(0, 1);
        lcd.print("T(");
        lcd.print(char(0xDF));
        lcd.print("C)");

        lcd.setCursor(6, 1);
        lcd.print(tempInVal, 1);
        lcd.setCursor(11, 1);
        lcd.print(tempMinVal, 1);
        lcd.setCursor(16, 1);
        lcd.print(tempMaxVal, 1);

        lcd.setCursor(0, 2);
        lcd.print("RH(%) ");


        lcd.print(rhInVal, 1);
        lcd.setCursor(11, 2);
        lcd.print(rhMinVal, 1);
        lcd.setCursor(16, 2);
        lcd.print(rhMaxVal, 1);

        lcd.setCursor(0, 3);
        lcdPrintDelay = millis();
    }
}
//-----------------------------------------------
void lcdBlinkChar(char inVal)// blink enterring character
{
    if (!lcdBlinker)
    {
        lcd.print(inVal);
        lcdBlinker = 1;
    }
    else
    {
        lcd.print(" ");
        lcdBlinker = 0;
    }
}

void lcdBlinkChar(int16_t inVal)// blink enterring integer
{
    if (!lcdBlinker)
    {
        lcd.print(inVal);
        lcdBlinker = 1;
    }
    else
    {
        lcd.print(" ");
        lcdBlinker = 0;
    }
}
//--------------------------------------------------

void selectorMenu(boolean inVal) // Main menu
{
    if (inVal)
    {
        rotE1.pushCounterReset();
        rotE2.pushCounterReset();
        lcd.clear();

        int8_t pCountMinus = 0;
        int8_t pCountPlus = 0;
        int8_t pCount = 0;

        while (pCount >= 0 && pCount < 2)
        {
            pCountPlus = rotE1.pushCounter();
            pCountMinus = rotE2.pushCounter();
            pCount = pCountPlus - pCountMinus;

            String menuItems[6] = { " Change Code      ", " Change Temp Alarm ", " Change RH Alarm   ", " Temp Calibration ", " RH Calibration  "};
            int8_t rotEVal = rotE1.rotValBetween(0, 8) / 2;
            //=================================
            if (pCount > 0)
            {
                if (rotEVal == 0) { setPassword(); }//<-----------
                else if (rotEVal == 1) { setTempAlarm(); }
                else if (rotEVal == 2) { setRhAlarm(); }
                else if (rotEVal == 3) { tempCal(accessToMenu(&engCode[0], "Manufacturer ")); }
                else if (rotEVal == 4) { rhCal(accessToMenu(&engCode[0], "Manufacturer ")); }

                rotE1.pushCounterReset();
                rotE2.pushCounterReset();
                pCount = 0;
            }

            if (millis() - lcdPrintDelay > LCD_PRINT_DELAY / 3)
            {
                lcd.setCursor(8, 0);
                lcd.print("Menu");

                lcd.setCursor(0, 2);
                lcdBlinkChar('>');
                lcd.print(menuItems[rotEVal]);

                lcdPrintDelay = millis();
            }
        }

        if (pCount < 0)
        {
            lcd.clear();
            lcd.setCursor(0, 1);
            lcd.print("Exit from the menu");
            delay(LCD_MSG_DELAY);
        }
        lcd.clear();
    }
    //------------------------------------------------------------
}

void setTempAlarm() // temp alarm set
{
    rotE1.pushCounterReset();
    rotE2.pushCounterReset();
    lcd.clear();

    int8_t pCountMinus = 0;
    int8_t pCountPlus = 0;
    int8_t pCount = 0;

    float rotaryVal1;
    float rotaryVal2;

    while (pCount >= 0 && pCount < 1)
    {
        pCountPlus = rotE1.pushCounter();
        pCountMinus = rotE2.pushCounter();
        pCount = pCountPlus - pCountMinus;

        rotaryVal1 = 15.00 - (float)rotE1.rotVal() / 2.00;
        rotaryVal2 = 30.00 - (float)rotE2.rotVal() / 2.00;

        if (millis() - lcdPrintDelay > LCD_PRINT_DELAY / 3)
        {
            lcd.setCursor(0, 0);
            lcd.print("Temperature Alarm");
            lcd.setCursor(0, 1);
            lcd.print("Set to: ");

            lcd.setCursor(5, 2);
            lcd.print("Min = ");
            lcd.print(rotaryVal1, 1);
            lcd.print(char(0xDF));
            lcd.print("C ");

            lcd.setCursor(5, 3);
            lcd.print("Max = ");
            lcd.print(rotaryVal2, 1);
            lcd.print(char(0xDF));
            lcd.print("C ");

            tempAlarmMin = rotaryVal1;
            tempAlarmMax = rotaryVal2;
            write2Eeprom(TEMP_ALARM_MIN_ADDR, rotaryVal1);
            write2Eeprom(TEMP_ALARM_MAX_ADDR, rotaryVal2);

            lcdPrintDelay = millis();
        }
    }
    if (pCount > 0)
    {
        lcd.setCursor(0, 0);
        lcd.print("Temperature Alarm");
        lcd.setCursor(0, 1);
        lcd.print("Values Chaneged to: ");

        lcd.setCursor(5, 2);
        lcd.print("Min = ");
        lcd.print(rotaryVal1, 1);
        lcd.print(char(0xDF));
        lcd.print("C ");

        lcd.setCursor(5, 3);
        lcd.print("Max = ");
        lcd.print(rotaryVal2, 1);
        lcd.print(char(0xDF));
        lcd.print("C ");

        delay(LCD_MSG_DELAY);
        lcd.clear();
    }

    if (pCount < 0)
    {
        lcd.clear();
        lcd.setCursor(4, 1);
        lcd.print("Cancelled...");

        delay(LCD_MSG_DELAY);
        lcd.clear();
    }
}

void setRhAlarm() // rh alarm set
{
    rotE1.pushCounterReset();
    rotE2.pushCounterReset();
    lcd.clear();

    int8_t pCountMinus = 0;
    int8_t pCountPlus = 0;
    int8_t pCount = 0;

    float rotaryVal1;
    float rotaryVal2;

    while (pCount >= 0 && pCount < 1)
    {
        pCountPlus = rotE1.pushCounter();
        pCountMinus = rotE2.pushCounter();
        pCount = pCountPlus - pCountMinus;

        rotaryVal1 = 10.00 - (float)rotE1.rotVal() / 2.00;
        rotaryVal2 = 90.00 - (float)rotE2.rotVal() / 2.00;

        if (millis() - lcdPrintDelay > LCD_PRINT_DELAY / 3)
        {
            lcd.setCursor(0, 0);
            lcd.print("RH Alarm Set: ");

            lcd.setCursor(5, 2);
            lcd.print("Min = ");
            lcd.print(rotaryVal1, 1);
            lcd.print("%");

            lcd.setCursor(5, 3);
            lcd.print("Max = ");
            lcd.print(rotaryVal2, 1);
            lcd.print("%");

            lcdPrintDelay = millis();
        }
    }
    if (pCount > 0)
    {
        lcd.setCursor(0, 0);
        lcd.print("RH Alarm Values ");
        lcd.setCursor(0, 1);
        lcd.print("Chaneged to: ");

        lcd.setCursor(5, 2);
        lcd.print("Min = ");
        lcd.print(rotaryVal1, 1);
        lcd.print("%");

        lcd.setCursor(5, 3);
        lcd.print("Max = ");
        lcd.print(rotaryVal2, 1);
        lcd.print("%");

        rhAlarmMin = rotaryVal1;
        rhAlarmMax = rotaryVal2;
        write2Eeprom(RH_ALARM_MIN_ADDR, rotaryVal1);
        write2Eeprom(RH_ALARM_MAX_ADDR, rotaryVal2);

        delay(LCD_MSG_DELAY);
        lcd.clear();
    }

    if (pCount < 0)
    {
        lcd.clear();
        lcd.setCursor(4, 1);
        lcd.print("Cancelled...");

        delay(LCD_MSG_DELAY);
        lcd.clear();
    }
}

//------------Calibration Functions----------------------

void tempCal(boolean inVal) // temperature calibration
{
    if (inVal)
    {
        rotE1.pushCounterReset();
        rotE2.pushCounterReset();
        lcd.clear();

        int8_t pCountMinus = 0;
        int8_t pCountPlus = 0;
        int8_t pCount = 0;

        float rotaryVal1;
        float rotaryVal2;

        while (pCount >= 0 && pCount < 1)
        {
            pCountPlus = rotE1.pushCounter();
            pCountMinus = rotE2.pushCounter();
            pCount = pCountPlus - pCountMinus;

            rotaryVal1 = 25.00 - (float)rotE1.rotVal() / 10;
            rotaryVal2 = 50.00 - (float)rotE2.rotVal() / 10;

            if (millis() - lcdPrintDelay > LCD_PRINT_DELAY / 3)
            {
                lcd.setCursor(0, 0);
                lcd.print("Temperature ");
                lcd.setCursor(0, 1);
                lcd.print("Calibration:  ");

                lcd.setCursor(3, 2);
                lcd.print(rotaryVal1, 1);
                lcd.print(char(0xDF));
                lcd.print("C => 25");
                lcd.print(char(0xDF));
                lcd.print("C ");

                lcd.setCursor(3, 3);
                lcd.print(rotaryVal2, 1);
                lcd.print(char(0xDF));
                lcd.print("C => 50");
                lcd.print(char(0xDF));
                lcd.print("C ");

                lcdPrintDelay = millis();
            }
        }
        if (pCount > 0)
        {
            lcd.setCursor(0, 0);
            lcd.print("Temperature Cal.");
            lcd.setCursor(0, 1);
            lcd.print("Completed    ");

            lcd.setCursor(3, 2);
            lcd.print(rotaryVal1, 1);
            lcd.print(char(0xDF));
            lcd.print("C => 25");
            lcd.print(char(0xDF));
            lcd.print("C ");

            lcd.setCursor(3, 3);
            lcd.print(rotaryVal2, 1);
            lcd.print(char(0xDF));
            lcd.print("C => 50");
            lcd.print(char(0xDF));
            lcd.print("C ");

            tempCalMin = rotaryVal1;
            tempCalMax = rotaryVal2;
            write2Eeprom(TEMP_CAL_MIN_ADDR, rotaryVal1);
            write2Eeprom(TEMP_CAL_MAX_ADDR, rotaryVal2);

            tempCalValue = (float)(rotaryVal1 + rotaryVal2 - 75.00) / 2.00;

            delay(LCD_MSG_DELAY);
            lcd.clear();
        }

        if (pCount < 0)
        {
            lcd.clear();
            lcd.setCursor(4, 1);
            lcd.print("Cancelled...");

            delay(LCD_MSG_DELAY);
            lcd.clear();
        }
    }
}

void rhCal(boolean inVal) // rh calibration
{
    if (inVal)
    {
        rotE1.pushCounterReset();
        rotE2.pushCounterReset();
        lcd.clear();

        int8_t pCountMinus = 0;
        int8_t pCountPlus = 0;
        int8_t pCount = 0;

        float rotaryVal1;
        float rotaryVal2;

        while (pCount >= 0 && pCount < 1)
        {
            pCountPlus = rotE1.pushCounter();
            pCountMinus = rotE2.pushCounter();
            pCount = pCountPlus - pCountMinus;

            rotaryVal2 = 80.00 - (float)rotE1.rotVal() / 10.00;
            rotaryVal1 = 35.00 - (float)rotE2.rotVal() / 10.00;

            if (millis() - lcdPrintDelay > LCD_PRINT_DELAY / 3)
            {
                lcd.setCursor(0, 0);
                lcd.print("RH Calibration: ");

                lcd.setCursor(4, 2);
                lcd.print(rotaryVal2, 1);
                lcd.print("% => 80%");

                lcd.setCursor(4, 3);
                lcd.print(rotaryVal1, 1);
                lcd.print("% => 35%");

                lcdPrintDelay = millis();
            }
        }
        if (pCount > 0)
        {
            lcd.setCursor(0, 0);
            lcd.print("RH Calibration ");
            lcd.setCursor(0, 1);
            lcd.print("Completed");

            lcd.setCursor(4, 2);
            lcd.print(rotaryVal2, 1);
            lcd.print("% => 80%");

            lcd.setCursor(4, 3);
            lcd.print(rotaryVal1, 1);
            lcd.print("% => 35%");

            rhCalMin = rotaryVal1;
            rhCalMax = rotaryVal2;
            write2Eeprom(RH_CAL_MIN_ADDR, rotaryVal1);
            write2Eeprom(RH_CAL_MAX_ADDR, rotaryVal2);

            rhCalValue = (float)(rotaryVal2 + rotaryVal1 - 115.00) / 2.00;

            delay(LCD_MSG_DELAY);
            lcd.clear();
        }

        if (pCount < 0)
        {
            lcd.clear();
            lcd.setCursor(4, 1);
            lcd.print("Cancelled...");

            delay(LCD_MSG_DELAY);
            lcd.clear();
        }
    }
}

boolean accessToMenu(byte *addrOfInVal, String inStr) //password prompt screen before the menu
{
    analogWrite(buzzerPin, 0);
    rotE1.pushCounterReset();
    rotE2.pushCounterReset();
    lcd.clear();
    lcd.setCursor(0, 0);
    lcd.print("Please Enter the");
    lcd.setCursor(0, 1);
    lcd.print(inStr);
    lcd.print("Code:");

    int8_t pCountMinus = 0;
    int8_t pCountPlus = 0;
    int8_t pCount = 0;

    int8_t pCode[5];

    while (pCount >= 0 && pCount < 4)
    {
        pCountPlus = rotE1.pushCounter();
        pCountMinus = rotE2.pushCounter();
        pCount = pCountPlus - pCountMinus;
        if (pCount == 0)
        {
            pCode[pCount] = rotE1.rotValBetween() / 2;
        }
        else if (pCount == 1)
        {
            pCode[pCount] = rotE1.rotValBetween() / 2;
        }
        else if (pCount == 2)
        {
            pCode[pCount] = rotE1.rotValBetween() / 2;
        }
        else if (pCount == 3)
        {
            pCode[pCount] = rotE1.rotValBetween() / 2;
        }

        if (millis() - lcdPrintDelay > LCD_PRINT_DELAY / 2)
        {
            if (pCount == 0)
            {

                lcd.setCursor(6, 2);
                lcdBlinkChar(pCode[pCount]);

                lcd.print(" * * *");
            }
            else if (pCount == 1)
            {
                lcd.setCursor(6, 2);

                lcd.print("* ");
                lcdBlinkChar(pCode[pCount]);

                lcd.print(" * *");
            }
            else if (pCount == 2)
            {
                lcd.setCursor(6, 2);

                lcd.print("* * ");
                lcdBlinkChar(pCode[pCount]);

                lcd.print(" *");
            }
            else if (pCount == 3)
            {
                lcd.setCursor(6, 2);

                lcd.print("* * * ");
                lcdBlinkChar(pCode[pCount]);
            }
            lcdPrintDelay = millis();
        }
    }

    if (pCount == 4)
    {
        lcd.setCursor(6, 2);
        lcd.print("* * * *");

        delay(LCD_MSG_DELAY / 2);
        lcd.clear();
    }


    if (pCode[0] == *addrOfInVal && pCode[1] == *(addrOfInVal + 1) && pCode[2] == *(addrOfInVal + 2) && pCode[3] == *(addrOfInVal + 3))
    {
        lcd.clear();
        lcd.setCursor(2, 2);
        lcd.print("Access Granted..");
        delay(LCD_MSG_DELAY);
        return 1;
    }
    else if (pCount < 0)
    {
        lcd.clear();
        lcd.setCursor(4, 1);
        lcd.print("Cancelled..");
        delay(LCD_MSG_DELAY);
    }
    else
    {
        lcd.clear();
        lcd.setCursor(0, 2);
        lcd.print("  Access Denied...!");
        delay(LCD_MSG_DELAY);
    }

    rotE1.pushCounterReset();
    rotE2.pushCounterReset();
    lcd.clear();
    return 0;
}

void write2Eeprom(int16_t eepromAddr, float inVal) // write float to eeprom
{
    inVal *= 100;
    int16_t outVal = (int16_t)inVal;
   
    EEPROM.write(eepromAddr, outVal >> 8);
    EEPROM.write(eepromAddr + 1, outVal); 
}

float readFromEeprom(int16_t eepromAddr) // read float from eeprom
{
    int16_t inVal = EEPROM.read(eepromAddr) << 8 | EEPROM.read(eepromAddr + 1);

    float outVal = (float)((inVal + .49) / 100);
    return outVal;
}
//==================================================================

float tempReadVal() // read temperature through ads1115
{ 
    int16_t senseIn = ads.readADC_SingleEnded(0);
    senseIn = constrain(senseIn, SENSE_RAW_MIN, SENSE_RAW_MAX);
    senseIn = map(senseIn, SENSE_RAW_MIN, SENSE_RAW_MAX, -3000, 7000);
    float senseOut = tempCalValue + senseIn / 100.00;
    return senseOut;
}                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                          

float rhReadVal() // read rh through ads1115
{
    int16_t senseIn = ads.readADC_SingleEnded(1);
    senseIn = constrain(senseIn, SENSE_RAW_MIN, SENSE_RAW_MAX);
    senseIn = map(senseIn, SENSE_RAW_MIN, SENSE_RAW_MAX, 0, 10000);
    float senseOut = rhCalValue + senseIn / 100.00;
    return senseOut;
}

boolean alarmTrig(String inStr, boolean high_low_val, float inVal, float inSetVal) // trigger alarm
{
    boolean outVal = 0;
    if (!high_low_val && inVal < inSetVal)
    {
        lcd.setCursor(0, 3);
        lcd.print("LOW ");
        lcd.print(inStr);
        outVal = 1;
        buzzerCtrl();
    }
    else if (high_low_val && inVal > inSetVal)
    {
        lcd.setCursor(0, 3);
        lcd.print("HIGH ");
        lcd.print(inStr);
        outVal = 1;
        buzzerCtrl();
    } 
    return outVal;
}

void buzzerCtrl() // buzzer pattern
{
    if (millis() - buzzerDelay > buzzerDly)
    {
        buzzerDelay = millis();
        
        if (buzzerDly == 500)
        { 
            buzzerDly = 100;
            analogWrite(buzzerPin, 160);
        }
        else 
        { 
            buzzerDly = 500; 
            analogWrite(buzzerPin, 0);
        }
    }
}

void alarmCtrl()// trigger alarm below or exceed any set values
{
    if (alarmTrig("temperature", 0, tempReadVal(), tempAlarmMin)) {}
    else if (alarmTrig("temperature", 1, tempReadVal(), tempAlarmMax)) {}
    else if (alarmTrig("RH ", 0, rhReadVal(), rhAlarmMin)) {}
    else if (alarmTrig("RH ", 1, rhReadVal(), rhAlarmMax)) {}
    else { analogWrite(buzzerPin, 0); }
}

void lcdClr(uint16_t inVal) // lcd clear every defined time
{
    if (millis() - lcdClearDelay > inVal)
    {
        lcd.clear();
        lcdClearDelay = millis();
    }
}

void transferData(float tempIn, float rhIn) //transfer requested values through Lora  
{
    int packetSize = LoRa.parsePacket();
    if (packetSize)
    {
        while (LoRa.available())
        {
            reqIn += (char)LoRa.read();
        }
        if (reqIn == sensorAddr) // if requested address matches sensor address send data
        {
            Serial.println("requested");

            LoRa.beginPacket();
            LoRa.print(tempIn);
            LoRa.endPacket();

            LoRa.beginPacket();
            LoRa.print(rhIn);
            LoRa.endPacket();
        }
        reqIn = "";
    }
}