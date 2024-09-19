#define TINY_GSM_MODEM_SIM800
#define SerialMon Serial
#define SerialAT Serial1
#define TINY_GSM_DEBUG SerialMon
#include <Arduino.h>
#include <EEPROM.h>
#include <Wire.h>
#include <SPIFFS.h>
#include <TinyGsmClient.h>
#include <ArduinoJson.h>
#include <PubSubClient.h>
#include <LiquidCrystal_I2C.h>
#include "HX711.h"
#include <freertos/FreeRTOS.h>
#define DTR_GSM 19
#define RX_PIN 21
#define TX_PIN 22

#define EEPROM_TARE_ADDRESS 0
#define GrossWeight_adreess 12
#define Gas_Type_adress 2
#define DecimalPoint_Address 20

long tareValue = 0;

const char apn[] = "internet";
const char gprsUser[] = "";
const char gprsPass[] = "";
uint8_t updateRate = 0;

const char simPIN[] = "";

const char *broker = "45.79.53.206";

// const char *broker = "test.mosquitto.org";
const char *serialNumber = "s2AhayZQwier";
const char *mqttUsername = "yosia";
const char *mqttPassword = "lukumai";

const char *pathconfNotification = "device/config";
const char *pathSettingChanged = "device/setting/changed";
const char *topicNotification = "log/parameter";
const char *topicCritical = "notification/critical";
constexpr uint8_t Buzzer = 4;

constexpr uint8_t Pot = 34;
constexpr uint8_t ConfirmButton = 35;
constexpr uint8_t CancelButton = 10;
double options[] = {3, 6, 12, 12.5, 15, 38}; // Available weight options
int selectedOption = 0;
float level_percentage;

// Load cell pins
const int LOADCELL_DOUT_PIN = 15; // GPIO4 on ESP32
const int LOADCELL_SCK_PIN = 5;   // GPIO5 on ESP32

struct ButtonPressedConfig
{
  bool ButtonPressed;
  bool TimeOut;
};

byte upArrow[8] = {
    0b00100,
    0b01110,
    0b11111,
    0b00100,
    0b00100,
    0b00100,
    0b00000,
    0b00000};

byte downArrow[8] = {
    0b00000,
    0b00000,
    0b00100,
    0b00100,
    0b00100,
    0b11111,
    0b01110,
    0b00100};

ButtonPressedConfig myButton;

void IRAM_ATTR attachInterruptSetup()
{
  if (!myButton.ButtonPressed)
  {
    myButton.ButtonPressed = true;
    myButton.TimeOut = false;
    // SerialMon.println("Pressed....");
  }
}

TaskHandle_t handleSensorData = NULL;

HX711 scale;

float calibration_factor = -(312885 / 8.6);   // Adjust based on your load cell calibration
float tare_weight = 8.4;                      // Tare weight of the empty cylinder in kg
float net_weights = 6.0;                      // Net weight of the gas when full in kg
float gross_weight = 0;                       // Gross weight (tare + net) in kg
float lpg_density = 0.493;                    // Density of LPG in kg/L
float max_volume = net_weights / lpg_density; // Maximum volume of LPG cylinder in liters

// Function prototype for setting tare weight

uint32_t lastReconnectAttempt = 0;
float temperature = 0;
float humidity = 0;
long lastMsg = 0;

#ifdef DUMP_AT_COMMANDS
#include <StreamDebugger.h>
StreamDebugger debugger(SerialAT, SerialMon);
TinyGsm modem(debugger);
#else
TinyGsm modem(SerialAT);
#endif

TinyGsmClient client(modem);
PubSubClient mqtt(client);
LiquidCrystal_I2C lcd(0x27, 16, 2); // Change 0x27 to your I2C address if different

void mqttCallback(char *topic, byte *message, unsigned int len)
{
  SerialMon.print("Message arrived on topic: ");
  SerialMon.print(topic);
  SerialMon.print(". Message: ");
  String messageTemp;

  for (int i = 0; i < len; i++)
  {
    SerialMon.print((char)message[i]);
    messageTemp += (char)message[i];
  }
  SerialMon.println();

  // Feel free to add more if statements to control more GPIOs with MQTT

  // If a message is received on the topic esp/output1, you check if the message is either "true" or "false".
  // Changes the output state according to the message
  if (String(topic) == "esp/output1")
  {
    SerialMon.print("Changing output to ");
    if (messageTemp == "true")
    {
      SerialMon.println("true");
    }
    else if (messageTemp == "false")
    {
      SerialMon.println("false");
    }
  }
}

void saveTareToEEPROM(int address, long tareValue)
{
  EEPROM.put(address, tareValue);
  EEPROM.commit(); // Save changes
}

void performTare()
{
  SerialMon.println("Performing tare...");
  scale.tare();
  tareValue = scale.get_offset();
  saveTareToEEPROM(EEPROM_TARE_ADDRESS, tareValue);
  SerialMon.println("Tare complete and saved to EEPROM");
}

long readFromEEPROM(int address)
{
  long storedTare = 0;
  EEPROM.get(address, storedTare);
  return storedTare;
}

boolean mqttConnect()
{
  SerialMon.print("Connecting to ");
  SerialMon.print(broker);

  boolean status = mqtt.connect(serialNumber);

  if (status == false)
  {
    SerialMon.println(" fail");
    ESP.restart();
    return false;
  }
  SerialMon.println(" success");
  mqtt.subscribe(pathconfNotification);

  return mqtt.connected();
}

void PrintToScreen(uint8_t column, uint8_t row, String details)
{
  lcd.setCursor(column, row);
  lcd.print(details);
}

void HomeScreen()
{
  PrintToScreen(0, 0, "   GESI SMART   ");
  for (int x = 0; x < 16; x++)
  {
    PrintToScreen(x, 1, "=");
    delay(200);
  }
}

void clearScreen()
{
  PrintToScreen(0, 0, "                ");
  PrintToScreen(0, 1, "                ");
}

void alertUser()
{
  digitalWrite(Buzzer, HIGH);
  delay(2000);
  digitalWrite(Buzzer, LOW);
  delay(2000);
}

void setup()
{
  SerialMon.begin(115200);
  pinMode(Buzzer, OUTPUT);
  pinMode(CancelButton, INPUT);
  pinMode(ConfirmButton, INPUT);
  delay(10);
  Wire.begin(32, 13);
  lcd.init();
  lcd.backlight();

  SerialAT.begin(9600, SERIAL_8N1, RX_PIN, TX_PIN);

  HomeScreen();
  delay(6000);

  String modemInfo = modem.getModemInfo();
  SerialMon.print("Modem Info: ");
  SerialMon.println(modemInfo);

  SerialMon.print("Connecting to APN: ");
  SerialMon.print(apn);
  if (!modem.gprsConnect(apn, gprsUser, gprsPass))
  {
    SerialMon.println(" fail");
    ESP.restart();
  }
  else
  {
    SerialMon.println(" OK");
  }

  if (modem.isGprsConnected())
  {
    SerialMon.println("GPRS connected");
  }

  attachInterrupt(ConfirmButton, attachInterruptSetup, RISING);
  mqtt.setServer(broker, 1883);
  mqtt.setCallback(mqttCallback);
  scale.begin(LOADCELL_DOUT_PIN, LOADCELL_SCK_PIN);

  EEPROM.begin(512);
  // try retriving the tear weight from eeprom
  tareValue = readFromEEPROM(EEPROM_TARE_ADDRESS);
  delay(1500);
  SerialMon.print("Tare Value:  ");
  SerialMon.println(tareValue);
  if (tareValue == 0)
  {
    performTare();
  }
  else
  {
    scale.set_offset(tareValue); // Set the stored tare value
    SerialMon.println("Tare value loaded from EEPROM");
  }

  scale.set_scale(calibration_factor); // Set the calibration factor
  myButton = {false, false};
  // load the setting
  gross_weight = readFromEEPROM(GrossWeight_adreess);
  delay(1000);
  net_weights = options[readFromEEPROM(Gas_Type_adress)];
  delay(1000);

  float DecimalPoints = (readFromEEPROM(DecimalPoint_Address) / 100.0);
  delay(500);
  gross_weight = gross_weight + DecimalPoints;
  SerialMon.println("Setting:  Gross: " + String(gross_weight) + "  " + " Gas Type: " + String(net_weights));
}

void UpdateScreen(float level, float weight)
{
  PrintToScreen(0, 0, " UZITO: " + String(weight) + " kg");

  PrintToScreen(0, 1, " KWNGO: " + String(level) + " %");
}

void checkTimeOut()
{
}

String returnPad(int value)
{
  return "   " + String(options[value]) + "    Kg";
}

void PrintSelectGas(int Selectd)
{
  PrintToScreen(0, 0, " =AINA YA GAS= ");
  PrintToScreen(0, 1, returnPad(Selectd));
}

void loop()
{

  if (!mqtt.connected())
  {
    SerialMon.println("=== MQTT NOT CONNECTED ===");
    uint32_t t = millis();
    if (t - lastReconnectAttempt > 10000L)
    {
      lastReconnectAttempt = t;
      if (mqttConnect())
      {
        lastReconnectAttempt = 0;
      }
    }
    delay(100);
    return;
  }

  if (scale.is_ready())
  {
    float weight = scale.get_units(10) - 3.6; // Average over 10 readings for stability

    // Apply tare weight to calculate net weight
    float net_weight = weight - (gross_weight - net_weights);
    // Ensure net weight is not negative (for empty cylinders)
    if (net_weight < 0)
      net_weight = 0;

    // Calculate the gas level as a percentage
    level_percentage = (net_weight / gross_weight) * 100;

    // Limit level_percentage between 0 and 100 for realistic values
    if (level_percentage >= 100)
      level_percentage = 100;
    if (level_percentage <= 0)
      level_percentage = 0;

    // SerialMon.println(level_percentage);
    // SerialMon.print("Weight: ");
    // SerialMon.print(weight);
    // SerialMon.println(" kg");
    if (updateRate == 0 || updateRate > 10)
    {
      clearScreen();
      UpdateScreen(level_percentage, weight < 0 ? 0 : weight);
      updateRate = 1;
    }
    updateRate++;
  }
  else
  {
    SerialMon.println("HX711 not found.");
    delay(1500);
    return;
  }

  long now = millis(); // Current time in milliseconds
  if (now - lastMsg > 50000)
  {
    JsonDocument doc;
    lastMsg = now;
    char buffer[200];
    doc["serialNumber"] = serialNumber;
    doc["levelPercentage"] = level_percentage;
    doc["grossWeight"] = gross_weight;
    serializeJson(doc, buffer);
    // Temperature in Celsius
    SerialMon.print("MyBuffer: ");
    SerialMon.println(buffer);
    mqtt.publish(topicNotification, buffer);
  }

  if (myButton.ButtonPressed)
  {
    int selected;
    // setting mode
    clearScreen();
    PrintToScreen(0, 0, "  SETTING MODE ");
    delay(2000);
    clearScreen();
    lcd.setCursor(0, 0);
    lcd.write(byte(0));
    lcd.setCursor(2, 0);
    lcd.print(" New Config");
    lcd.setCursor(0, 1);
    lcd.write(byte(1));
    lcd.setCursor(2, 1);
    lcd.print(" View Config");
    long lastMsg = millis(); // Current time in milliseconds

    struct PressedSetting
    {
      bool CancelButton;
      bool ConfirmButton;
    };
    PressedSetting buttonSetting;
    buttonSetting = {false, false};
    clearScreen();
    for (;;)
    {
      vTaskDelay(1);
      if (millis() - lastMsg > 10000)
      {
        myButton.TimeOut = true;
        break;
      }
      selected = map(analogRead(Pot), 0, 4095, 0, 5);
      PrintSelectGas(selected);
      if (!digitalRead(ConfirmButton))
      {
        // SerialMon.println("   ConfirmButton: ");
        buttonSetting.ConfirmButton = true;
        break;
      }

      if (!digitalRead(CancelButton))
      {
        // SerialMon.println("   Cancelling: ");
        buttonSetting.CancelButton = true;
        break;
      }
    }

    // checkTimeOut();
    if (myButton.TimeOut)
    {
      clearScreen();
      PrintToScreen(0, 0, "    TIMEOUT  ");
      PrintToScreen(0, 1, "  PLEASE RETRY ");
      // mqtt.loop();
      myButton.ButtonPressed = false;
      delay(1000);
      clearScreen();
      // SerialMon.println("....");
      return;
    }

    myButton.TimeOut = false;

    if (buttonSetting.CancelButton)
    {
      // SerialMon.println(" ============================");
      clearScreen();
      updateRate = 0;
      myButton.ButtonPressed = false;
      return;
      // View last config...
    }
    float weight = 0;

    if (buttonSetting.ConfirmButton)
    {
      buttonSetting.ConfirmButton = false;
      buttonSetting.CancelButton = false;
      // new config...
      // SerialMon.println("jj------------");
      long lastMsg = millis();
      clearScreen();
      PrintToScreen(0, 0, " PLACE CYLINDER ");
      PrintToScreen(0, 1, " Measure gross");
      delay(1000);
      clearScreen();

      for (;;)
      {
        vTaskDelay(1);
        if (millis() - lastMsg > 20000)
        {
          myButton.TimeOut = true;
          break;
        }
        if (scale.is_ready())
        {
          weight = scale.get_units(10) - 3.6; // Average over 10 readings for stability
          PrintToScreen(0, 0, " Gross weight  ");
          PrintToScreen(0, 1, "   " + String(weight) + "  Kg");
          // SerialMon.println(!digitalRead(ConfirmButton));
          // SerialMon.print(" Some mutex: ");
          // SerialMon.println(myButton.ButtonPressed);
          // SerialMon.println(!digitalRead(CancelButton));
          // SerialMon.print("Weight: ");
          // Serial.println(weight);
          if (!digitalRead(ConfirmButton))
          {
            // SerialMon.println("   ConfirmButton 2222222222222222222222222: ");
            buttonSetting.ConfirmButton = true;
            break;
          }

          if (!digitalRead(CancelButton))
          {
            // SerialMon.println("   Cancelling: 2222222222222222222222");
            buttonSetting.CancelButton = true;
            break;
          }
        }
        else
        {
          weight = 0;
        }
      }
      // SerialMon.println("_______________////////// out please_________________");
    }

    // SerialMon.print("check this: ");
    // SerialMon.println(buttonSetting.CancelButton);
    // SerialMon.print("check this 2: ");
    // SerialMon.println(buttonSetting.ConfirmButton);
    if (myButton.TimeOut)
    {
      clearScreen();

      if (weight == 0)
      {
        PrintToScreen(0, 0, "    TIMEOUT  ");
        PrintToScreen(0, 1, "  HAMNA MTUNGI ");
      }
      else
      {
        PrintToScreen(0, 0, "    TIMEOUT  ");
        PrintToScreen(0, 1, "  HAUJA HAKIKI");
      }
      // mqtt.loop();
      myButton.ButtonPressed = false;
      delay(1000);
      clearScreen();
      // SerialMon.println("....");
      return;
    }

    if (buttonSetting.ConfirmButton)
    {
      // SerialMon.println("Enterd///////////////");
      // we have to set this int eepromm
      if (weight > 3)
      {
        clearScreen();
        PrintToScreen(0, 0, " Saving Config");
        saveTareToEEPROM(GrossWeight_adreess, double(weight));
        delay(1500);
        saveTareToEEPROM(Gas_Type_adress, selected);
        delay(500);
        int wholeNumber = (int)weight;                  // Whole number part
        int decimalPart = (weight - wholeNumber) * 100; // Decimal part, multiplied by 100 to get 45
        // SerialMon.println(decimalPart);
        saveTareToEEPROM(DecimalPoint_Address, decimalPart);
        delay(500);
        // save & decimal
      }
      else
      {
        PrintToScreen(0, 0, " hakuna mtungi..");
        PrintToScreen(0, 1, "  ===========");
        delay(1000);
      }
    }
    if (buttonSetting.CancelButton)
    {
      clearScreen();
      PrintToScreen(0, 0, " Umesitisha  ");
      PrintToScreen(0, 1, "   Mchakato");
      delay(1000);
    }

    clearScreen();
    updateRate = 0;
    // SerialMon.println("___________________________________))))))___");
    myButton.ButtonPressed = false;
  }
  mqtt.loop();
}
