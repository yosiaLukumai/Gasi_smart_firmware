#define TINY_GSM_MODEM_SIM800
#define SerialMon Serial
#define SerialAT Serial1
#define TINY_GSM_DEBUG SerialMon
#include <Arduino.h>
#include <EEPROM.h>
#include <Wire.h>
#include <SPIFFS.h>
#include <TinyGsmClient.h>
#include <WiFi.h>
#include <ArduinoJson.h>
#include <PubSubClient.h>
#include <LiquidCrystal_I2C.h>
#include "HX711.h"
#include <freertos/FreeRTOS.h>
#include <esp_sleep.h>
#include <esp_bt.h>
#include <esp_wifi.h>
#define DTR_GSM 19
#define RX_PIN 21
#define TX_PIN 22

#define EEPROM_TARE_ADDRESS 0
#define CylinderTypeAdress 30
#define uS_TO_S_FACTOR 1000000ULL // Conversion factor for micro seconds to seconds
#define TIME_TO_SLEEP 3600          // Time ESP32 / 60 (minutes)

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
int WorkingCylindeType = -1;

struct Cylinder
{
  float tare;
  float net;
  String name;
};

// Example cylinder database (You can add more cylinders with their specifications)
Cylinder cylinders[] = {
    {8.6, 6.0, "14.6kg Cylinder"},  // Tare 8.6kg, Net 6.0kg (Total 14.6kg)
    {0, 12, "12kg TestContainer"},  // Tare 0kg, Net 12.0kg (Total 12kg)
    {14, 15, "29kg Cylinder"},      // Tare 14kg, Net 15kg (Total 29kg)
    {15.5, 15, "30.5kg Cylinder"}}; // Tare 15.5kg, Net 15kg (Total 30.5kg)

int selectedOption = 0;
float level_percentage;
float lpg_weight;

// Load cell pins
const int LOADCELL_DOUT_PIN = 15; // GPIO4 on ESP32
const int LOADCELL_SCK_PIN = 5;   // GPIO5 on ESP32

struct ButtonPressedConfig
{
  bool ButtonPressed;
  bool TimeOut;
  bool SettingMode;
};

ButtonPressedConfig myButton;

void IRAM_ATTR attachInterruptSetup()
{
  if (!myButton.ButtonPressed)
  {
    myButton.TimeOut = false;
    myButton.ButtonPressed = true;
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

void saveDataToEEPROM(int address, long tareValue)
{
  EEPROM.put(address, tareValue);
  EEPROM.commit(); // Save changes
}

void performTare()
{
  SerialMon.println("Performing tare...");
  scale.tare();
  tareValue = scale.get_offset();
  saveDataToEEPROM(EEPROM_TARE_ADDRESS, tareValue);
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

void printWakeUpReason()
{
  esp_sleep_wakeup_cause_t wakeup_reason;
  wakeup_reason = esp_sleep_get_wakeup_cause();

  switch (wakeup_reason)
  {
  case ESP_SLEEP_WAKEUP_EXT0:
    Serial.println("Wakeup caused by external signal using RTC_IO (button press)");
    break;
  case ESP_SLEEP_WAKEUP_TIMER:
    Serial.println("Wakeup caused by timer");
    break;
  default:
    Serial.println("Wakeup was not caused by deep sleep");
    break;
  }
}

void UpdateScreen(float level, float weight)
{
  PrintToScreen(0, 0, " UZITO: " + String(weight) + " kg");
  PrintToScreen(0, 1, " KWNGO: " + String(level) + " %");
}

void PrintSelectGas(int Selectd)
{
  PrintToScreen(0, 0, "    =GAS TYPE= ");
  PrintToScreen(0, 1, cylinders[Selectd].name);
}

void ComputeAndShow(bool display)
{

  int iteration = 0;
  float sum = 0;
  clearScreen();
  PrintToScreen(0, 0, " =COMPUTING=");

  // Take 10 readings for averaging
  while (iteration < 10)
  {
    String progres = "   .";
    if (scale.is_ready())
    {
      float measured_weight = scale.get_units(10) - 2.93;
      sum += measured_weight;
      iteration++;
      progres = progres + ".";
      PrintToScreen(0, 1, progres);
      delay(100); // Add a small delay to stabilize readings
    }
  }

  float measured_avg_weight = sum / 10.0;

  SerialMon.print("wE: ");
  SerialMon.println(measured_avg_weight);

  // Apply tare weight to calculate net weight
  float lpg_weight = measured_avg_weight - cylinders[WorkingCylindeType].tare;

  // Ensure net weight is not negative (for empty cylinders)
  if (lpg_weight < 0)
    lpg_weight = 0;

  // Calculate the gas level as a percentage
  level_percentage = (lpg_weight / cylinders[WorkingCylindeType].net) * 100;

  // Limit level_percentage between 0 and 100 for realistic values

  if (level_percentage > 100)
    level_percentage = 100;
  if (level_percentage < 0)
    level_percentage = 0;

  // Clear and update the screen with new gas level and weight
  if (display)
  {
    clearScreen();
    UpdateScreen(level_percentage, lpg_weight);
  }
}

void ButtonSettingFunc()
{

  int selected;
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
    bool ConfirmButton;
  };

  PressedSetting buttonSetting = {false};
  clearScreen();
  for (;;)
  {
    vTaskDelay(1);
    if (millis() - lastMsg > 60000)
    { // Timeout after 60 seconds
      myButton.TimeOut = true;
      break;
    }

    selected = map(analogRead(Pot), 0, 4095, 0, sizeof(cylinders)/sizeof(Cylinder)); // Cylinder selection
    PrintSelectGas(selected);

    if (!digitalRead(CancelButton))
    { // Confirmation button pressed
      buttonSetting.ConfirmButton = true;
      break;
    }
  }

  // Handle timeout scenario
  if (myButton.TimeOut)
  {
    clearScreen();
    PrintToScreen(0, 0, "    TIMEOUT  ");
    PrintToScreen(0, 1, "  PLEASE RETRY ");
    myButton.ButtonPressed = false;
    delay(1000);
    clearScreen();
    return;
  }

  // Confirm new configuration
  if (buttonSetting.ConfirmButton)
  {
    saveDataToEEPROM(CylinderTypeAdress, selected); // Save cylinder type to EEPROM
    WorkingCylindeType = selected;
    clearScreen();
    PrintToScreen(0, 0, " NEW SETTING");
    PrintToScreen(0, 1, " ---SAVED--");
    delay(1500);
    clearScreen();
    myButton.ButtonPressed = false;
    return;
  }

  clearScreen();
  myButton.ButtonPressed = false;
}

void setup()
{
  WiFi.disconnect(true);
  WiFi.mode(WIFI_OFF);
  esp_wifi_stop();
  btStop();
  SerialMon.begin(115200);
  EEPROM.begin(512);

  // pinMode(Buzzer, OUTPUT);
  pinMode(CancelButton, INPUT);
  pinMode(ConfirmButton, INPUT);
  delay(10);
  SerialAT.begin(9600, SERIAL_8N1, RX_PIN, TX_PIN);

  printWakeUpReason();
  Wire.begin(32, 13);
  lcd.init();
  lcd.noBacklight();
  pinMode(DTR_GSM, OUTPUT);
  modem.sendAT("+CSCLK=1");

  delay(1000);
  digitalWrite(DTR_GSM, LOW);

  if (modem.sleepEnable(true))
  {
    SerialMon.println("Modem went to sleep");
  }
  else
  {
    SerialMon.println("Failed to sleep Gsm.");
  }

  if (esp_sleep_get_wakeup_cause() == ESP_SLEEP_WAKEUP_TIMER)
  {
    // send data to server
    WorkingCylindeType = (int)readFromEEPROM(CylinderTypeAdress);
    delay(500);
    if (WorkingCylindeType >= 0)
    {
      if (modem.sleepEnable(false))
      {
        SerialMon.println("Testing ");
      }
      SerialMon.println(" I am sending data after wakeup from the timer output");
      delay(6000);
      String modemInfo = modem.getModemInfo();
      SerialMon.print("Modem Info: ");
      SerialMon.print("Connecting to APN: ");
      SerialMon.print(apn);
      PrintToScreen(0, 0, " Connecting to ");
      PrintToScreen(0, 1, "====INTERNET====");
      if (!modem.gprsConnect(apn, gprsUser, gprsPass))
      {
        SerialMon.println(" fail");
        PrintToScreen(0, 0, "Failed to Connect");
        PrintToScreen(0, 1, "RESTARTING.......");
        delay(1500);
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

      mqtt.setServer(broker, 1883);
      mqtt.setCallback(mqttCallback);
      scale.begin(LOADCELL_DOUT_PIN, LOADCELL_SCK_PIN);

      while (true)
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
        }
        if (mqtt.connected())
        {
          break;
        }
      }

      // try retriving the tare weight from eeprom
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
      myButton = {false, false, false};
      // load the setting
      delay(1000);
      ComputeAndShow(false);
      DynamicJsonDocument doc(512); // Define a larger JsonDocument
      char buffer[512];             // Increased buffer size

      doc["serialNumber"] = serialNumber; // Add your variables
      doc["levelPercentage"] = level_percentage;
      doc["lpg_weight"] = lpg_weight;

      serializeJson(doc, buffer); // Serializing JSON data to buffer

      // Print the serialized JSON to Serial Monitor for verification
      SerialMon.print("Serialized Buffer: ");
      SerialMon.println(buffer);

      // Check MQTT connection before publishing
      if (mqtt.connected())
      {
        // Try publishing the message and check if it succeeds
        bool success = mqtt.publish(topicNotification, buffer);
        if (success)
        {
          SerialMon.println("MQTT Publish Successful");
        }
        else
        {
          SerialMon.println("MQTT Publish Failed");
        }
      }
      else
      {
        SerialMon.println("MQTT not connected");
      }
      mqtt.loop(); // Keep the MQTT connection alive
      delay(3000);
      modem.sleepEnable(true);
       modem.sendAT("+CSCLK=1");

  delay(1000);
  digitalWrite(DTR_GSM, LOW);
  delay(1000);

    }
  }

  if (esp_sleep_get_wakeup_cause() == ESP_SLEEP_WAKEUP_EXT0)
  {
    lcd.backlight();
    HomeScreen();
    bool SettingMode = false;
    long lastTime = millis();
    for (;;)
    {
      if (millis() - lastTime >= 3000)
      {
        break;
      }
      if (!digitalRead(CancelButton))
      {
        SettingMode = true;
        break;
      }
    }
    WorkingCylindeType = (int)readFromEEPROM(CylinderTypeAdress);
    delay(500);
    SerialMon.print("Mode:  ");
    SerialMon.println(SettingMode);
    if (!SettingMode)
    {
      if (WorkingCylindeType)
      {
        scale.begin(LOADCELL_DOUT_PIN, LOADCELL_SCK_PIN);

        // try retriving the tare weight from eeprom
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
        myButton = {false, false, false};
        delay(1000);

        SerialMon.print("Working Value: ");
        SerialMon.println(WorkingCylindeType);
        ComputeAndShow(true);
        delay(20000); //
      }
      else
      {
        PrintToScreen(0, 0, " No cylinder");
        PrintToScreen(0, 1, " Configured..");
        delay(1500);
      }
    }
    else
    {
      ButtonSettingFunc();
    }
  }
  esp_sleep_enable_ext0_wakeup(GPIO_NUM_35, 0);
  esp_sleep_enable_timer_wakeup(TIME_TO_SLEEP * uS_TO_S_FACTOR);
  lcd.noBacklight();
  lcd.noDisplay();

  // Go to deep sleep
  Serial.println("Going to sleep now...");
  delay(1000);
  esp_deep_sleep_start(); // Enter deep sleep
  // register handlers for
}

void loop()
{
}
