


// #include <Arduino.h>
// #include "HX711.h"
// #include "soc/rtc.h"
// #include <LiquidCrystal_I2C.h>

// // HX711 circuit wiring
// const int LOADCELL_DOUT_PIN = 16;
// const int LOADCELL_SCK_PIN = 4;

// HX711 scale;

// void setup() {
//   Serial.begin(9600);
//   rtc_cpu_freq_config_t config;
//   rtc_clk_cpu_freq_get_config(&config);
//   rtc_clk_cpu_freq_to_config(RTC_CPU_FREQ_80M, &config);
//   rtc_clk_cpu_freq_set_config_fast(&config);
//   Serial.println("HX711 Demo");

//   Serial.println("Initializing the scale");

//   scale.begin(LOADCELL_DOUT_PIN, LOADCELL_SCK_PIN);

//   Serial.println("Before setting up the scale:");
//   Serial.print("read: \t\t");
//   Serial.println(scale.read());      // print a raw reading from the ADC

//   Serial.print("read average: \t\t");
//   Serial.println(scale.read_average(20));   // print the average of 20 readings from the ADC

//   Serial.print("get value: \t\t");
//   Serial.println(scale.get_value(5));   // print the average of 5 readings from the ADC minus the tare weight (not set yet)

//   Serial.print("get units: \t\t");
//   Serial.println(scale.get_units(5), 1);  // print the average of 5 readings from the ADC minus tare weight (not set) divided
//             // by the SCALE parameter (not set yet)

// //   scale.set_scale(INSERT YOUR CALIBRATION FACTOR);
//   scale.set_scale(-528793/1);                      // this value is obtained by calibrating the scale with known weights; see the README for details
//   scale.tare();               // reset the scale to 0

//   Serial.println("After setting up the scale:");

//   Serial.print("read: \t\t");
//   Serial.println(scale.read());                 // print a raw reading from the ADC

//   Serial.print("read average: \t\t");
//   Serial.println(scale.read_average(20));       // print the average of 20 readings from the ADC

//   Serial.print("get value: \t\t");
//   Serial.println(scale.get_value(5));   // print the average of 5 readings from the ADC minus the tare weight, set with tare()

//   Serial.print("get units: \t\t");
//   Serial.println(scale.get_units(5), 1);        // print the average of 5 readings from the ADC minus tare weight, divided
//             // by the SCALE parameter set with set_scale

//   Serial.println("Readings:");
// }

// void loop() {
//   Serial.print("one reading:\t");
//   Serial.print(scale.get_units(), 1);
//   Serial.print("\t| average:\t");
//   Serial.println(scale.get_units(10), 5);

//   scale.power_down();             // put the ADC in sleep mode
//   delay(5000);
//   scale.power_up();
// }

// calibration

// #include <Arduino.h>
// // #include "HX711.h"
// // #include "soc/rtc.h"
// #include <LiquidCrystal_I2C.h>
// #include "HX711.h"

// // HX711 circuit wiring
// const int LOADCELL_DOUT_PIN = 16;
// const int LOADCELL_SCK_PIN = 4;

// HX711 scale;

// void setup() {
//   Serial.begin(9600);
//   scale.begin(LOADCELL_DOUT_PIN, LOADCELL_SCK_PIN);
// }

// void loop() {

//   if (scale.is_ready()) {
//     scale.set_scale();
//     Serial.println("Tare... remove any weights from the scale.");
//     delay(5000);
//     scale.tare();
//     Serial.println("Tare done...");
//     Serial.print("Place a known weight on the scale...");
//     delay(5000);
//     long reading = scale.get_units(10);
//     Serial.print("Result: ");
//     Serial.println(reading);
//   }
//   else {
//     Serial.println("HX711 not found.");
//   }
//   delay(1000);
// }


#include <Arduino.h>
#include <HX711.h>
#include <stdint.h>
#include <LiquidCrystal_I2C.h>
#include <SPIFFS.h>
#include <WiFi.h>
#include "soc/rtc.h"
#include <HTTPClient.h>
#include <ArduinoJson.h>

const char *ssid = "Happy";
const char *password = "shwaa100";
// 192.168.1.110
// 192.168.1.110
const char *serverName = "http://192.168.16.206:3400/data/collect";

LiquidCrystal_I2C lcd(0x27, 16, 2);

#define LOADCELL_DOUT_PIN 16
#define LOADCELL_SCK_PIN 4

#define I2C_SDA 21
#define I2C_SCL 22
#define Buzzer 26
#define TopButton 25
#define BottomButton 33
#define Potentiometer 32
#define LeftButton 19
#define SetupChangesButton 2
#define RightButton 18
bool toogler = false;
int32_t count = 0;
unsigned long previousMillis = 0;
const long interval = 10000;
HX711 scale;
int avoidMuchCounting = 0;

struct NewConfiguration
{
  bool UnderProcess;
};

struct GasConfiguration
{
  bool Configured;
  double GasAmount; // kg of gas in gramms
  double ReminderPercentage;
  double CylinderCapacity;
};
GasConfiguration MyLPG;

NewConfiguration ConfigStruct = {false};
// Variable definations

void PrintToScreen(uint8_t column, uint8_t row, String details)
{
  lcd.setCursor(column, row);
  lcd.print(details);
}

void HomeScreen()
{
  PrintToScreen(0, 0, "   GAS YANGU   ");
  for (int x = 0; x < 16; x++)
  {
    PrintToScreen(x, 1, "=");
    delay(200);
  }
}

void InitWiFi()
{
  WiFi.mode(WIFI_STA);
  WiFi.begin(ssid, password);
  Serial.print("Connecting to WiFi ..");
  while (WiFi.status() != WL_CONNECTED)
  {
    Serial.print('.');
    delay(1000);
  }
  Serial.println(WiFi.localIP());
}

void ClearRow(int row)
{
  lcd.setCursor(0, row);
  lcd.print("                ");
}

void IRAM_ATTR attachInterruptSetup()
{
  if (!ConfigStruct.UnderProcess)
  {
    ConfigStruct.UnderProcess = true;
  }
}
void ClearScreen()
{
  ClearRow(0);
  ClearRow(1);
}

void writeGasConfigToSPIFFS(GasConfiguration config)
{
  File file = SPIFFS.open("/gas_config.bin", FILE_WRITE);
  if (!file)
  {
    Serial.println("Failed to open file for writing");
    return;
  }

  // Write the structure as binary data
  file.write((uint8_t *)&config, sizeof(GasConfiguration));
  file.close();

  Serial.println("Gas configuration written to SPIFFS");
}

GasConfiguration readGasConfigFromSPIFFS()
{
  GasConfiguration config;

  File file = SPIFFS.open("/gas_config.bin", FILE_READ);
  if (!file)
  {
    Serial.println("Failed to open file for reading");
    return config; // Return an empty or default-initialized config
  }

  // Read the binary data into the structure
  file.read((uint8_t *)&config, sizeof(GasConfiguration));
  file.close();

  Serial.println("Gas configuration read from SPIFFS");

  return config;
}

void setup()
{

  // rtc_cpu_freq_config_t config;
  // rtc_clk_cpu_freq_get_config(&config);
  // rtc_clk_cpu_freq_to_config(RTC_CPU_FREQ_240M, &config);
  // rtc_clk_cpu_freq_set_config_fast(&config);
  lcd.init(I2C_SDA, I2C_SCL);
  lcd.backlight();
  HomeScreen();
  delay(1000);
  Serial.begin(9600);
  ClearScreen();
  PrintToScreen(0,0, "Trying connecting...");
  delay(500);
  InitWiFi();
  PrintToScreen(0, 1, "    CONNECTED ");
  ClearScreen();

  // if (!SPIFFS.begin(true))
  // {
  //   Serial.println("An Error has occurred while mounting SPIFFS");
  //   return;
  // }
  // else
  // {
  //   ClearScreen();
  //   PrintToScreen(0, 0, " MEMORY ERROR  ");
  //   PrintToScreen(0, 1, "  ===========");
  // }
  // GasConfiguration config = {false, 0.0, 50.0, 0.0};
  // writeGasConfigToSPIFFS(config);
  // MyLPG = readGasConfigFromSPIFFS();
  MyLPG = {true, 0.0, 50.0, 0.0};

  pinMode(TopButton, INPUT);
  pinMode(BottomButton, INPUT);
  pinMode(Buzzer, OUTPUT);
  pinMode(RightButton, INPUT);
  pinMode(LeftButton, INPUT);
  pinMode(Potentiometer, INPUT);
  pinMode(SetupChangesButton, INPUT_PULLUP);

  attachInterrupt(SetupChangesButton, attachInterruptSetup, FALLING);
  scale.begin(LOADCELL_DOUT_PIN, LOADCELL_SCK_PIN);
  scale.set_scale(-528793 / 1);
  Serial.print("Offese: ");
  Serial.println(MyLPG.GasAmount);
  scale.tare();
}

void DisplayGasAmount()
{
  String Printable = "GAS REM:" + String(MyLPG.GasAmount) + " Kg";
  String Printable2 = "% Usage: " + String(((MyLPG.GasAmount / MyLPG.CylinderCapacity) * 100)) + " %";
  PrintToScreen(0, 0, Printable);
  PrintToScreen(0, 1, Printable2);
}

void SendData(float weight, float ReminderValue, float GasSize)
{
  WiFiClient client;
  HTTPClient http;
  http.begin(client, serverName);
  StaticJsonDocument<200> doc;
  doc["deviceId"] = 1001;
  doc["weight"] = weight;
  doc["ReminderValue"] = ReminderValue;
  doc["GasSize"] = GasSize;
  String requestBody;
  serializeJson(doc, requestBody);
  http.addHeader("Content-Type", "application/json");
  int httpResponseCode = http.POST(requestBody);
  Serial.print("HTTP Response code: ");
  Serial.println(httpResponseCode);

  // Free resources
  http.end();
}
void loop()
{
  if (ConfigStruct.UnderProcess)
  {
    previousMillis = millis();
    bool timeout = false;
    int8_t configured; // top 1, bottom 2
    // Display the menu
    ClearScreen();
    PrintToScreen(0, 0, " TOP: Setting");
    PrintToScreen(0, 1, " BOT: Rem Lev");

    while (true)
    {

      if (digitalRead(TopButton) == LOW)
      {
        configured = 1;
        break;
      }
      if (digitalRead(BottomButton) == LOW)
      {
        configured = 2;
        break;
      }
      if (millis() - previousMillis >= interval)
      {
        // previousMillis = currentMillis;
        timeout = true;
        break;
      }
    }
    if (!timeout)
    {
      if (configured == 1)
      {
        delay(100);

        ClearScreen();
        PrintToScreen(0, 0, " PLACE CYLINDER ");
        PrintToScreen(0, 1, "PRESS: THEN =>");
        while (true)
        {
          // waiting for cylinder placement and loading of the new config
          if (digitalRead(RightButton) == LOW)
          {
            break;
          }
        }

        // get the cylinder weight form the load cell
        String outputString = " WEIGHT: ";
        float weight = scale.get_units();
        if (weight <= 0)
        {
          ClearScreen();
          PrintToScreen(0, 0, " NO GAS PLACED  ");
          PrintToScreen(0, 1, "  ===========");
        }
        else
        {
          outputString.concat(weight);
          outputString.concat(" Kg");
          ClearScreen();
          PrintToScreen(0, 0, " CYLINDER GAS");
          PrintToScreen(0, 1, outputString);

          // SAVED into spiffs
          MyLPG.CylinderCapacity = weight;
          MyLPG.GasAmount = weight;
          // save to spiffs
          // writeGasConfigToSPIFFS(MyLPG);
        }
      }
      if (configured == 2)
      {
        delay(100);
        ClearScreen();
        PrintToScreen(0, 0, " TUNE THE POT ");
        PrintToScreen(0, 1, "TO SET REM LEVEL");
        delay(2000);
        ClearScreen();
        PrintToScreen(0, 0, "  VALUE SET    ");
        int percentage;
        while (true)
        {
          percentage = map(analogRead(Potentiometer), 0, 4095, 0, 100);
          String Percentages = "       " + String(percentage) + "%" + "   ";
          PrintToScreen(0, 1, Percentages);

          if (digitalRead(RightButton) == LOW)
          {
            break;
          }
        }
        ClearScreen();
        MyLPG.ReminderPercentage = percentage;
        String printabelScreen = " % REMINDER SET";
        String printabelScreen2 = "      " + String(percentage) + "%" + "   ";
        PrintToScreen(0, 0, printabelScreen);
        PrintToScreen(0, 1, printabelScreen2);
        // delay(1000);
      }
    }
    else
    {
      ClearScreen();
      PrintToScreen(0, 0, "    TIMEOUT     ");
      PrintToScreen(0, 1, " PLEASE SELECT");
      delay(2000);
      ClearScreen();

      DisplayGasAmount();
    }

    ConfigStruct.UnderProcess = false;
    ClearScreen();
    DisplayGasAmount();
  }

  float percentageComputed = ((MyLPG.GasAmount / MyLPG.CylinderCapacity) * 100);
  //  checking if the grade has passed
  if (percentageComputed <= MyLPG.ReminderPercentage && percentageComputed != 0)
  {
    digitalWrite(Buzzer, HIGH);
    delay(500);
    digitalWrite(Buzzer, LOW);
    delay(500);
  }

  // delay(1000);

  // Read and update the information
  if (scale.is_ready())
  {
    scale.set_scale(-528793 / 1);
    // scale.tare();
    float weight = scale.get_units();

    if (weight > 0)
    {
      // Serial.print("Gas measured:                        ");
      // Serial.println(weight);
      MyLPG.GasAmount = weight;
    }
    // count how many before saving
    if (avoidMuchCounting == 0 != avoidMuchCounting >= 2)
    {
      // saves into spiffs
      // writeGasConfigToSPIFFS(MyLPG);
      SendData(MyLPG.GasAmount, MyLPG.ReminderPercentage, MyLPG.CylinderCapacity);
      avoidMuchCounting = 1;
    }
    avoidMuchCounting++;
    DisplayGasAmount();
    delay(1000);
  }
  // debug buttons
  // Serial.print("Top: ");
  // Serial.println(digitalRead(TopButton));
  // Serial.print("Bot: ");
  // Serial.println(digitalRead(BottomButton));
  // Serial.print("Left: ");
  // Serial.println(digitalRead(LeftButton));
  // Serial.print("Righ: ");
  // Serial.println(digitalRead(RightButton));
  // Serial.print("Gas amount: ");
  // Serial.println(MyLPG.GasAmount);
  // Serial.print("Gas Capacity: ");
  // Serial.println(MyLPG.CylinderCapacity);
  // Serial.print("%: ");
  // Serial.println(percentageComputed);

  // if(scale.is_ready()) {
  //     Serial.print("one reading:\t");
  // Serial.print(scale.get_units(), 1);
  // Serial.print("\t| average:\t");
  // Serial.println(scale.get_units(10), 5);

  // scale.power_down();             // put the ADC in sleep mode
  // delay(2000);
  // scale.power_up();
  // }
}

// #include <Arduino.h>
// #include "HX711.h"
// #include "soc/rtc.h"
// #include <LiquidCrystal_I2C.h>

// // HX711 circuit wiring
// const int LOADCELL_DOUT_PIN = 16;
// const int LOADCELL_SCK_PIN = 4;

// HX711 scale;

// void setup() {
//   Serial.begin(9600);
//   rtc_cpu_freq_config_t config;
//   rtc_clk_cpu_freq_get_config(&config);
//   rtc_clk_cpu_freq_to_config(RTC_CPU_FREQ_80M, &config);
//   rtc_clk_cpu_freq_set_config_fast(&config);
//   Serial.println("HX711 Demo");

//   Serial.println("Initializing the scale");

//   scale.begin(LOADCELL_DOUT_PIN, LOADCELL_SCK_PIN);

//   Serial.println("Before setting up the scale:");
//   Serial.print("read: \t\t");
//   Serial.println(scale.read());      // print a raw reading from the ADC

//   Serial.print("read average: \t\t");
//   Serial.println(scale.read_average(20));   // print the average of 20 readings from the ADC

//   Serial.print("get value: \t\t");
//   Serial.println(scale.get_value(5));   // print the average of 5 readings from the ADC minus the tare weight (not set yet)

//   Serial.print("get units: \t\t");
//   Serial.println(scale.get_units(5), 1);  // print the average of 5 readings from the ADC minus tare weight (not set) divided
//             // by the SCALE parameter (not set yet)

// //   scale.set_scale(INSERT YOUR CALIBRATION FACTOR);
//   scale.set_scale(-528793/1);                      // this value is obtained by calibrating the scale with known weights; see the README for details
//   scale.tare();               // reset the scale to 0

//   Serial.println("After setting up the scale:");

//   Serial.print("read: \t\t");
//   Serial.println(scale.read());                 // print a raw reading from the ADC

//   Serial.print("read average: \t\t");
//   Serial.println(scale.read_average(20));       // print the average of 20 readings from the ADC

//   Serial.print("get value: \t\t");
//   Serial.println(scale.get_value(5));   // print the average of 5 readings from the ADC minus the tare weight, set with tare()

//   Serial.print("get units: \t\t");
//   Serial.println(scale.get_units(5), 1);        // print the average of 5 readings from the ADC minus tare weight, divided
//             // by the SCALE parameter set with set_scale

//   Serial.println("Readings:");
// }

// void loop() {
//   Serial.print("one reading:\t");
//   Serial.print(scale.get_units(), 1);
//   Serial.print("\t| average:\t");
//   Serial.println(scale.get_units(10), 5);

//   scale.power_down();             // put the ADC in sleep mode
//   delay(5000);
//   scale.power_up();
// }

// calibration

// #include <Arduino.h>
// // #include "HX711.h"
// // #include "soc/rtc.h"
// #include <LiquidCrystal_I2C.h>
// #include "HX711.h"

// // HX711 circuit wiring
// const int LOADCELL_DOUT_PIN = 16;
// const int LOADCELL_SCK_PIN = 4;

// HX711 scale;

// void setup() {
//   Serial.begin(9600);
//   scale.begin(LOADCELL_DOUT_PIN, LOADCELL_SCK_PIN);
// }

// void loop() {

//   if (scale.is_ready()) {
//     scale.set_scale();
//     Serial.println("Tare... remove any weights from the scale.");
//     delay(5000);
//     scale.tare();
//     Serial.println("Tare done...");
//     Serial.print("Place a known weight on the scale...");
//     delay(5000);
//     long reading = scale.get_units(10);
//     Serial.print("Result: ");
//     Serial.println(reading);
//   }
//   else {
//     Serial.println("HX711 not found.");
//   }
//   delay(1000);
// }



// #include <Arduino.h>
// #include <SoftwareSerial.h>
// #include <ArduinoJson.h>
// #include <thread>
// #include <freertos/FreeRTOS.h>
// #define BMS 18
// #define LED_INDICATOR 2
// #define DTR_GSM 19
// #define RX_PIN 18
// #define TX_PIN 5
// #define RING 23

// const char *kitID = "6677be97bb2d5395b2c5c55b";

// SoftwareSerial gsm(TX_PIN, RX_PIN);

// void waitForResponse()
// {
//   while (!gsm.available())
//   {
//     delay(500);
//   }
//   while (gsm.available())
//   {
//     Serial.write(gsm.read());
//   }
// }

// void sendCommand(String command)
// {
//   gsm.println(command);
//   waitForResponse();
// }

// void setup()
// {
//   Serial.begin(9600);
//   gsm.begin(9600);
//   pinMode(BMS, OUTPUT);
//   pinMode(DTR_GSM, OUTPUT);
//   digitalWrite(DTR_GSM, LOW);
//   pinMode(LED_INDICATOR, OUTPUT);
//   delay(5000);

//   Serial.println("Initializing...");

//   // Check if the module is responding
//   gsm.println("AT");
//   waitForResponse();
//   delay(1500);
//   gsm.println("AT+HTTPSSL=?");
//   waitForResponse();

//   delay(1500);

//   gsm.println("AT+HTTPSSL?");
//   waitForResponse();

//   gsm.println("AT+HTTPSSL=1");
//   waitForResponse();
// }

// void loop()
// {
//   // Serial.println("Not reached....");
//   // delay(2000);
//   // digitalWrite(DTR_GSM, HIGH);
//   // delay(10000);
//   // digitalWrite(DTR_GSM, LOW);
//   // gsm.print("AT");
//   // delay(1000);
//   // configureGPRS();
//   // delay(2000);
//   // performGETRequest("167.71.173.184", "3485", "/test");

//   // updateSerial();
//   // SendMSG();
//   // updateSerial();
//   // delay(1000);
// }

// void performGETRequest(String server, String port, String path)
// {
//   sendCommand("AT+HTTPINIT");
//   sendCommand("AT+HTTPPARA=\"CID\",1");
//   sendCommand("AT+HTTPPARA=\"URL\",\"http://" + server + ":" + port + path + "\"");
//   sendCommand("AT+HTTPACTION=0");
//   waitForResponse();
//   Serial.println("--------------------------");
//   sendCommand("AT+HTTPREAD");
//   delay(6000);
//   waitForResponse();
//   Serial.println("At last..........");

//   sendCommand("AT+HTTPTERM");
// }

// bool waitForHTTPResponse()
// {
//   unsigned long startTime = millis();
//   while (!gsm.available() && millis() - startTime < 10000)
//   {
//     delay(100);
//   }
//   while (gsm.available())
//   {
//     String response = gsm.readString();
//     Serial.println(response);
//     if (response.indexOf("+HTTPACTION:") != -1)
//     {
//       return true;
//     }
//   }
//   return false;
// }

// void configureGPRS()
// {
//   gsm.println("AT+SAPBR=3,1,\"CONTYPE\",\"GPRS\"");
//   waitForResponse();

//   gsm.println("AT+SAPBR=3,1,\"APN\",\"internet\"");
//   waitForResponse();

//   gsm.println("AT+SAPBR=1,1");
//   waitForResponse();

//   gsm.println("AT+SAPBR=2,1");
//   waitForResponse();
// }

// void sendDataToServer()
// {
//   gsm.println("AT+HTTPINIT");
//   waitForResponse();
//   gsm.println("AT+HTTPPARA=\"CID\",1");
//   waitForResponse();
//   gsm.println("AT+HTTPPARA=\"URL\",\"http://167.71.173.184:3485/test\"");
//   waitForResponse();

//   gsm.println("AT+HTTPACTION=0");
//   waitForResponse();

//   gsm.println("AT+HTTPREAD");
//   waitForResponse();

//   gsm.println("AT+HTTPTERM");
//   waitForResponse();
// }





