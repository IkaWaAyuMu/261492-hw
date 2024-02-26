/**
 * @file      src.ino
 * @author    Chayanon Pitak (Chayanon_Pitak@cmu.ac.th)
 */

#include "utilities.h"
#include "MQTT_credentials.h"
#include "MQTT_cert.h"
#include <Arduino.h>
#include <SoftwareSerial.h>
#include <TinyGsmClient.h>
#include <TinyGPSPlus.h>
#include <WiFiClientSecure.h>
#include <PubSubClient.h>
#include <ArduinoJson.h>

#define TINY_GSM_RX_BUFFER 1024 // Set RX buffer to 1Kb

// Define the serial console for debug prints, if needed
#define TINY_GSM_DEBUG TinyGSMSerialMonitor

// See all AT commands, if wanted
// #define DUMP_AT_COMMANDS

#ifdef DUMP_AT_COMMANDS
#include <StreamDebugger.h>

StreamDebugger debugger(SerialAT, Serial);
TinyGsm modem(debugger);
#else
TinyGsm modem(SerialAT);
#endif

WiFiClientSecure secureWiFiClient;
PubSubClient mqttClient(secureWiFiClient);

TinyGPSPlus gps;
SoftwareSerial gpsss(GPS_RX, GPS_TX);

void writeRGB(int r, int g, int b)
{
    ledcWrite(0, 255 - r);
    ledcWrite(1, 255 - g);
    ledcWrite(2, 255 - b);
}

void printTime()
{
    Serial.print("[");
    Serial.print(millis() / 1000.0);
    Serial.print("] \t");
}

// This custom version of delay() ensures that the gps object
// is being "fed".
void smartDelay(unsigned long ms)
{
    unsigned long start = millis();
    do
    {
        while (gpsss.available())
            gps.encode(gpsss.read());
    } while (millis() - start < ms);
}

// Read analog pin and return the median value of the last n readings.
unsigned short analogDownsamplingReadMedian(uint8_t pin, unsigned int samplingRate = 5)
{
    unsigned short *results = (unsigned short *)malloc(samplingRate * sizeof(short));
    for (unsigned int i = 0; i < samplingRate; i++)
    {
        results[i] = analogRead(pin);
    }
    for (unsigned int i = 0; i < samplingRate; i++)
    {
        for (unsigned int j = i + 1; j < samplingRate; j++)
        {
            if (results[i] > results[j])
            {
                unsigned short temp = results[i];
                results[i] = results[j];
                results[j] = temp;
            }
        }
    }
    unsigned short result = results[samplingRate / 2] + (samplingRate % 2 == 0 ? results[samplingRate / 2 - 1] : 0);
    free(results);
    return result;
}

// Read analog pin pair and return array of the downsampled pair.
unsigned short *readPair(short pin0, short pin1)
{
    unsigned short *result = (unsigned short *)malloc(2 * sizeof(short));
    result[0] = analogDownsamplingReadMedian(pin0);
    result[1] = analogDownsamplingReadMedian(pin1);
    return result;
}

String ToISO8601String(int year, int month, int day, int hour, int minute, int second, float timezone);

int lastResetYear;
int lastResetMonth;
int lastResetDay;
int lastResetHour;
int lastResetMinute;
int lastResetSecond;
float lastResetTimezone;

void setup()
{
    Serial.begin(115200);

    // Set status light up / Status LED check
    ledcSetup(0, 12000, 8);
    ledcSetup(1, 12000, 8);
    ledcSetup(2, 12000, 8);
    ledcAttachPin(STATUS_LED_RED, 0);
    ledcAttachPin(STATUS_LED_GREEN, 1);
    ledcAttachPin(STATUS_LED_BLUE, 2);
    pinMode(LED_IN1, OUTPUT);
    pinMode(LED_OUT1, OUTPUT);
    pinMode(LED_IN2, OUTPUT);
    pinMode(LED_OUT2, OUTPUT);
    digitalWrite(LED_IN1, HIGH);
    digitalWrite(LED_OUT1, HIGH);
    digitalWrite(LED_IN2, HIGH);
    digitalWrite(LED_OUT2, HIGH);
    writeRGB(0, 0, 0);
    delay(500);
    digitalWrite(LED_IN1, LOW);
    digitalWrite(LED_OUT1, LOW);
    digitalWrite(LED_IN2, LOW);
    digitalWrite(LED_OUT2, LOW);
    writeRGB(255, 0, 0);
    delay(500);
    writeRGB(0, 255, 0);
    delay(500);
    writeRGB(0, 0, 255);
    delay(500);
    writeRGB(0, 0, 0);
    delay(500);

    /* STATUS LIGHT
     * YELLOW    Setting modem & SIM card up
     * RED       ERROR
     * GREEN     SIM card & Network online
     */

    // Set modem up
    writeRGB(255, 255, 0); // Yellow
    SerialAT.begin(115200, SERIAL_8N1, MODEM_RX_PIN, MODEM_TX_PIN);
    pinMode(BOARD_POWERON_PIN, OUTPUT);
    digitalWrite(BOARD_POWERON_PIN, HIGH);

    pinMode(MODEM_RESET_PIN, OUTPUT);
    digitalWrite(MODEM_RESET_PIN, !MODEM_RESET_LEVEL);
    delay(100);
    digitalWrite(MODEM_RESET_PIN, MODEM_RESET_LEVEL);
    delay(2600);
    digitalWrite(MODEM_RESET_PIN, !MODEM_RESET_LEVEL);

    pinMode(BOARD_PWRKEY_PIN, OUTPUT);
    digitalWrite(BOARD_PWRKEY_PIN, LOW);
    delay(100);
    digitalWrite(BOARD_PWRKEY_PIN, HIGH);
    delay(100);
    digitalWrite(BOARD_PWRKEY_PIN, LOW);

    // Check if the modem is online
    printTime();
    Serial.print("Starting modem ");

    int retry = 0;
    while (!modem.testAT(1000))
    {
        Serial.print(".");
        if (retry++ > 10)
        {
            digitalWrite(BOARD_PWRKEY_PIN, LOW);
            delay(100);
            digitalWrite(BOARD_PWRKEY_PIN, HIGH);
            delay(1000);
            digitalWrite(BOARD_PWRKEY_PIN, LOW);
            retry = 0;
        }
    }
    Serial.println();

    // Check if SIM card is online
    SimStatus sim = SIM_ERROR;
    while (sim != SIM_READY)
    {
        sim = modem.getSimStatus();
        switch (sim)
        {
        case SIM_READY:
            printTime();
            Serial.println("SIM card online");
            break;
        case SIM_LOCKED:
            printTime();
            Serial.println("The SIM card is locked. Please unlock the SIM card first.");
            // const char *SIMCARD_PIN_CODE = "123456";
            // modem.simUnlock(SIMCARD_PIN_CODE);
            printTime();
            Serial.println(" ===== HALT =====");
            writeRGB(255, 0, 0); // Red
            break;
        default:
            break;
        }
        delay(1000);
    }

    // Check network registration status and network signal status
    int16_t sq;
    printTime();
    Serial.println("Wait for the modem to register with the network.");
    RegStatus status = REG_NO_RESULT;
    while (status == REG_NO_RESULT || status == REG_SEARCHING || status == REG_UNREGISTERED)
    {
        status = modem.getRegistrationStatus();
        switch (status)
        {
        case REG_UNREGISTERED:
        case REG_SEARCHING:
            sq = modem.getSignalQuality();
            printTime();
            Serial.printf("Signal Quality:%d\n", sq);
            delay(1000);
            break;
        case REG_DENIED:
            printTime();
            Serial.println("Network registration was rejected, please check if the APN is correct");
            printTime();
            Serial.println(" ===== HALT =====");
            writeRGB(255, 0, 0); // Red
            return;
        case REG_OK_HOME:
            printTime();
            Serial.println("Online registration successful");
            break;
        case REG_OK_ROAMING:
            printTime();
            Serial.println("Network registration successful, currently in roaming mode");
            break;
        default:
            printTime();
            Serial.printf("Registration Status:%d\n", status);
            delay(1000);
            break;
        }
    }
    Serial.println();
    printTime();
    Serial.printf("Registration Status:%d\n", status);
    Serial.println();

    String ueInfo;
    if (modem.getSystemInformation(ueInfo))
    {
        printTime();
        Serial.print("Inquiring UE system information: ");
        Serial.println(ueInfo);
    }

    if (!modem.enableNetwork())
    {
        printTime();
        Serial.println("Enable network failed!");
        printTime();
        Serial.println(" ===== HALT =====");
        writeRGB(255, 0, 0); // Red
        return;
    }

    delay(1000);

    printTime();
    Serial.println("Network IP:" + modem.getLocalIP());
    writeRGB(0, 255, 0); // Green

    delay(1000);

    /* STATUS LIGHT
     * YELLOW    Setting NTP up
     * ORANGE    Retrying
     * RED       ERROR
     * GREEN     NTP online
     */

    // Set NTP up
    writeRGB(255, 255, 0); // Yellow
    printTime();
    Serial.print("Connecting to NTP server ");
    Serial.println(NTP_SERVER);
    modem.NTPServerSync(NTP_SERVER, 0);
    for (int i = 0; i < 10; i++)
    {
        if (modem.getGSMDateTime(DATE_FULL) == "")
        {
            printTime();
            Serial.println("Failed to obtain time. Trying again in 5 second. ");
            writeRGB(255, 165, 0); // Orange
        }
        else
            break;
        delay(5000);
    }
    if (modem.getGSMDateTime(DATE_FULL) == "")
    {
        printTime();
        Serial.println("Failed to obtain time. Please check NTP server or network connection.");
        printTime();
        Serial.println(" ===== HALT =====");
        writeRGB(255, 0, 0); // Red
        return;
    }
    printTime();
    Serial.println("Time obtained: " + modem.getGSMDateTime(DATE_FULL));
    writeRGB(0, 255, 0); // Green
    delay(1000);

    /* STATUS LIGHT
     * YELLOW    Setting MQTT up
     * ORANGE    Retrying
     * GREEN     MQTT online
     */

    // Set MQTT up

    writeRGB(255, 255, 0); // Yellow
    // modem.mqtt_begin(true, true); // SSL, SNI
    // modem.mqtt_set_certificate(MQTT_CA);

    // printTime();
    // Serial.println("Connecting to " + String(MQTT_BROKER));

    // bool mqtt_status = modem.mqtt_connect(0, MQTT_BROKER, MQTT_BROKER_PORT, MQTT_CLIENT_ID, MQTT_BROKER_USERNAME, MQTT_BROKER_PASSWORD);
    // while (!mqtt_status || !modem.mqtt_connected())
    // {
    //     printTime();
    //     Serial.println("Failed to connect to " + String(MQTT_BROKER) + ". Trying again in 5 seconds.");
    //     writeRGB(255, 165, 0); // Orange
    //     delay(5000);
    //     mqtt_status = modem.mqtt_connect(0, MQTT_BROKER, MQTT_BROKER_PORT, MQTT_CLIENT_ID, MQTT_BROKER_USERNAME, MQTT_BROKER_PASSWORD);
    // }

    // Connect to wifi
    WiFi.begin("139/24", "WIFI13924");
    printTime();
    Serial.print("Connecting to wifi");

    while (WiFi.status() != WL_CONNECTED)
    {
        Serial.print(".");
        delay(100);
    }
    secureWiFiClient.setCACert(MQTT_CA);
    Serial.println();
    printTime();
    Serial.println("Connected to wifi");

    // Connect to MQTT broker
    mqttClient.setServer(MQTT_BROKER, MQTT_BROKER_PORT);
    printTime();
    Serial.println("Connecting to " + String((char *)MQTT_BROKER) + ":" + String(MQTT_BROKER_PORT));
    while (!mqttClient.connect(MQTT_CLIENT_ID, MQTT_BROKER_USERNAME, MQTT_BROKER_PASSWORD))
    {
        Serial.print(".");
        delay(5000);
    }

    printTime();
    Serial.println("Connected to MQTT broker");
    writeRGB(0, 255, 0); // Green
    delay(1000);

    /* STATUS LIGHT
     * YELLOW    Setting GPS up, waiting for first GPS data
     * GREEN     First GPS data received
     */

    // Set GPS up
    writeRGB(255, 255, 0); // Yellow
    // gpsss.begin(GPS_BAUD);

    // while (true)
    // {
    //     gps.encode(gpsss.read());
    //     if (gps.location.isValid())
    //     {
    //         printTime();
    //         Serial.print("First GPS data received: ");
    //         Serial.print(gps.location.lat(), 6);
    //         Serial.print(", ");
    //         Serial.println(gps.location.lng(), 6);
    //         writeRGB(0, 255, 0); // Green
    //         break;
    //     }
    //     else
    //     {
    //         printTime();
    //         Serial.println("No GPS data received yet");
    //         smartDelay(5000);
    //     }
    // }

    delay(1000);

    // TODO: GET STATIONS LOCATION

    /* STATUS LIGHT
     * YELLOW    Set other PINs up
     * GREEN     PINs ready
     */

    // Set other PINs up
    writeRGB(255, 255, 0); // Yellow
    pinMode(IN1, INPUT);
    pinMode(OUT1, INPUT);
    pinMode(IN2, INPUT);
    pinMode(OUT2, INPUT);
    // pinMode(POT_IN1, INPUT);
    // pinMode(POT_OUT1, INPUT);
    // pinMode(POT_IN2, INPUT);
    // pinMode(POT_OUT2, INPUT);

    printTime();
    Serial.println("Pin setup completed");
    writeRGB(0, 255, 0); // Green

    smartDelay(1000);

    modem.getNetworkTime(&lastResetYear, &lastResetMonth, &lastResetDay, &lastResetHour, &lastResetMinute, &lastResetSecond, &lastResetTimezone);

    // End of setup
    printTime();
    Serial.println("Setup completed");
    Serial.println("=============================================");
    Serial.println(" Send 'd' to toggle debug mode (default off)");
    Serial.println(" Send 'r' to reset counters");
    Serial.println("=============================================");
}

bool debug = false;

// State machine states
enum state
{
    IDLE = 0,
    SEMI_OUT_1 = -1,
    SEMI_OUT_2 = -2,
    SEMI_OUT_3 = -3,
    OUT = -4,
    SEMI_IN_1 = 1,
    SEMI_IN_2 = 2,
    SEMI_IN_3 = 3,
    IN = 4,
    SPECIAL = 6
};
state currentState1 = IDLE;
state currentState2 = IDLE;

state getState(state currState, bool inside, bool outside)
{
    switch (currState)
    {
    case IDLE: // 0 0
        if (!inside && outside)
            return SEMI_IN_1;
        if (inside && !outside)
            return SEMI_OUT_1;
        if (inside && outside)
            return SPECIAL;
        break;
    // IN
    case SEMI_IN_1: // 0 1
        if (inside && outside)
            return SEMI_IN_2;
        if (inside && !outside)
            return SEMI_IN_3;
        if (inside && !outside)
            return SEMI_IN_3;
        if (!inside && !outside)
            return IDLE;
        break;
    case SEMI_IN_2: // 1 1
        if (inside && !outside)
            return SEMI_IN_3;
        if (!inside && !outside)
            return IN;
        if (!inside && outside)
            return SEMI_IN_1;
        break;
    case SEMI_IN_3: // 1 0
        if (!inside && !outside)
            return IN;
        if (inside && outside)
            return SEMI_IN_2;
        if (!inside && outside)
            return SEMI_IN_1;
        if (!inside && outside)
            return SEMI_IN_1;
        break;
    // OUT
    case SEMI_OUT_1: // 1 0
        if (inside && outside)
            return SEMI_OUT_2;
        if (!inside && outside)
            return SEMI_OUT_3;
        if (!inside && outside)
            return SEMI_OUT_3;
        if (!inside && !outside)
            return IDLE;
        break;
    case SEMI_OUT_2: // 1 1
        if (!inside && outside)
            return SEMI_OUT_3;
        if (!inside && !outside)
            return OUT;
        if (inside && !outside)
            return SEMI_OUT_1;
        break;
    case SEMI_OUT_3: // 0 1
        if (!inside && !outside)
            return OUT;
        if (inside && outside)
            return SEMI_OUT_2;
        if (inside && !outside)
            return SEMI_OUT_1;
        if (inside && !outside)
            return SEMI_OUT_1;
        break;
    // SPECIAL
    case SPECIAL: // 1 1
        if (!inside && outside)
            return SEMI_OUT_3;
        if (inside && !outside)
            return SEMI_IN_3;
        if (!inside && !outside)
            return IDLE;
        break;
    }
    return currState;
}

void setState(state *currentState, unsigned short distances[], unsigned short bounds[], unsigned short led_pins[])
{
    bool inside = distances[0] > bounds[0];
    bool outside = distances[1] > bounds[1];

    digitalWrite(led_pins[0], inside);
    digitalWrite(led_pins[1], outside);

    *currentState = getState(*currentState, inside, outside);
}

// Occupancy counter
short people = 0;
short in = 0;
short out = 0;

void resetCounters(int year, int month, int day, int hour, int minute, int second, float timezone)
{
    currentState1 = IDLE;
    currentState2 = IDLE;
    people = 0;
    in = 0;
    out = 0;

    lastResetYear = year;
    lastResetMonth = month;
    lastResetDay = day;
    lastResetHour = hour;
    lastResetMinute = minute;
    lastResetSecond = second;
    lastResetTimezone = timezone;

    if (debug)
    {
        printTime();
        Serial.print("Resettted at ");
        Serial.print(ToISO8601String(year, month, day, hour, minute, second, timezone));
        Serial.println(", people: " + String(people));
    }
}

void setOccupancy(state *currentState)
{
    switch (*currentState)
    {
    case OUT:
        if (people > 0)
        {
            people--;
            out++;
        }
        if (debug)
        {
            printTime();
            Serial.println("Exited, people: " + String(people));
        }
        *currentState = IDLE;
        break;
    case IN:
        people++;
        in++;
        if (debug)
        {
            printTime();
            Serial.println("Entered, people: " + String(people));
        }
        *currentState = IDLE;
        break;
    }
}

// Debugging
void plotSensor(char label, unsigned short *distances, unsigned short *bounds, state *currentState)
{
    Serial.println(">INSIDE_" + String(label) + ":" + String(distances[0]));
    Serial.println(">OUTSIDE_" + String(label) + ":" + String(distances[1]));

    Serial.println(">BOUNDS_INSIDE_" + String(label) + ":" + String(bounds[0]));
    Serial.println(">BOUNDS_OUTSIDE_" + String(label) + ":" + String(bounds[1]));

    Serial.println(">BOOL_INSIDE_" + String(label) + ":" + String(distances[0] > bounds[0]));
    Serial.println(">BOOL_OUTSIDE_" + String(label) + ":" + String(distances[1] > bounds[1]));

    Serial.println(">STATE_" + String(label) + ":" + String(*currentState));
}

unsigned short LED1[2] = {LED_IN1, LED_OUT1};
unsigned short LED2[2] = {LED_IN2, LED_OUT2};

unsigned int lastSend = 0;
unsigned const int sendInterval = 300000;

void loop()
{

    // Get current Time
    int year;
    int month;
    int day;
    int hour;
    int minute;
    int second;
    int millisecond;
    float timezone;
    modem.getNetworkTime(&year, &month, &day, &hour, &minute, &second, &timezone);

    bool forceSend = false;

    // Serial Check
    if (Serial.available() > 0)
    {
        char c = Serial.read();
        if (c == 'd')
        {
            debug = !debug;
            if (debug)
            {
                printTime();
                Serial.println("Debug mode on");
            }
            else
            {
                printTime();
                Serial.println("Debug mode off");
            }
        }
        else if (c == 'r')
            resetCounters(year,  month,  day,  hour,  minute,  second,  timezone);
        else if (c == 's')
            forceSend = true;
    }

    // Perform a reset at midnight (in UTC+7)
    if ((day + (hour + 7 > 23 ? 1 : 0)) != (lastResetDay + (hour + 7 > 23 ? 1 : 0)) )
    {
        resetCounters(year,  month,  day,  hour,  minute,  second,  timezone);
    }

    // Read sensors
    unsigned short *distances1 = readPair(IN1, OUT1);
    // unsigned short *bounds1 = readPair(POT1, POT1);
    unsigned short bounds1[] = {1800, 1800};

    unsigned short *distances2 = readPair(IN2, OUT2);
    // unsigned short *bounds2 = readPair(POT2, POT2);
    unsigned short bounds2[] = {1800, 1800};

    setState(&currentState1, distances1, bounds1, LED1);
    setState(&currentState2, distances2, bounds2, LED2);

    if (debug)
    {
        plotSensor('1', distances1, bounds1, &currentState1);
        plotSensor('2', distances2, bounds2, &currentState2);
        Serial.print(">PEOPLE:");
        Serial.println(people);
        Serial.print(">IN:");
        Serial.println(in);
        Serial.print(">OUT:");
        Serial.println(out);
    }

    setOccupancy(&currentState1);
    setOccupancy(&currentState2);

    free(distances1);
    // free(bounds1);
    free(distances2);
    // free(bounds2);

    // MQTT
    if (forceSend || (millis() - lastSend > sendInterval) || lastSend == 0)
    {
        if (mqttClient.connected())
        {
            JsonDocument doc;

            doc["id"] = MQTT_CLIENT_ID;
            doc["time"] = ToISO8601String(year, month, day, hour, minute, second, timezone);
            doc["lastReset"] = ToISO8601String(lastResetYear, lastResetMonth, lastResetDay, lastResetHour, lastResetMinute, lastResetSecond, lastResetTimezone);
            JsonObject location = doc["location"].to<JsonObject>();
            location["latitude"] = 00.0;
            location["longitude"] = 00.0;
            JsonObject data = doc["data"].to<JsonObject>();
            data["enter"] = in;
            data["exit"] = out;
            data["current"] = people;

            String output;
            serializeJson(doc, output);
            mqttClient.publish(MQTT_PUBLISHTO_TOPIC, output.c_str());

            if (debug)
            {
                printTime();
                Serial.println("Published to " + String(MQTT_PUBLISHTO_TOPIC) + " as\n" + output);
            }

            lastSend = millis();
            forceSend = false;
        }
        else if (!mqttClient.connected())
        {
            printTime();
            Serial.println("Reconnecting to MQTT broker");
            if (!mqttClient.connect(MQTT_CLIENT_ID, MQTT_BROKER_USERNAME, MQTT_BROKER_PASSWORD)) lastSend += 10000; // Retry in 10 seconds
        }
    }
}

String ToISO8601String(int year, int month, int day, int hour, int minute, int second, float timezone)
{
    char buffer[30];
    if (timezone != 0)
    {
        hour -= (int)timezone;
        minute -= (int)(timezone * 60) % 60;
    }
    sprintf(buffer, "%04d-%02d-%02dT%02d:%02d:%02d.000Z", year, month, day, hour, minute, second);
    return String(buffer);
}
