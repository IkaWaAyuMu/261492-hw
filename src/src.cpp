/**
 * @file      src.ino
 * @author    Chayanon Pitak (Chayanon_Pitak@cmu.ac.th)
 */

#include "utilities.h"
#include "MQTT_credentials.h"
#include <Arduino.h>
#include <SoftwareSerial.h>
#include <TinyGsmClient.h>
#include <ESP32Time.h>
#include <TinyGPSPlus.h>
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

TinyGsmClient client(modem);
PubSubClient mqtt(client);

TinyGPSPlus gps;
SoftwareSerial gpsss(GPS_RX, GPS_TX);

ESP32Time rtc(0);

bool debug = false;

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
    unsigned long start = pdTICKS_TO_MS(xTaskGetTickCount());
    do
    {
        while (gpsss.available())
            gps.encode(gpsss.read());
    } while (pdTICKS_TO_MS(xTaskGetTickCount()) - start < ms);
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

TaskHandle_t Task1 = NULL;
TaskHandle_t Task2 = NULL;

void read(void *args);
void send(void *args);

int lastResetYear, lastResetMonth, lastResetDay, lastResetHour, lastResetMinute, lastResetSecond;
float lastResetTimezone;

JsonDocument locations;
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
    int year, month, day, hour, minute, second;
    float timezone;
    modem.getNetworkTime(&year, &month, &day, &hour, &minute, &second, &timezone);
    rtc.setTime(second, minute, hour, day, month, year);

    writeRGB(0, 255, 0); // Green
    delay(1000);

    /* STATUS LIGHT
     * YELLOW    Setting MQTT up
     * ORANGE    Retrying
     * GREEN     MQTT online
     */

    // Set MQTT up

    writeRGB(255, 255, 0); // Yellow

    printTime();
    Serial.println("Connecting to " + String(MQTT_BROKER));

    mqtt.setServer(MQTT_BROKER, 1883);

    mqtt.connect(CLIENT_ID, MQTT_BROKER_USERNAME, MQTT_BROKER_PASSWORD);
    while (!mqtt.connected())
    {
        printTime();
        Serial.println("Failed to connect to " + String(MQTT_BROKER) + ". Trying again in 5 seconds.");
        writeRGB(255, 165, 0); // Orange
        delay(5000);
        mqtt.connect(CLIENT_ID, MQTT_BROKER_USERNAME, MQTT_BROKER_PASSWORD);
    }

    printTime();
    Serial.println("Connected to MQTT broker");
    writeRGB(0, 255, 0); // Green
    delay(1000);

    /* STATUS LIGHT
     * YELLOW    Fetching data.
     * ORANGE    Retrying
     * RED       ERROR
     * GREEN     First GPS data received
     */

    // Get station location
    writeRGB(255, 255, 0); // Yellow
    modem.https_begin();

    if (!modem.https_set_url((String)LOCATION_ROUTE_URL + "?id=" + String(CLIENT_ID)))
    {
        printTime();
        Serial.println("Failed to set the URL. Please check the validity of the URL!");
        writeRGB(255, 0, 0); // Red
        return;
    }

    int httpStatus = 0;
    int httpRetryCounter = 0;
    httpStatus = modem.https_get();
    while (httpStatus != 200)
    {
        writeRGB(255, 165, 0); // Orange
        printTime();
        Serial.print("Location fetch failed : ");
        Serial.println(httpStatus);
        if (httpRetryCounter++ > 10)
        {
            printTime();
            Serial.println("Failed to fetch location. Please check the server.");
            writeRGB(255, 0, 0); // Red
            return;
        }
        delay(5000);
        httpStatus = modem.https_get();
    }
    writeRGB(255, 255, 0); // Yellow

    DeserializationError err = deserializeJson(locations, modem.https_body());

    if (err)
    {
        printTime();
        Serial.print("Deserialization error:");
        Serial.println(err.f_str());
        writeRGB(255, 0, 0); // Red
        return;
    }

    printTime();
    Serial.println("Location fetched");
    writeRGB(0, 255, 0); // Green
    delay(1000);

    /* STATUS LIGHT
     * GREEN     GPS ready
     */

    writeRGB(0, 255, 0); // Green
    gpsss.begin(9600);
    delay(1000);

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
    pinMode(POT_IN1, INPUT);
    pinMode(POT_OUT1, INPUT);
    pinMode(POT_IN2, INPUT);
    pinMode(POT_OUT2, INPUT);

    printTime();
    Serial.println("Pin setup completed");
    writeRGB(0, 255, 0); // Green

    modem.getNetworkTime(&lastResetYear, &lastResetMonth, &lastResetDay, &lastResetHour, &lastResetMinute, &lastResetSecond, &lastResetTimezone);

    // End of setup
    printTime();
    Serial.println("Setup completed");
    Serial.println("=============================================");
    Serial.println(" Send 'd' to toggle debug mode (default off)");
    Serial.println(" Send 'r' to reset counters");
    Serial.println(" Send 's' to force send data to MQTT broker");
    Serial.println("=============================================");

    // Create tasks

    xTaskCreatePinnedToCore(read, "Task1", 10000, NULL, 1, &Task1, 0);
    xTaskCreatePinnedToCore(send, "Task2", 10000, NULL, 2, &Task2, 1);
}

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

bool forceSend = true;

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
    forceSend = true;
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
bool isInStation = false;
bool isStationForced = false;

void loop() {}

void read(void *args)
{
    while (true)
    {
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
            {
                resetCounters(rtc.getYear(), rtc.getMonth(), rtc.getDay(), rtc.getHour(), rtc.getMinute(), rtc.getSecond(), 0);
                forceSend = true;
            }
            else if (c == 's')
            {
                if (debug)
                {
                    printTime();
                    Serial.println("Forcing send data to MQTT broker");
                }
                forceSend = true;
            }
            else if (c == 'f')
            {
                isStationForced = !isStationForced;
                if (isStationForced)
                {
                    printTime();
                    Serial.println("Force as in station");
                }
                else
                {
                    printTime();
                    Serial.println("Station boundary check on GPS location");
                }
            }
        }

        if (isInStation || isStationForced)
        {
            writeRGB(0, 255, 0); // Green
            // Read sensors
            unsigned short *distances1 = readPair(IN1, OUT1);
            // unsigned short *bounds1 = readPair(POT_IN1, POT_OUT1);
            unsigned short bounds1[2] = {1800, 1800};

            unsigned short *distances2 = readPair(IN2, OUT2);
            // unsigned short *bounds2 = readPair(POT_IN2, POT_OUT2);
            unsigned short bounds2[2] = {1800, 1800};

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
        }
        vTaskDelay(pdMS_TO_TICKS(1));
    }
}

void send(void *args)
{
    while (true)
    {
        // Reset at midnight (in UTC+7)
        if ((rtc.getDay() + (rtc.getHour() + 7 > 23 ? 1 : 0)) != (lastResetDay + (rtc.getHour() + 7 > 23 ? 1 : 0)))
        {
            resetCounters(rtc.getYear(), rtc.getMonth(), rtc.getDay(), rtc.getHour(), rtc.getMinute(), rtc.getSecond(), 0);
        }

        while (gpsss.available() > 0)
            (gps.encode(gpsss.read()));

        float latitude = gps.location.isValid() ? gps.location.lat() : 360;
        float longitude = gps.location.isValid() ? gps.location.lng() : 360;

        if (debug)
        {
            Serial.print(">Latitude:");
            Serial.println(latitude, 6);
            Serial.print(">Longitude:");
            Serial.println(longitude, 6);
        }
        // Check for station boundary

        if (!isStationForced)
            if (latitude != 360 && longitude != 360 && locations.size() > 0)
            {
                // schema
                // [
                // {
                // "station_id": int,
                // "start_lat": float,
                // "start_lon": float,
                // "end_lat": float,
                // "end_lon": float
                // }, ...
                // ]

                for (int i = 0; i < locations.size(); i++)
                {
                    float start_lat = locations[i]["start_lat"];
                    float start_lon = locations[i]["start_lon"];
                    float end_lat = locations[i]["end_lat"];
                    float end_lon = locations[i]["end_lon"];
                    if (debug)
                    {
                    printTime();
                    Serial.print("Comparing : (");
                    Serial.print(start_lat, 6);
                    Serial.print(", ");
                    Serial.print(start_lon, 6);
                    Serial.print(") and (");
                    Serial.print(end_lat, 6);
                    Serial.print(", ");
                    Serial.print(end_lon, 6);
                    Serial.print(") with (");
                    Serial.print(latitude, 6);
                    Serial.print(", ");
                    Serial.print(longitude, 6);
                    Serial.print(") Result ");
                    Serial.println((latitude >= start_lat && latitude <= end_lat && longitude >= start_lon && longitude <= end_lon) ? "True" : "False");
                    }
                    if (latitude >= start_lat && latitude <= end_lat && longitude >= start_lon && longitude <= end_lon)
                    {
                        if (!isInStation)
                        {
                            isInStation = true;
                            if (debug)
                            {
                                printTime();
                                Serial.println("Entered station");
                            }
                            forceSend = true;
                            break;
                        }
                    }
                    else
                    {
                        if (isInStation)
                        {
                            isInStation = false;
                            if (debug)
                            {
                                printTime();
                                Serial.println("Exited station");
                            }
                            forceSend = true;
                            writeRGB(0, 0, 0);
                            break;
                        }
                    }
                }
            }
        if (debug)
        {
            Serial.print(">is_station:");
            Serial.println(isInStation || isStationForced);
        }
        // MQTT
        if (debug)
        {
            Serial.print(">time_from_lastsend:");
            Serial.println(pdTICKS_TO_MS(xTaskGetTickCount()) - lastSend);
        }
        if (forceSend || (pdTICKS_TO_MS(xTaskGetTickCount()) - lastSend > sendInterval))
        {
            writeRGB(0, 0, 255); // Red
            modem.sleepEnable(false);
            while (!modem.testAT())
            {
                if (debug)
                {
                    printTime();
                    Serial.print("Waiting for modem to wake up");
                }
                delay(500);
            }

            writeRGB(255, 165, 0); // Orange
            while (!mqtt.connected())
            {
                if (debug)
                {
                    printTime();
                    Serial.println("Reconnecting to MQTT broker");
                }
                mqtt.connect(CLIENT_ID, MQTT_BROKER_USERNAME, MQTT_BROKER_PASSWORD);
            }
            writeRGB(0, 0, 255); // Blue
            JsonDocument doc;

            doc["id"] = CLIENT_ID;
            doc["time"] = ToISO8601String(rtc.getYear(), rtc.getMonth(), rtc.getDay(), rtc.getHour(), rtc.getMinute(), rtc.getSecond(), 0);
            doc["lastReset"] = ToISO8601String(lastResetYear, lastResetMonth, lastResetDay, lastResetHour, lastResetMinute, lastResetSecond, lastResetTimezone);
            JsonObject location = doc["location"].to<JsonObject>();
            if (latitude != 360)
                location["latitude"] = latitude;
            if (longitude != 360)
                location["longitude"] = longitude;
            JsonObject data = doc["data"].to<JsonObject>();
            data["enter"] = in;
            data["exit"] = out;
            data["current"] = people;

            String output;
            serializeJson(doc, output);
            mqtt.publish(MQTT_PUBLISHTO_TOPIC, output.c_str());

            if (debug)
            {
                printTime();
                Serial.println("Published to " + String(MQTT_PUBLISHTO_TOPIC) + " as\n" + output);
            }

            lastSend = pdTICKS_TO_MS(xTaskGetTickCount());
            forceSend = false;
            modem.sleepEnable(true);
            if (debug)
            {
                printTime();
                Serial.println("Modem is going to sleep");
            }
        }
        smartDelay(3000);
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
