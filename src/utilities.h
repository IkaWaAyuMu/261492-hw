// Pins

# define STATUS_LED_RED     25
# define STATUS_LED_GREEN   26
# define STATUS_LED_BLUE    27

# define LED_IN1            22
# define LED_OUT1           21
# define LED_IN2            23
# define LED_OUT2           19

# define IN1                32
# define OUT1               36 //
# define IN2                34
# define OUT2               35

# define POT_IN1            15
# define POT_OUT1           14
# define POT_IN2            13
# define POT_OUT2           12

# define GPS_RX             4
# define GPS_TX             2 

// Client ID

# define CLIENT_ID          "33"

// NTP Settings

# define NTP_SERVER         "pool.ntp.org"

// MQTT Settings

# define MQTT_BROKER_PORT       1883
# define MQTT_PUBLISHTO_TOPIC   "bus"

// GPS Settings

# define GPS_BAUD           9600

// HtTP GET location

# define LOCATION_ROUTE_URL         "http://128.199.248.64:3000/location"

// Modem Settings

#define MODEM_BAUDRATE                      (115200)
#define MODEM_DTR_PIN                       (25)
#define MODEM_TX_PIN                        (26)
#define MODEM_RX_PIN                        (27)
// The modem boot pin needs to follow the startup sequence.
#define BOARD_PWRKEY_PIN                    (4)
#define BOARD_BAT_ADC_PIN                   (35)
// The modem power switch must be set to HIGH for the modem to supply power.
#define BOARD_POWERON_PIN                   (12)
#define MODEM_RING_PIN                      (33)
#define MODEM_RESET_PIN                     (5)
#define BOARD_MISO_PIN                      (2)
#define BOARD_MOSI_PIN                      (15)
#define BOARD_SCK_PIN                       (14)
#define BOARD_SD_CS_PIN                     (13)

#define MODEM_RESET_LEVEL                   HIGH
#define SerialAT                            Serial1