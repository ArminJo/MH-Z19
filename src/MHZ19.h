/*   Version: 1.5.3  |  License: LGPLv3  |  Author: JDWifWaf@gmail.com   */

#ifndef MHZ19_H
#define MHZ19_H

#include <Arduino.h>

#ifdef ESP32
#include "esp32-hal-log.h"
#define TAG_MHZ19 "MH-Z19"
#endif

#define MHZ19_ERRORS 1                  // Set to 0 to disable error prints
#define TEMP_ADJUST 40                  // This is the value used to adjust the temperature.
#define MHZ19_RESPONSE_TIMEOUT_MILLIS 500              // Time out period for response (ms)
#define DEFAULT_RANGE 2000              // For range function (sensor works best in this range)
#define MHZ19_DATA_LEN 9                // Data protocol length

// Command bytes -------------------------- //
#define MHZ19_ABC_PERIOD_OFF    0x00
#define MHZ19_ABC_PERIOD_DEF    0xA0

/* enum alias for error code definitions */
enum ERRORCODE {
    RESULT_NULL = 0, RESULT_OK = 1, RESULT_TIMEOUT = 2, RESULT_MATCH = 3, RESULT_CHECKSUM = 4, RESULT_FILTER = 5
};

class MHZ19 {
public:
    /*###########################-Variables-##########################*/

    /* enum for command types */
    typedef enum COMMAND_TYPE {
        RECOVER = 0x78,     // Recovery Reset - Changes operation mode and performs MCU reset
        ABC = 0x79,         // Turns ABC (Automatic Baseline Correction) logic on or off (b[3] == 0xA0 - on, 0x00 - off)
        GETABC = 0x7D,      // Get ABC logic status  (1 - enabled, 0 - disabled)
        RAWCO2 = 0x84,      // Raw CO2 ADC value
        CO2UNLIM = 0x85,    // Smoothed temperature ADC value, CO2 level
        CO2LIM = 0x86,      // CO2 masked at 500 in the first minute, Temperature integer
        ZEROCAL = 0x87,     // Zero Calibration
        SPANCAL = 0x88,     // Span Calibration
        RANGE = 0x99,       // Sets sensor range. Note that parameter bytes differ from those in the datasheet.
        GETRANGE = 0x9B,    // Get Range
        GETCALPPM = 0x9C,   // Get Background CO2 / lowest ppm value to show
        GETFIRMWARE = 0xA0, // Get Firmware Version
        GETLASTRESP = 0xA2, // Get Last Response
        GETEMPCAL = 0xA3    // Get temperature offset - returns constant 40
    } MHZ19_command_t;

    /* pointer for Stream class to accept reference for hardware and software ports */
    Stream *mySerial;

    /* Holds last received error code from recieveResponse() */
    byte errorCode;

    /* for keeping track of the ABC run interval */
    unsigned long ABCRepeatTimer;
    char VersionString[5];
    uint8_t VersionMajor;

    uint16_t CO2RawADC;
    uint16_t Unknown1;
    uint16_t Unknown2;

    uint16_t CO2; // Is displayed / masked as 500 for the first minute. Values are clipped between 405 and 5000.
    int8_t Temperature;
    uint8_t ABCCounter; // Is incremented by MHZ19 every 10 minutes

    float TemperatureFloat;
    uint16_t CO2Unmasked; // Is displayed even at first minute. Values are also clipped between 405 and 5000.
    uint16_t MinimumLightADC;

    uint16_t SensorRange;
    bool AutoBaselineCorrectionEnabled;
    uint16_t BackgroundCO2;


    uint8_t CommandToSend[MHZ19_DATA_LEN];      // Array for commands to be sent
    uint8_t ReceivedResponse[MHZ19_DATA_LEN];   // Array for response

    bool processCommand(MHZ19_command_t aCommand, bool aDoNotWaitForResponse = false);
    bool readResponse();
    bool readCO2AndTemperature();
    bool readCO2AndTemperatureRaw();
    bool readVersion();
    bool readABC(); // Reads ABC-Status using command 125 / 0x7D
    bool readRange();
    bool readBackgroundCO2();
    bool readCO2Raw();
//    bool readRange();



    void setAutoCalibration(bool aSwitchOn);

    void printErrorCode(Print *aSerial);
    void printCommand(MHZ19_command_t aCommand, Print *aSerial);

    /*#####################-Initiation Functions-#####################*/

    /* essential begin */
    void begin(Stream &aStream);

    /*########################-Set Functions-##########################*/

    /* Sets Range to desired value*/
    void setRange(int range = 2000);

    /* Sets Span to desired value below 10,000*/
    void zeroSpan(int span = 2000);

    /* Sets "filter mode" to ON or OFF & mode type (see example) */
    void setFilter(bool isON = true, bool isCleared = true);

    /*########################-Get Functions-##########################*/

    /* request CO2 values, 2 types of CO2 can be returned, isLimted = true (command 134) and is Limited = false (command 133) */
    unsigned int getCO2(bool isunLimited = true, bool force = true);

    /* returns the "raw" CO2 value of unknown units */
    unsigned int getCO2Raw(bool force = true);

    /* returns Raw CO2 value as a % of transmittance */         //<--- needs work to understand
    float getTransmittance(bool force = true);

    /*  returns temperature using command 133 or 134 */
    float getTemperature(bool force = true);

    /* reads range using command 153 */
    unsigned int getRange();

    /* reads ABC-Status using command 125 / 0x7D */
    bool getABC();

    /* Returns accuracy value if available */
    byte getAccuracy(bool force = true);

    /* not yet implemented */
    byte getPWMStatus();

    /* returns MH-Z19 version using command 160, to the entered array */
    void getVersion(char rVersion[]);

    /* returns background CO2 used by sensor using command 156 */
    unsigned int getBackgroundCO2();

    /* returns temperature using command 163 (Note: this library deducts -2 when the value is used) */
    byte getTempAdjustment();

    /* returns last recorded response from device using command 162 */
    byte getLastResponse(byte bytenum);

    /*######################-Utility Functions-########################*/

    /* ensure communication is working (included in begin())*/
    void verify();

    /* disables calibration or sets ABCPeriod */
    void autoCalibration(bool isON = true, byte ABCPeriod = 24);

    /* Calibrates "Zero" (Note: Zero refers to 400ppm for this sensor)*/
    void calibrate();

    /*  Calibrate Backwards compatibility */
    void inline calibrateZero() {
        calibrate();
    }

    /* requests a reset */
    void recoveryReset();

    /* use to show communication between MHZ19 and  Device */
    void printCommunication(bool isDec = true, bool isPrintComm = true);

private:
    /*###########################-Variables-##########################*/

    /* Memory Pool */
    struct mempool {
        struct config {
            bool ABCRepeat = false;     // A flag which represents whether auto calibration ABC period was checked
            bool filterMode = false;    // Flag set by setFilter() to signify is "filter mode" was made active
            bool filterCleared = true;  // Additional flag set by setFilter() to store which mode was selected
            bool printcomm = false;     // Communication print options
            bool _isDec = true;         // Holds preference for communication printing
            uint8_t fw_ver = 0;         // Holds the major version of the firmware
        } settings;

        byte constructedCommand[MHZ19_DATA_LEN];    // holder for new commands which are to be sent

        struct indata {
            byte CO2UNLIM[MHZ19_DATA_LEN];  // Holds command 133 response values "CO2 unlimited and temperature for unsigned"
            byte CO2LIM[MHZ19_DATA_LEN];    // Holds command 134 response values "CO2 limited and temperature for signed"
            byte RAW[MHZ19_DATA_LEN];       // Holds command 132 response values "CO2 Raw"
            byte STAT[MHZ19_DATA_LEN];      // Holds other command response values such as range, background CO2 etc
        } responses;

    } storage;

    /*######################-Internal Functions-########################*/

    /* Coordinates  sending, constructing and receiving commands */
    void provisioning(MHZ19_command_t commandtype, int inData = 0);

    /* Constructs commands using command array and entered values */
    void constructCommand(MHZ19_command_t commandtype, int inData = 0);

    /* generates a checksum for sending and verifying incoming data */
    byte getCRC(byte inBytes[]);

    /* Sends commands to the sensor */
    void write(byte toSend[]);

    /* Call retrieveData to retrieve values from the sensor and check return code */
    byte read(byte inBytes[9]);

    /* Assigns response to the correct communication arrays */
    void handleResponse();

    /* prints sending / receiving messages if enabled */
    void printstream(byte inbytes[9], bool isSent, byte pserrorCode);

    /* Checks whether time elapse for next ABC OFF cycle has occurred */
    void ABCCheck();

    /* converts integers to bytes according to /256 and %256 */
    void makeByte(int inInt, byte *high, byte *low);

    /* converts bytes to integers according to *256 and + value */
    unsigned int makeInt(byte high, byte low);

    void cleanUp(uint8_t cnt);
};
#endif
