/*   Version: 1.5.3  |  License: LGPLv3  |  Author: JDWifWaf@gmail.com   */

#include "MHZ19.h"

//#define DEBUG_LOCAL
/*#####################-Initialization Functions-#####################*/

/*
 * Initializes some variables and read version string
 */
void MHZ19::begin(Stream &aSerial) {
    mySerial = &aSerial;
    mySerial->setTimeout(MHZ19_RESPONSE_TIMEOUT_MILLIS);

    // preset send array once
    CommandToSend[0] = 0xFF;
    CommandToSend[1] = 0x01;
    CommandToSend[3] = 0;
    CommandToSend[4] = 0;
    CommandToSend[6] = 0;
    CommandToSend[7] = 0;

    VersionString[4] = '\0';

#if defined(DEBUG_LOCAL)
    if (readCO2AndTemperature()) {
        printErrorCode(&Serial); // print error to standard Serial, not to MHZ19 connection :-)
    }
#else
    readCO2AndTemperature(); // Test connection with standard command
#endif

    this->readVersion();
}

/**
 * Sends the 9 byte command stream, and receives the 9 byte response into ReceivedResponse
 * 9 bytes at 9600 baud takes 10 ms.
 * Checks for checksum
 * Timeout of MHZ19_RESPONSE_TIMEOUT_MILLIS (500) is specified in begin();
 * @return true if error happened during response receiving
 */
bool MHZ19::processCommand(MHZ19_command_t aCommand, bool aDoNotWaitForResponse) {
// Flush response buffer
    while (mySerial->available()) {
        mySerial->read();
    }

    CommandToSend[2] = aCommand;

    /*
     * Datasheet: Checksum = NOT (Byte1+Byte2+Byte3+Byte4+Byte5+Byte6+Byte7))+1, Byte0 is start byte
     * Set checksum. We have constant Byte1 0x01 and Bytes4 to 7 are also zero.
     */
    CommandToSend[8] = (~(0x01 + aCommand + CommandToSend[3])) + 1;

    mySerial->write(CommandToSend, MHZ19_DATA_LEN); // Start sending
    mySerial->flush(); // wait to be sent

    if (!aDoNotWaitForResponse) {
        delay(12);
        return readResponse();
    }

    return false;
}

/**
 * Receives the 9 byte response into ReceivedResponse
 * Response starts 2 ms after end of request (23 ms for setAutocalibration())
 * Checks for checksum
 * Timeout of MHZ19_RESPONSE_TIMEOUT_MILLIS (500) is specified in begin();
 * @return true if error happened during response receiving
 */
bool MHZ19::readResponse() {
    /* response received, read buffer */
    if (mySerial->readBytes(ReceivedResponse, MHZ19_DATA_LEN) != MHZ19_DATA_LEN) {
        this->errorCode = RESULT_TIMEOUT;
#if defined(DEBUG_LOCAL)
        Serial.print(F("Timeout error. Available="));
        Serial.println(mySerial->available());
#endif
        return true;
    }

    uint8_t tChecksum = 0;
    for (uint_fast8_t i = 1; i < 8; i++) {
        tChecksum += ReceivedResponse[i];
    }
    tChecksum = ~tChecksum + 1;

#if defined(DEBUG_LOCAL)
    Serial.print(F(" Received cmd=0x"));
    Serial.print(ReceivedResponse[1], HEX);
    Serial.print(F("|"));
    printCommand((MHZ19_command_t)ReceivedResponse[1], &Serial);

    for (uint_fast8_t i = 2; i < 8; i += 2) {
        Serial.print(F("   0x"));
        Serial.print(ReceivedResponse[i], HEX);
        Serial.print(F(",0x"));
        Serial.print(ReceivedResponse[i + 1], HEX);
        Serial.print(F(" ="));
        Serial.print((uint16_t) (ReceivedResponse[i] << 8 | ReceivedResponse[i + 1])); // Unsigned decimal word
    }

    Serial.println();
#endif

    if (tChecksum != ReceivedResponse[8]) {
        this->errorCode = RESULT_CHECKSUM;
#if defined(DEBUG_LOCAL)
        Serial.print(F("Checksum error. Received="));
        Serial.print(ReceivedResponse[8]);
        Serial.print(F(" expected="));
        Serial.println(tChecksum);
#endif
        return true;
    }

#if defined(DEBUG_LOCAL)
    if (CommandToSend[2] != ReceivedResponse[1]) {
        Serial.print(F("Command mismatch error. Sent=0x"));
        Serial.print(CommandToSend[2], HEX);
        Serial.print(F(" received=0x"));
        Serial.println(ReceivedResponse[1], HEX);
        this->errorCode = RESULT_MATCH;
    }
#endif
    this->errorCode = RESULT_OK;
    return false;
}

void MHZ19::printErrorCode(Print *aSerial) {
    aSerial->print(F("Response error code="));
    aSerial->println(errorCode);
}

void MHZ19::printCommand(MHZ19_command_t aCommand, Print *aSerial) {
    switch (aCommand) {
    case GETABC:
        aSerial->print(F("getABC   "));
        break;
    case RAWCO2:
        aSerial->print(F("getRawCO2"));
        break;
    case CO2UNLIM:
        aSerial->print(F("getCO2Unl"));
        break;
    case CO2LIM:
        aSerial->print(F("getCO2   "));
        break;
    case GETRANGE:
        aSerial->print(F("getRange"));
        break;
    case GETCALPPM:
        aSerial->print(F("getMinPPM"));
        break;
    case GETFIRMWARE:
        aSerial->print(F("getFW    "));
        break;
    default:
        break;
    }

}

bool MHZ19::readCO2AndTemperature() {
    if (processCommand(CO2LIM)) { // 0x86
        return true;
    }
    this->CO2 = ReceivedResponse[2] << 8 | ReceivedResponse[3];
    this->Temperature = ReceivedResponse[4] - TEMP_ADJUST;
    this->ABCCounter = ReceivedResponse[6];
    return false;
}

/**
 * Fills TemperatureFloat, CO2Unmasked and MinimumLightADC
 * @return true if error happened during response receiving
 */
bool MHZ19::readCO2AndTemperatureRaw() {
    if (processCommand(CO2UNLIM)) { // 0x85
        return true;
    }
    this->TemperatureFloat = (ReceivedResponse[2] << 8 | ReceivedResponse[3]) / 100.0;
    this->CO2Unmasked = ReceivedResponse[4] << 8 | ReceivedResponse[5];
    this->MinimumLightADC = ReceivedResponse[6] << 8 | ReceivedResponse[7]; // Observed 1013 to 1044
    return false;
}

/**
 * Fills CO2RawADC
 * @return true if error happened during response receiving
 */
bool MHZ19::readCO2Raw() {
    if (processCommand(RAWCO2)) { // 0x84
        return true;
    }
    this->CO2RawADC = ReceivedResponse[2] << 8 | ReceivedResponse[3];
    this->Unknown1 = ReceivedResponse[4] << 8 | ReceivedResponse[5]; // Observed 0xA3C8 to 0xA438
    this->Unknown2 = ReceivedResponse[6] << 8 | ReceivedResponse[7]; // Observed 0x0B57 to 0x0BEB
    return false;
}

/**
 * fills VersionString
 * @return true if error happened during response receiving
 */
bool MHZ19::readVersion() {
    if (processCommand(GETFIRMWARE)) {
        return true;
    }
    for (uint_fast8_t i = 0; i < 4; i++) {
        this->VersionString[i] = char(this->ReceivedResponse[i + 2]);
    }
    this->VersionMajor = this->ReceivedResponse[3] - '0';
    return false;
}

/**
 * fills SensorRange
 * @return true if error happened during response receiving
 */
bool MHZ19::readRange() {
    if (processCommand(GETRANGE)) {
        return true;
    }
    this->SensorRange = ReceivedResponse[4] << 8 | ReceivedResponse[5];
    return false;
}

// TODO or better getBackgroundCO2 ???
bool MHZ19::readBackgroundCO2() {
    if (processCommand(GETCALPPM)) {
        return true;
    }
    this->BackgroundCO2 = ReceivedResponse[4] << 8 | ReceivedResponse[5];
    return false;
}

/*
 * @return true if ABC enabled
 */
bool MHZ19::readABC() {
    if (processCommand(GETABC)) { // 0x7D
        return true;
    }
    this->AutoBaselineCorrectionEnabled = this->ReceivedResponse[7]; // (1 - enabled, 0 - disabled)
    return false;
}

void MHZ19::setAutoCalibration(bool aSwitchOn) {
    if (aSwitchOn) {
        this->CommandToSend[3] = 0xA0; // set parameter to command buffer
    }
    processCommand(ABC); // 0x79
    this->CommandToSend[3] = 0x00; // clear parameter from command buffer
}

/*########################-Set Functions-##########################*/

void MHZ19::setRange(int range) {
    if (range < 500 || range > 20000) {
#if defined (ESP32) && (MHZ19_ERRORS)
        ESP_LOGE(TAG_MHZ19, "Invalid Range value (500 - 20000)");
#elif MHZ19_ERRORS
        Serial.println("!ERROR: Invalid Range value (500 - 20000)");
#endif

        return;
    }

    else
        provisioning(RANGE, range);
}

void MHZ19::zeroSpan(int span) {
    if (span > 10000) {
#if defined (ESP32) && (MHZ19_ERRORS)
            ESP_LOGE(TAG_MHZ19, "Invalid Span value (0 - 10000)");
#elif MHZ19_ERRORS
        Serial.println("!ERROR: Invalid Span value (0 - 10000)");
#endif
    } else
        provisioning(SPANCAL, span);

    return;
}

void MHZ19::setFilter(bool isON, bool isCleared) {
    this->storage.settings.filterMode = isON;
    this->storage.settings.filterCleared = isCleared;
}

/*########################-Get Functions-##########################*/

unsigned int MHZ19::getCO2(bool isunLimited, bool force) {
    if (force == true) {
        if (isunLimited) {
            provisioning(CO2UNLIM);
        } else {
            provisioning(CO2LIM);
        }
    }

    if (this->errorCode == RESULT_OK || force == false) {
        if (!this->storage.settings.filterMode) {

            if (isunLimited) {
                this->CO2Unmasked = storage.responses.CO2UNLIM[4] << 8 | storage.responses.CO2UNLIM[5];
                return this->CO2Unmasked;
            } else {
                this->CO2 = storage.responses.CO2LIM[2] << 8 | storage.responses.CO2LIM[3];
                return this->CO2;
            }

        } else {
            /* FILTER BEGIN ----------------------------------------------------------- */
            unsigned int checkVal[2];
            bool trigFilter = false;

            // Filter must call the opposest unlimited/limited command to work
            if (!isunLimited)
                provisioning(CO2UNLIM);
            else
                provisioning(CO2LIM);

            checkVal[0] = makeInt(this->storage.responses.CO2UNLIM[4], this->storage.responses.CO2UNLIM[5]);
            checkVal[1] = makeInt(this->storage.responses.CO2LIM[2], this->storage.responses.CO2LIM[3]);

            // Limited CO2 stays at 410ppm during reset, so comparing unlimited which instead
            // shows an abnormal value, reset duration can be found. Limited CO2 ppm returns to "normal"
            // after reset.

            if (this->storage.settings.filterCleared) {
                if (checkVal[0] > 32767 || checkVal[1] > 32767 || (((checkVal[0] - checkVal[1]) >= 10) && checkVal[1] == 410)) {
                    this->errorCode = RESULT_FILTER;
                    return 0;
                }
            } else {
                if (checkVal[0] > 32767) {
                    checkVal[0] = 32767;
                    trigFilter = true;
                }
                if (checkVal[1] > 32767) {
                    checkVal[1] = 32767;
                    trigFilter = true;
                }
                if (((checkVal[0] - checkVal[1]) >= 10) && checkVal[1] == 410)
                    trigFilter = true;

                if (trigFilter) {
                    this->errorCode = RESULT_FILTER;
                }
            }

            if (isunLimited)
                return checkVal[0];
            else
                return checkVal[1];
            /* FILTER END ----------------------------------------------------------- */
        }
    }
    return 0;
}

unsigned int MHZ19::getCO2Raw(bool force) {
    if (force == true) {
        provisioning(RAWCO2);
    }
    if (this->errorCode == RESULT_OK || force == false) {
        return makeInt(this->storage.responses.RAW[2], this->storage.responses.RAW[3]);
    }
    return 0;
}

float MHZ19::getTransmittance(bool force) {
    if (force == true)
        provisioning(RAWCO2);

    if (this->errorCode == RESULT_OK || force == false) {
        float calc = (float) makeInt((this->storage.responses.RAW[2]), this->storage.responses.RAW[3]);

        return (calc * 100 / 35000); //  (calc * to percent / x(raw) zero)
    }

    else
        return 0;
}

float MHZ19::getTemperature(bool force) {
    if (this->storage.settings.fw_ver < 5) {
        if (force == true)
            provisioning(CO2LIM);

        if (this->errorCode == RESULT_OK || force == false)
            return (this->storage.responses.CO2LIM[4] - TEMP_ADJUST);
    } else {
        if (force == true)
            provisioning(CO2UNLIM);

        if (this->errorCode == RESULT_OK)
            return (float) (((int) this->storage.responses.CO2UNLIM[2] << 8) | this->storage.responses.CO2UNLIM[3]) / 100;
    }

    return -273.15;
}

unsigned int MHZ19::getRange() {
    /* check get range was received */
    provisioning(GETRANGE);

    if (this->errorCode == RESULT_OK) {
        /* convert MH-Z19 memory value and return */
        return (int) makeInt(this->storage.responses.STAT[4], this->storage.responses.STAT[5]);
    }

    return 0;
}

byte MHZ19::getAccuracy(bool force) {
    if (force == true)
        provisioning(CO2LIM);

    if (this->errorCode == RESULT_OK || force == false)
        return this->storage.responses.CO2LIM[5];

    else
        return 0;

//GetRange byte 7
}

byte MHZ19::getPWMStatus() {
//255 156 byte 4;
    return 0;
}

void MHZ19::getVersion(char rVersion[]) {
    provisioning(GETFIRMWARE);

    if (this->errorCode == RESULT_OK)
        for (byte i = 0; i < 4; i++) {
            rVersion[i] = char(this->storage.responses.STAT[i + 2]);
        }

    else
        memset(rVersion, 0, 4);
}

unsigned int MHZ19::getBackgroundCO2() {
    processCommand(GETCALPPM);

    if (this->errorCode == RESULT_OK) {
        return (int) makeInt(this->ReceivedResponse[4], this->ReceivedResponse[5]);
    }
    return 0;
}

byte MHZ19::getTempAdjustment() {
    provisioning(GETEMPCAL);

    /* Constant 40 is returned here, however this library uses TEMP_ADJUST
     when using temperature function as it appears inaccurate,
     */

    if (this->errorCode == RESULT_OK) {
        return (this->storage.responses.STAT[3]);
    }
    return 0;
}

byte MHZ19::getLastResponse(byte bytenum) {
    provisioning(GETLASTRESP);

    if (this->errorCode == RESULT_OK)
        return (this->storage.responses.STAT[bytenum]);

    else
        return 0;
}


bool MHZ19::getABC()
{
    /* check get ABC logic status (1 - enabled, 0 - disabled) */
    provisioning(GETABC);

    if (this->errorCode == RESULT_OK)
        /* convert MH-Z19 memory value and return */
        return this->storage.responses.STAT[7];
    else
        return 1;
}

/*######################-Utility Functions-########################*/

void MHZ19::verify() {
    unsigned long timeStamp = millis();

    /* construct common command (133) */
    constructCommand(CO2UNLIM);

    write(this->storage.constructedCommand);

    while (read(this->storage.responses.CO2UNLIM) != RESULT_OK) {
        if (millis() - timeStamp >= MHZ19_RESPONSE_TIMEOUT_MILLIS) {
#if defined (ESP32) && (MHZ19_ERRORS)
                ESP_LOGE(TAG_MHZ19, "Failed to verify connection(1) to sensor.");
#elif MHZ19_ERRORS
            Serial.println("!ERROR: Failed to verify connection(1) to sensor.");
#endif

            return;
        }
    }

    /* construct & write last response command (162) */
    constructCommand(GETLASTRESP);
    write(this->storage.constructedCommand);

    /* update timeStamp  for next comms iteration */
    timeStamp = millis();

    while (read(this->storage.responses.STAT) != RESULT_OK) {
        if (millis() - timeStamp >= MHZ19_RESPONSE_TIMEOUT_MILLIS) {
#if defined (ESP32) && (MHZ19_ERRORS)
                ESP_LOGE(TAG_MHZ19, "Failed to verify connection(2) to sensor.");
#elif MHZ19_ERRORS
            Serial.println("!ERROR: Failed to verify connection(2) to sensor.");
#endif

            return;
        }
    }

    /* compare CO2 & temp bytes, command(133), against last response bytes, command (162)*/
    for (byte i = 2; i < 6; i++) {
        if (this->storage.responses.CO2UNLIM[i] != this->storage.responses.STAT[i]) {
#if defined (ESP32) && (MHZ19_ERRORS)
                ESP_LOGE(TAG_MHZ19, "Last response is not as expected, verification failed.");
#elif MHZ19_ERRORS
            Serial.println("!ERROR: Last response is not as expected, verification failed.");
#endif

            return;
        }
    }
    return;
}

void MHZ19::autoCalibration(bool isON, byte ABCPeriod) {
    /* If ABC is ON */
    if (isON) {
        /* If a period was defined */
        if (ABCPeriod) {
            /* Catch values out of range */
            if (ABCPeriod >= 24)
                ABCPeriod = 24;

            /* Convert to bytes */
            ABCPeriod *= 6.7;
        }
        /* If no period was defined (for safety, even though default argument is given)*/
        else
            ABCPeriod = MHZ19_ABC_PERIOD_DEF;    // Default bytes
    }
    /* If ABC is OFF */
    else
        ABCPeriod = MHZ19_ABC_PERIOD_OFF;                      // Set command byte to Zero to match command format.

    /* Update storage */
    this->storage.settings.ABCRepeat = !isON;  // Set to opposite, as repeat command is sent only when ABC is OFF.

    provisioning(ABC, ABCPeriod);
}

void MHZ19::calibrate() {
    provisioning(ZEROCAL);
}

void MHZ19::recoveryReset() {
    provisioning(RECOVER);
}

void MHZ19::printCommunication(bool isDec, bool isPrintComm) {
    this->storage.settings._isDec = isDec;
    this->storage.settings.printcomm = isPrintComm;
}

/*######################-Internal Functions-########################*/

void MHZ19::provisioning(MHZ19_command_t commandtype, int inData) {
    /* construct command */
    constructCommand(commandtype, inData);

    /* write to serial */
    write(this->storage.constructedCommand);

    /*return response */
    handleResponse();

    /* Check if ABC_OFF needs to run */
    ABCCheck();
}

void MHZ19::constructCommand(MHZ19_command_t commandtype, int inData) {
    /* values for conversions */
    byte High;
    byte Low;

    /* Temporary holder */
    byte asemblecommand[MHZ19_DATA_LEN];

    /* prepare arrays */
    memset(asemblecommand, 0, MHZ19_DATA_LEN);
    memset(this->storage.constructedCommand, 0, MHZ19_DATA_LEN);

    /* set address to 'any' */
    asemblecommand[0] = 0xFF; // Start byte

    /* set  register */
    asemblecommand[1] = 1; // Sensor #

    /* set command */
    asemblecommand[2] = commandtype; // Command

    switch (commandtype) {
    case RECOVER:
        break;
    case ABC:
        if (this->storage.settings.ABCRepeat == false)
            asemblecommand[3] = inData;
        break;
    case RAWCO2:
        break;
    case CO2UNLIM:
        break;
    case CO2LIM:
        break;
    case ZEROCAL:
        if (inData)
            asemblecommand[6] = inData;
        break;
    case SPANCAL:
        makeByte(inData, &High, &Low);
        asemblecommand[3] = High;
        asemblecommand[4] = Low;
        break;
    case RANGE:
        makeByte(inData, &High, &Low);
        asemblecommand[6] = High;
        asemblecommand[7] = Low;
        break;
    case GETRANGE:
        break;
    case GETCALPPM:
        break;
    case GETFIRMWARE:
        break;
    case GETEMPCAL:
        break;
    case GETLASTRESP:
        break;
    case GETABC:
        break;
    }

    /* set checksum */
    asemblecommand[8] = getCRC(asemblecommand);

    /* copy bytes from asemblecommand to constructedCommand */
    memcpy(this->storage.constructedCommand, asemblecommand, MHZ19_DATA_LEN);
}

void MHZ19::write(byte toSend[]) {
    /* for print communications */
    if (this->storage.settings.printcomm == true)
        printstream(toSend, true, this->errorCode);

    /* transfer to buffer */
    mySerial->write(toSend, MHZ19_DATA_LEN);

    /* send */
    mySerial->flush();
}

byte MHZ19::read(byte inBytes[MHZ19_DATA_LEN]) {
    /* loop escape */
    unsigned long timeStamp = millis();

    /* prepare memory array with unsigned chars of 0 */
    memset(inBytes, 0, MHZ19_DATA_LEN);

    /* prepare errorCode */
    this->errorCode = RESULT_NULL;

    /* wait until we have exactly the 9 bytes reply (certain controllers call read() too fast) */
    while (mySerial->available() < MHZ19_DATA_LEN) {
        if (millis() - timeStamp >= MHZ19_RESPONSE_TIMEOUT_MILLIS) {
#if defined (ESP32) && (MHZ19_ERRORS)
                ESP_LOGW(TAG_MHZ19, "Timed out waiting for response");
#elif MHZ19_ERRORS
            Serial.println("!Error: Timed out waiting for response");
#endif

            this->errorCode = RESULT_TIMEOUT;

            /* clear incomplete 9 byte values, limit is finite */
            cleanUp(mySerial->available());

            //return error condition
            return RESULT_TIMEOUT;
        }
    }

    /* response received, read buffer */
    mySerial->readBytes(inBytes, MHZ19_DATA_LEN);

    if (this->errorCode == RESULT_TIMEOUT)
        return this->errorCode;

    byte crc = getCRC(inBytes);

    /* CRC error will not override match error */
    if (inBytes[8] != crc)
        this->errorCode = RESULT_CHECKSUM;

    /* construct error code */
    if (inBytes[0] != this->storage.constructedCommand[0] || inBytes[1] != this->storage.constructedCommand[2]) {
        /* clear rx buffer for desync correction */
        cleanUp(mySerial->available());
        this->errorCode = RESULT_MATCH;
    }

    /* if error has been assigned */
    if (this->errorCode == RESULT_NULL)
        this->errorCode = RESULT_OK;

    /* print results */
    if (this->storage.settings.printcomm == true)
        printstream(inBytes, false, this->errorCode);

    return this->errorCode;
}

void MHZ19::cleanUp(uint8_t cnt) {
    for (uint8_t x = 0; x < cnt; x++) {
#if (MHZ19_ERRORS) // to avoid nasty ESP32 compiler error.
        uint8_t eject = mySerial->read();
#else
            mySerial->read();
#endif
#if defined (ESP32) && (MHZ19_ERRORS)
            ESP_LOGW(TAG_MHZ19, "Clearing Byte: %d", eject);
#elif MHZ19_ERRORS
        Serial.print("!Warning: Clearing Byte: ");
        Serial.println(eject);
#endif
    }
}

void MHZ19::handleResponse() {
    if (this->storage.constructedCommand[2] == RAWCO2) {    // compare commands byte
        read(this->storage.responses.RAW);    // returns error number, passes back response and inputs command
    } else if (this->storage.constructedCommand[2] == CO2UNLIM) {
        read(this->storage.responses.CO2UNLIM);
    } else if (this->storage.constructedCommand[2] == CO2LIM) {
        read(this->storage.responses.CO2LIM);
    } else {
        read(this->storage.responses.STAT);
    }
}

void MHZ19::printstream(byte inBytes[MHZ19_DATA_LEN], bool isSent, byte pserrorCode) {
    if (pserrorCode != RESULT_OK && isSent == false) {
        Serial.print("Received >> ");
        if (this->storage.settings._isDec) {
            Serial.print("DEC: ");
            for (uint8_t i = 0; i < MHZ19_DATA_LEN; i++) {
                Serial.print(inBytes[i]);
                Serial.print(" ");
            }
        } else {
            for (uint8_t i = 0; i < MHZ19_DATA_LEN; i++) {
                Serial.print("0x");
                if (inBytes[i] < 16)
                    Serial.print("0");
                Serial.print(inBytes[i], HEX);
                Serial.print(" ");
            }
        }
        Serial.print("ERROR Code: ");
        Serial.println(pserrorCode);
    }

    else {
        isSent ? Serial.print("Sent << ") : Serial.print("Received >> ");

        if (this->storage.settings._isDec) {
            Serial.print("DEC: ");
            for (uint8_t i = 0; i < MHZ19_DATA_LEN; i++) {
                Serial.print(inBytes[i]);
                Serial.print(" ");
            }
        } else {
            for (uint8_t i = 0; i < MHZ19_DATA_LEN; i++) {
                Serial.print("0x");
                if (inBytes[i] < 16)
                    Serial.print("0");
                Serial.print(inBytes[i], HEX);
                Serial.print(" ");
            }
        }
        Serial.println(" ");
    }
}

byte MHZ19::getCRC(byte inBytes[]) {
    /* as shown in datasheet */
    byte x = 0, crc = 0;

    for (x = 1; x < 8; x++) {
        crc += inBytes[x];
    }

    crc = 255 - crc;
    crc++;

    return crc;
}

void MHZ19::ABCCheck() {
    /* check timer interval if dynamic hours have passed and if ABC_OFF was set to true */
    if (((millis() - ABCRepeatTimer) >= 4.32e7) && (this->storage.settings.ABCRepeat == true)) {
        /* update timer inerval */
        ABCRepeatTimer = millis();

        /* construct command to skip next ABC cycle */
        provisioning(ABC, MHZ19_ABC_PERIOD_OFF);
    }
}

void MHZ19::makeByte(int inInt, byte *high, byte *low) {
    *high = (byte) (inInt / 256);
    *low = (byte) (inInt % 256);

    return;
}

unsigned int MHZ19::makeInt(byte high, byte low) {
    unsigned int calc = ((unsigned int) high * 256) + (unsigned int) low;

    return calc;
}
