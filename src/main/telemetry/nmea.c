/*
 * NMEA Telemetry implementation by frank26080115
 * see https://github.com/frank26080115/cleanflight/wiki/Using-Smart-Port
 */
#include <stdio.h>
#include <stdbool.h>
#include <stdint.h>
#include <stdlib.h>
#include <math.h>

#include "platform.h"

#if defined(TELEMETRY) && defined(TELEMETRY_NMEA)

#include "common/axis.h"
#include "common/color.h"
#include "common/maths.h"

#include "drivers/system.h"
#include "drivers/serial.h"

#include "rx/rx.h"
#include "rx/msp.h"

#include "fc/rc_controls.h"
#include "fc/runtime_config.h"

#include "io/gps.h"
#include "io/serial.h"
#include "io/ledstrip.h"

#include "sensors/acceleration.h"
#include "sensors/battery.h"
#include "sensors/barometer.h"
#include "sensors/pitotmeter.h"

#include "flight/imu.h"
#include "flight/navigation_rewrite.h"

#include "telemetry/telemetry.h"
#include "telemetry/nmea.h"

#include "config/config.h"
#include "config/feature.h"

#include "build/debug.h"

enum
{
    NMEASTATE_UNINITIALIZED,
    NMEASTATE_INITIALIZED,
    NMEASTATE_WORKING,
    NMEASTATE_TIMEDOUT,
};

#define NMEA_BAUD 57600
#define NMEA_UART_MODE MODE_TX
#define NMEA_SERVICE_TIMEOUT_MS 1 // max allowed time to find a value to send
#define NMEA_NOT_CONNECTED_TIMEOUT_MS 7000

#define NMEA_GPS_SENTENCE_MAX_LENGTH 100

static serialPort_t *gpNMEASerialPort = NULL; // The 'NMEA'(tm) Port.
static serialPortConfig_t *portConfig;

static telemetryConfig_t *telemetryConfig;
static bool NMEATelemetryEnabled =  false;
static portSharing_e NMEAPortSharing;

char NMEAState = NMEASTATE_UNINITIALIZED;
static uint32_t NMEALastRequestTime = 0;

uint8_t gGPSSentence[NMEA_GPS_SENTENCE_MAX_LENGTH];

static void GPStoDDDMM_MMMM(int32_t mwiigps, gpsCoordinateDDDMMmmmm_t *result);
static uint8_t createAttitudeSentence(uint8_t **ppSentence);
static uint8_t createGPSGLLSentence(uint8_t **ppSentence);
static uint8_t createGPSGGASentence(uint8_t **ppSentence);
static uint8_t calculateChecksum(uint8_t *pSentence, u8 len);

void initNMEATelemetry(telemetryConfig_t *initialTelemetryConfig)
{
    telemetryConfig = initialTelemetryConfig;
    portConfig = findSerialPortConfig(FUNCTION_TELEMETRY_NMEA);
    NMEAPortSharing = determinePortSharing(portConfig, FUNCTION_TELEMETRY_NMEA);
    debug[0] = 1;
}

void freeNMEATelemetryPort(void)
{
    closeSerialPort(gpNMEASerialPort);
    gpNMEASerialPort = NULL;

    NMEAState = NMEASTATE_UNINITIALIZED;
    NMEATelemetryEnabled = false;
}

void configureNMEATelemetryPort(void)
{
    portOptions_t portOptions;

    debug[1] = 1;
    if (!portConfig) {
        return;
    }
    debug[1] = 2;

    portOptions = SERIAL_UNIDIR;

    gpNMEASerialPort = openSerialPort(portConfig->identifier, FUNCTION_TELEMETRY_NMEA, NULL, NMEA_BAUD, NMEA_UART_MODE, portOptions);

    if (!gpNMEASerialPort) {
        return;
    }
    debug[1] = 3;

    NMEAState = NMEASTATE_INITIALIZED;
    NMEATelemetryEnabled = true;
    NMEALastRequestTime = millis();


}

bool canSendNMEATelemetry(void)
{
    return gpNMEASerialPort && (NMEAState == NMEASTATE_INITIALIZED || NMEAState == NMEASTATE_WORKING);
}

bool isNMEATimedOut(void)
{
    return NMEAState >= NMEASTATE_TIMEDOUT;
}

void checkNMEATelemetryState(void)
{
    bool newTelemetryEnabledValue = telemetryDetermineEnabledState(NMEAPortSharing);

    if (newTelemetryEnabledValue == NMEATelemetryEnabled) {
        return;
    }

    if (newTelemetryEnabledValue)
        configureNMEATelemetryPort();
    else
        freeNMEATelemetryPort();
}

static uint8_t calculateChecksum(uint8_t *pSentence, u8 len)
{
    uint8_t crc = 0;
    uint32_t i;

    for (i = 0; i < len; i++)
    {
        crc ^= pSentence[i];
    }

    return crc;
}

static uint8_t createAttitudeSentence(uint8_t **ppSentence)
{
    uint8_t len = 0;
    int ret = sprintf((char *)gGPSSentence,
                       "$ATT,"
                       "%d.%d,"
                       "%d.%d,"
                       "%d.%d,"
                       "%lu,"
                       "*",
                       (attitude.values.pitch / 10), (abs(attitude.values.pitch) % 10),
                       (attitude.values.roll / 10), (abs(attitude.values.roll) % 10),
                       (attitude.values.yaw / 10), (abs(attitude.values.yaw) % 10),
                       (uint32_t)lrintf(getEstimatedActualPosition(Z))
                       );

    if (ret < 0)
    {
        len = 0;
    }
    else
    {
        len = (uint8_t)ret;

        uint8_t crc = calculateChecksum(gGPSSentence, len);

        ret = sprintf((char *)&gGPSSentence[len],
                       "%d\r\n",
                       crc);

        if (ret < 0)
        {
            len = 0;
        }
        else
        {
            len += (uint8_t)ret;
        }
    }

    *ppSentence = gGPSSentence;
    return len;
}

static uint8_t createGPSGLLSentence(uint8_t **ppSentence)
{
    uint8_t len = 0;
    gpsCoordinateDDDMMmmmm_t latCoords;
    gpsCoordinateDDDMMmmmm_t lonCoords;
    GPStoDDDMM_MMMM(gpsSol.llh.lat, &latCoords);
    GPStoDDDMM_MMMM(gpsSol.llh.lon, &lonCoords);
    latCoords.dddmm = 10023;
    latCoords.mmmm = 1234;
    lonCoords.dddmm = 56789;
    lonCoords.mmmm = 4321;
    int ret = sprintf((char *)gGPSSentence,
                       "$GPGLL,%d.%4d,N,%d.%4d,W,0,A,*",
                       latCoords.dddmm,
                       latCoords.mmmm,
                       lonCoords.dddmm,
                       lonCoords.mmmm);

    if (ret < 0)
    {
        len = 0;
    }
    else
    {
        len = (uint8_t)ret;

        uint8_t crc = calculateChecksum(gGPSSentence, len);

        ret = sprintf((char *)&gGPSSentence[len],
                       "%d\r\n",
                       crc);

        if (ret < 0)
        {
            len = 0;
        }
        else
        {
            len += (uint8_t)ret;
        }
    }

    *ppSentence = gGPSSentence;
    return len;
}

static uint8_t createGPSGGASentence(uint8_t **ppSentence)
{
    uint8_t len = 0;
    gpsCoordinateDDDMMmmmm_t latCoords;
    gpsCoordinateDDDMMmmmm_t lonCoords;
    GPStoDDDMM_MMMM(gpsSol.llh.lat, &latCoords);
    GPStoDDDMM_MMMM(gpsSol.llh.lon, &lonCoords);
    int ret = sprintf((char *) gGPSSentence,
                      "$GPGGA,"
                      "0,"          // 1: time
                      "%d.%4d,N,"   // 2: latitude
                      "%d.%4d,W,"   // 4: longitude
                      "%d,"         // 6: Fix quality, 0=invalid, 1=gps fix, 2=DGPS fix
                      "%02d,"       // 7: number of satellites
                      "%d.%d,"      // 8: HDOP
                      "%ld.%ld,M,"    // 9: Altitude
                      "0,M,"        // 11: Height of geoid above WGS84 ellipsoid (not supported)
                      "0,"          // 13: Time since last DGPS update (not supported)
                      "0,"          // 14: DGPS reference station id (not supported)
                      "*",
                      latCoords.dddmm,
                      latCoords.mmmm,
                      lonCoords.dddmm,
                      lonCoords.mmmm,
                      gpsSol.fixType,
                      gpsSol.numSat,
                      (gpsSol.hdop / 100), ((gpsSol.hdop % 100) / 10),
                      (gpsSol.llh.alt / 100), ((gpsSol.llh.alt % 100) / 10)
                      );

    if (ret < 0)
    {
        len = 0;
    }
    else
    {
        len = (uint8_t)ret;

        uint8_t crc = calculateChecksum(gGPSSentence, len);

        ret = sprintf((char *)&gGPSSentence[len],
                       "%d\r\n",
                       crc);

        if (ret < 0)
        {
            len = 0;
        }
        else
        {
            len += (uint8_t)ret;
        }
    }

    *ppSentence = gGPSSentence;
    return len;
}

void handleNMEATelemetry(void)
{
    static uint32_t NMEALastServiceTime = 0;


    if (!NMEATelemetryEnabled) {
        return;
    }

    if (!canSendNMEATelemetry()) {
        return;
    }

    uint32_t now = millis();

    if ((now - NMEALastServiceTime) > 50)
    {
        static uint8_t gpsCounter = 0;
        NMEALastServiceTime = now;
        uint8_t *pSentence;
        uint8_t len = 0;

        len = createAttitudeSentence(&pSentence);
        serialWriteBuf(gpNMEASerialPort, pSentence, len);

        if (gpsCounter >= 19)
        {
            gpsCounter = 0;
            len = createGPSGLLSentence(&pSentence);
            serialWriteBuf(gpNMEASerialPort, pSentence, len);
        }
        gpsCounter++;
    }
}

static void GPStoDDDMM_MMMM(int32_t mwiigps, gpsCoordinateDDDMMmmmm_t *result)
{
    int32_t absgps, deg, min;
    absgps = ABS(mwiigps);
    deg    = absgps / GPS_DEGREES_DIVIDER;
    absgps = (absgps - deg * GPS_DEGREES_DIVIDER) * 60;        // absgps = Minutes left * 10^7
    min    = absgps / GPS_DEGREES_DIVIDER;                     // minutes left

    if (telemetryConfig->frsky_coordinate_format == FRSKY_FORMAT_DMS) {
        result->dddmm = deg * 100 + min;
    } else {
        result->dddmm = deg * 60 + min;
    }

    result->mmmm  = (absgps - min * GPS_DEGREES_DIVIDER) / 1000;
}

#endif
