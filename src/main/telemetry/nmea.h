/*
 * nmea.h
 *
 *  Created on: 14 December 2016
 *      Author: lkaino
 */

#ifndef TELEMETRY_NMEA_H_
#define TELEMETRY_NMEA_H_

void initNMEATelemetry(telemetryConfig_t *);

void handleNMEATelemetry(void);
void checkNMEATelemetryState(void);

void configureNMEATelemetryPort(void);
void freeNMEATelemetryPort(void);

bool isNMEATimedOut(void);

#endif /* TELEMETRY_NMEA_H_ */
