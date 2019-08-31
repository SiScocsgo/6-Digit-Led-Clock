#include "gps.h"

#include <stdbool.h>
#include <stdint.h>
#include <ctype.h>
#include <string.h>
#include <math.h>

#define GPS_SV_MAXSATS   16

extern uint8_t gpschar;
extern uint32_t time;

bool GPS_FIX;


gpsSolutionData_t gpsSol;
uint32_t GPS_packetCount = 0;
uint32_t GPS_svInfoReceivedCount = 0; // SV = Space Vehicle, counter increments each time SV info is received.
uint8_t GPS_update = 0;             // toogle to distinct a GPS position update (directly or via MSP)

uint8_t GPS_numCh;                          // Number of channels
uint8_t GPS_svinfo_chn[GPS_SV_MAXSATS];     // Channel number
uint8_t GPS_svinfo_svid[GPS_SV_MAXSATS];    // Satellite ID
uint8_t GPS_svinfo_quality[GPS_SV_MAXSATS]; // Bitfield Qualtity
uint8_t GPS_svinfo_cno[GPS_SV_MAXSATS];     // Carrier to Noise Ratio (Signal Strength)

static bool gpsNewFrameNMEA(char c);

void gpsUpdate()
{
    gpsNewFrameNMEA(gpschar);
}

#define NO_FRAME   0
#define FRAME_GGA  1
#define FRAME_RMC  2
#define FRAME_GSV  3

// helper functions
static uint32_t grab_fields(char *src, uint8_t mult)
{                               // convert string to uint32
    uint32_t i;
    uint32_t tmp = 0;
    int isneg = 0;
    for (i = 0; src[i] != 0; i++) {
        if ((i == 0) && (src[0] == '-')) { // detect negative sign
            isneg = 1;
            continue; // jump to next character if the first one was a negative sign
        }
        if (src[i] == '.') {
            i++;
            if (mult == 0) {
                break;
            } else {
                src[i + mult] = 0;
            }
        }
        tmp *= 10;
        if (src[i] >= '0' && src[i] <= '9') {
            tmp += src[i] - '0';
        }
        if (i >= 15) {
            return 0; // out of bounds
        }
    }
    return isneg ? -tmp : tmp;    // handle negative altitudes
}

extern uint32_t h, m, s;

typedef struct gpsDataNmea_s {
    int32_t latitude;
    int32_t longitude;
    uint8_t numSat;
    int32_t altitudeCm;
    uint16_t speed;
    uint16_t hdop;
    uint16_t ground_course;
    uint32_t time;
    uint32_t date;
} gpsDataNmea_t;

static bool gpsNewFrameNMEA(char c)
{
    static gpsDataNmea_t gps_Msg;

    uint8_t frameOK = 0;
    static uint8_t param = 0, offset = 0, parity = 0;
    static char string[15];
    static uint8_t checksum_param, gps_frame = NO_FRAME;
    static uint8_t svMessageNum = 0;
    uint8_t svSatNum = 0, svPacketIdx = 0, svSatParam = 0;

    switch (c) {
        case '$':
            param = 0;
            offset = 0;
            parity = 0;
            break;
        case ',':
        case '*':
            string[offset] = 0;
            if (param == 0) {       //frame identification
                gps_frame = NO_FRAME;
                if (0 == strcmp(string, "GPGGA") || 0 == strcmp(string, "GNGGA")) {
                    gps_frame = FRAME_GGA;
                } else if (0 == strcmp(string, "GPRMC") || 0 == strcmp(string, "GNRMC")) {
                    gps_frame = FRAME_RMC;
                } else if (0 == strcmp(string, "GPGSV")) {
                    gps_frame = FRAME_GSV;
                }
            }
            
            switch (gps_frame) {
                case FRAME_GGA:        //************* GPGGA FRAME parsing
                    switch (param) {
                        //case 1:             // Time information
                          //break;
                        case 2:
                            // gps_Msg.latitude = GPS_coord_to_degrees(string);
                            break;
                        case 3:
                            if (string[0] == 'S')
                                gps_Msg.latitude *= -1;
                            break;
                        case 4:
                            // gps_Msg.longitude = GPS_coord_to_degrees(string);
                            break;
                        case 5:
                            if (string[0] == 'W')
                                gps_Msg.longitude *= -1;
                            break;
                        case 6:
                            if (string[0] > '0') {
                                GPS_FIX = 1;
                            } else {
                                GPS_FIX = 0;
                            }
                            break;
                        case 7:
                            gps_Msg.numSat = grab_fields(string, 0);
                            break;
                        case 8:
                            gps_Msg.hdop = grab_fields(string, 1) * 100;          // hdop
                            break;
                        case 9:
                            gps_Msg.altitudeCm = grab_fields(string, 1) * 10;     // altitude in centimeters. Note: NMEA delivers altitude with 1 or 3 decimals. It's safer to cut at 0.1m and multiply by 10
                            break;
                    }
                    break;
                case FRAME_RMC:        //************* GPRMC FRAME parsing
                    switch (param) {
                        case 1:
                            gps_Msg.time = grab_fields(string, 2); // UTC time hhmmss.ss
                            time = grab_fields(string, 2);
                            break;
                        case 7:
                            gps_Msg.speed = ((grab_fields(string, 1) * 5144L) / 1000L);    // speed in cm/s added by Mis
                            break;
                        case 8:
                            gps_Msg.ground_course = (grab_fields(string, 1));      // ground course deg * 10
                            break;
                        case 9:
                            gps_Msg.date = grab_fields(string, 0); // date dd/mm/yy
                            break;
                    }
                    break;
                case FRAME_GSV:
                    switch (param) {
                      /*case 1:
                            // Total number of messages of this type in this cycle
                            break; */
                        case 2:
                            svMessageNum = grab_fields(string, 0); // Message number
                            break;
                        case 3:
                            GPS_numCh = grab_fields(string, 0); // Total number of SVs visible
                            break;
                    }
                    if (param < 4)
                        break;

                    svPacketIdx = (param - 4) / 4 + 1; // satellite number in packet, 1-4
                    svSatNum    = svPacketIdx + (4 * (svMessageNum - 1)); // global satellite number
                    svSatParam  = param - 3 - (4 * (svPacketIdx - 1)); // parameter number for satellite

                    if (svSatNum > GPS_SV_MAXSATS)
                        break;

                    switch (svSatParam) {
                        case 1:
                            // SV PRN number
                            GPS_svinfo_chn[svSatNum - 1]  = svSatNum;
                            GPS_svinfo_svid[svSatNum - 1] = grab_fields(string, 0);
                            break;
                        // case 2:
                        //     // Elevation, in degrees, 90 maximum
                        //     break;
                        // case 3:
                        //     // Azimuth, degrees from True North, 000 through 359
                        //     break;
                        case 4:
                            // SNR, 00 through 99 dB (null when not tracking)
                            GPS_svinfo_cno[svSatNum - 1] = grab_fields(string, 0);
                            GPS_svinfo_quality[svSatNum - 1] = 0; // only used by ublox
                            break;
                    }

                    GPS_svInfoReceivedCount++;

                    break;
            }

            param++;
            offset = 0;
            if (c == '*')
                checksum_param = 1;
            else
                parity ^= c;
            break;
        case '\r':
        case '\n':
            if (checksum_param) {   //parity checksum
                uint8_t checksum = 16 * ((string[0] >= 'A') ? string[0] - 'A' + 10 : string[0] - '0') + ((string[1] >= 'A') ? string[1] - 'A' + 10 : string[1] - '0');
                if (checksum == parity) {
                    GPS_packetCount++;
                    switch (gps_frame) {
                    case FRAME_GGA:
                      frameOK = 1;
                      if (GPS_FIX == 1) {
                            gpsSol.llh.lat = gps_Msg.latitude;
                            gpsSol.llh.lon = gps_Msg.longitude;
                            gpsSol.numSat = gps_Msg.numSat;
                            gpsSol.llh.altCm = gps_Msg.altitudeCm;
                            gpsSol.hdop = gps_Msg.hdop;
                        }
                        break;
                    case FRAME_RMC:
                        gpsSol.groundSpeed = gps_Msg.speed;
                        gpsSol.groundCourse = gps_Msg.ground_course;
                        break;
                    } // end switch
                } else {
                }
            }
            checksum_param = 0;
            break;
        default:
            if (offset < 15)
                string[offset++] = c;
            if (!checksum_param)
                parity ^= c;
    }
    return frameOK;
}