/*!
 * \file    main.c
 * \brief   JSON telemetry bridge for the SBG Ellipse-D, for use by a GUI front-end.
 *
 * Reads the device over serial using the sbgECom library and:
 *   - emits one compact JSON object per received log on stdout (line-delimited), and
 *   - accepts simple text commands on stdin to (re)configure the device.
 *
 * This keeps all binary-protocol parsing inside the proven C library; the GUI
 * only deals with line-delimited JSON.
 *
 * Usage:   sbg-bridge <SERIAL_DEVICE> <BAUDRATE>
 *
 * stdin commands (one per line):
 *   outputs                              enable a default output set on PORT_A
 *   motion <id>                          set motion profile (1..10)
 *   dual <px> <py> <pz> <sx> <sy> <sz>   set GNSS dual-antenna lever arms (precise)
 *   save                                 save settings to flash + reboot
 *   restore                              restore factory defaults + reboot
 *   info                                 re-emit device info
 */

#include <fcntl.h>
#include <unistd.h>
#include <string.h>
#include <stdarg.h>

#include <sbgCommon.h>
#include <version/sbgVersion.h>
#include <sbgEComLib.h>

//----------------------------------------------------------------------//
//  JSON emit helpers (always followed by a newline + flush)            //
//----------------------------------------------------------------------//

static void emit(const char *fmt, ...)
{
    va_list args;
    va_start(args, fmt);
    vprintf(fmt, args);
    va_end(args);
    putchar('\n');
    fflush(stdout);
}

//----------------------------------------------------------------------//
//  Log reception -> JSON                                               //
//----------------------------------------------------------------------//

static SbgErrorCode onLog(SbgEComHandle *pHandle, SbgEComClass cls, SbgEComMsgId msg,
                          const SbgEComLogUnion *p, void *pUser)
{
    SBG_UNUSED_PARAMETER(pHandle);
    SBG_UNUSED_PARAMETER(pUser);

    if (cls != SBG_ECOM_CLASS_LOG_ECOM_0)
    {
        return SBG_NO_ERROR;
    }

    switch (msg)
    {
    case SBG_ECOM_LOG_EKF_EULER:
        emit("{\"t\":\"euler\",\"roll\":%.5f,\"pitch\":%.5f,\"yaw\":%.5f,"
             "\"rollStd\":%.5f,\"pitchStd\":%.5f,\"yawStd\":%.5f,\"mode\":%d,\"status\":%u}",
             sbgRadToDegf(p->ekfEulerData.euler[0]), sbgRadToDegf(p->ekfEulerData.euler[1]),
             sbgRadToDegf(p->ekfEulerData.euler[2]),
             sbgRadToDegf(p->ekfEulerData.eulerStdDev[0]), sbgRadToDegf(p->ekfEulerData.eulerStdDev[1]),
             sbgRadToDegf(p->ekfEulerData.eulerStdDev[2]),
             (int)sbgEComLogEkfGetSolutionMode(p->ekfEulerData.status), p->ekfEulerData.status);
        break;

    case SBG_ECOM_LOG_EKF_NAV:
        emit("{\"t\":\"nav\",\"lat\":%.7f,\"lon\":%.7f,\"alt\":%.3f,"
             "\"vN\":%.3f,\"vE\":%.3f,\"vD\":%.3f,\"altStd\":%.3f,\"mode\":%d,\"status\":%u}",
             p->ekfNavData.position[0], p->ekfNavData.position[1], p->ekfNavData.position[2],
             p->ekfNavData.velocity[0], p->ekfNavData.velocity[1], p->ekfNavData.velocity[2],
             p->ekfNavData.positionStdDev[2],
             (int)sbgEComLogEkfGetSolutionMode(p->ekfNavData.status), p->ekfNavData.status);
        break;

    case SBG_ECOM_LOG_GPS1_POS:
        emit("{\"t\":\"gnss\",\"fix\":%d,\"sv\":%d,\"lat\":%.7f,\"lon\":%.7f,\"alt\":%.3f}",
             (int)sbgEComLogGnssPosGetType(&p->gpsPosData), (int)p->gpsPosData.numSvUsed,
             p->gpsPosData.latitude, p->gpsPosData.longitude, p->gpsPosData.altitude);
        break;

    case SBG_ECOM_LOG_GPS1_HDT:
        emit("{\"t\":\"hdt\",\"status\":%d,\"heading\":%.3f,\"headingAcc\":%.3f,"
             "\"pitch\":%.3f,\"sv\":%d}",
             (int)sbgEComLogGnssHdtGetStatus(&p->gpsHdtData), p->gpsHdtData.heading,
             p->gpsHdtData.headingAccuracy, p->gpsHdtData.pitch, (int)p->gpsHdtData.numSvUsed);
        break;

    case SBG_ECOM_LOG_IMU_DATA:
        emit("{\"t\":\"imu\",\"ax\":%.4f,\"ay\":%.4f,\"az\":%.4f,"
             "\"gx\":%.4f,\"gy\":%.4f,\"gz\":%.4f,\"temp\":%.2f}",
             p->imuData.accelerometers[0], p->imuData.accelerometers[1], p->imuData.accelerometers[2],
             sbgRadToDegf(p->imuData.gyroscopes[0]), sbgRadToDegf(p->imuData.gyroscopes[1]),
             sbgRadToDegf(p->imuData.gyroscopes[2]), p->imuData.temperature);
        break;

    case SBG_ECOM_LOG_STATUS:
        emit("{\"t\":\"status\",\"general\":%u}", p->statusData.generalStatus);
        break;

    default:
        break;
    }

    return SBG_NO_ERROR;
}

//----------------------------------------------------------------------//
//  Command handling                                                    //
//----------------------------------------------------------------------//

static void enableDefaultOutputs(SbgEComHandle *h)
{
    struct { SbgEComMsgId id; SbgEComOutputMode mode; } o[] = {
        { SBG_ECOM_LOG_IMU_DATA,  SBG_ECOM_OUTPUT_MODE_DIV_8  },
        { SBG_ECOM_LOG_EKF_EULER, SBG_ECOM_OUTPUT_MODE_DIV_8  },
        { SBG_ECOM_LOG_EKF_NAV,   SBG_ECOM_OUTPUT_MODE_DIV_8  },
        { SBG_ECOM_LOG_GPS1_POS,  SBG_ECOM_OUTPUT_MODE_DIV_40 },
        { SBG_ECOM_LOG_GPS1_HDT,  SBG_ECOM_OUTPUT_MODE_DIV_40 },
        { SBG_ECOM_LOG_STATUS,    SBG_ECOM_OUTPUT_MODE_DIV_40 },
    };
    SbgErrorCode last = SBG_NO_ERROR;
    for (size_t i = 0; i < SBG_ARRAY_SIZE(o); i++)
    {
        SbgErrorCode e = sbgEComCmdOutputSetConf(h, SBG_ECOM_OUTPUT_PORT_A,
                                                 SBG_ECOM_CLASS_LOG_ECOM_0, o[i].id, o[i].mode);
        if (e != SBG_NO_ERROR) { last = e; }
    }
    emit("{\"t\":\"ack\",\"cmd\":\"outputs\",\"ok\":%s}", last == SBG_NO_ERROR ? "true" : "false");
}

static void setMotion(SbgEComHandle *h, int id)
{
    SbgErrorCode e = sbgEComCmdSensorSetMotionProfileId(h, (SbgEComMotionProfileStdIds)id);
    emit("{\"t\":\"ack\",\"cmd\":\"motion\",\"ok\":%s}", e == SBG_NO_ERROR ? "true" : "false");
}

static void setDual(SbgEComHandle *h, float px, float py, float pz, float sx, float sy, float sz)
{
    SbgEComGnssInstallation g;
    if (sbgEComCmdGnss1InstallationGet(h, &g) != SBG_NO_ERROR)
    {
        emit("{\"t\":\"ack\",\"cmd\":\"dual\",\"ok\":false}");
        return;
    }
    g.leverArmPrimary[0] = px; g.leverArmPrimary[1] = py; g.leverArmPrimary[2] = pz;
    g.leverArmSecondary[0] = sx; g.leverArmSecondary[1] = sy; g.leverArmSecondary[2] = sz;
    g.leverArmSecondaryMode = SBG_ECOM_GNSS_INSTALLATION_MODE_DUAL_PRECISE;
    g.leverArmPrimaryPrecise = true;
    SbgErrorCode e = sbgEComCmdGnss1InstallationSet(h, &g);
    emit("{\"t\":\"ack\",\"cmd\":\"dual\",\"ok\":%s}", e == SBG_NO_ERROR ? "true" : "false");
}

static void settingsAction(SbgEComHandle *h, SbgEComSettingsAction action, const char *name)
{
    SbgErrorCode e = sbgEComCmdSettingsAction(h, action);
    emit("{\"t\":\"ack\",\"cmd\":\"%s\",\"ok\":%s}", name, e == SBG_NO_ERROR ? "true" : "false");
}

static void emitInfo(SbgEComHandle *h, const char *port, const char *baud)
{
    SbgEComDeviceInfo info;
    if (sbgEComCmdGetInfo(h, &info) == SBG_NO_ERROR)
    {
        char fw[32];
        sbgVersionToStringEncoded(info.firmwareRev, fw, sizeof(fw));
        emit("{\"t\":\"info\",\"product\":\"%s\",\"sn\":\"%09u\",\"fw\":\"%s\",\"port\":\"%s\",\"baud\":\"%s\"}",
             info.productCode, info.serialNumber, fw, port, baud);
    }
    else
    {
        emit("{\"t\":\"info\",\"product\":\"?\",\"sn\":\"?\",\"fw\":\"?\",\"port\":\"%s\",\"baud\":\"%s\"}", port, baud);
    }
}

static void handleCommand(SbgEComHandle *h, char *line, const char *port, const char *baud)
{
    int   id;
    float px, py, pz, sx, sy, sz;

    if      (strncmp(line, "outputs", 7) == 0)              { enableDefaultOutputs(h); }
    else if (sscanf(line, "motion %d", &id) == 1)           { setMotion(h, id); }
    else if (sscanf(line, "dual %f %f %f %f %f %f",
                    &px, &py, &pz, &sx, &sy, &sz) == 6)      { setDual(h, px, py, pz, sx, sy, sz); }
    else if (strncmp(line, "save", 4) == 0)                 { settingsAction(h, SBG_ECOM_SAVE_SETTINGS, "save"); }
    else if (strncmp(line, "restore", 7) == 0)              { settingsAction(h, SBG_ECOM_RESTORE_DEFAULT_SETTINGS, "restore"); }
    else if (strncmp(line, "info", 4) == 0)                 { emitInfo(h, port, baud); }
    else if (line[0] != '\0')                               { emit("{\"t\":\"ack\",\"cmd\":\"unknown\",\"ok\":false}"); }
}

//----------------------------------------------------------------------//
//  Main                                                                //
//----------------------------------------------------------------------//

int main(int argc, char **argv)
{
    SbgErrorCode    err;
    SbgInterface    iface;
    SbgEComHandle   handle;
    char            stdinBuf[512];
    size_t          stdinLen = 0;

    if (argc != 3)
    {
        emit("{\"t\":\"error\",\"msg\":\"usage: sbg-bridge <device> <baud>\"}");
        return EXIT_FAILURE;
    }

    err = sbgInterfaceSerialCreate(&iface, argv[1], atoi(argv[2]));
    if (err != SBG_NO_ERROR)
    {
        emit("{\"t\":\"error\",\"msg\":\"cannot open serial port %s\"}", argv[1]);
        return EXIT_FAILURE;
    }

    err = sbgEComInit(&handle, &iface);
    if (err != SBG_NO_ERROR)
    {
        emit("{\"t\":\"error\",\"msg\":\"sbgEComInit failed\"}");
        sbgInterfaceDestroy(&iface);
        return EXIT_FAILURE;
    }

    emitInfo(&handle, argv[1], argv[2]);
    sbgEComSetReceiveLogCallback(&handle, onLog, NULL);

    // Make stdin non-blocking so we can interleave command reading with the rx loop.
    fcntl(STDIN_FILENO, F_SETFL, fcntl(STDIN_FILENO, F_GETFL, 0) | O_NONBLOCK);

    for (;;)
    {
        err = sbgEComHandle(&handle);
        if (err == SBG_NOT_READY)
        {
            sbgSleep(1);
        }

        // Drain available stdin, splitting on newlines.
        ssize_t n = read(STDIN_FILENO, stdinBuf + stdinLen, sizeof(stdinBuf) - 1 - stdinLen);
        if (n == 0)
        {
            break;                          // stdin closed -> parent exited
        }
        else if (n > 0)
        {
            stdinLen += (size_t)n;
            stdinBuf[stdinLen] = '\0';

            char *nl;
            char *start = stdinBuf;
            while ((nl = strchr(start, '\n')) != NULL)
            {
                *nl = '\0';
                handleCommand(&handle, start, argv[1], argv[2]);
                start = nl + 1;
            }
            // Shift any partial remainder to the front of the buffer.
            stdinLen = strlen(start);
            memmove(stdinBuf, start, stdinLen + 1);
        }
    }

    sbgEComSetReceiveLogCallback(&handle, NULL, NULL);
    sbgEComClose(&handle);
    sbgInterfaceDestroy(&iface);
    return EXIT_SUCCESS;
}
