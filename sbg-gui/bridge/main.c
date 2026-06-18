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

static void ackr(const char *cmd, SbgErrorCode e)
{
    emit("{\"t\":\"ack\",\"cmd\":\"%s\",\"ok\":%s}", cmd, e == SBG_NO_ERROR ? "true" : "false");
}

static SbgEComCanBitRate canBitRate(int kbit)
{
    switch (kbit)
    {
    case 10:   return SBG_ECOM_CAN_BITRATE_10;
    case 20:   return SBG_ECOM_CAN_BITRATE_20;
    case 25:   return SBG_ECOM_CAN_BITRATE_25;
    case 50:   return SBG_ECOM_CAN_BITRATE_50;
    case 100:  return SBG_ECOM_CAN_BITRATE_100;
    case 125:  return SBG_ECOM_CAN_BITRATE_125;
    case 250:  return SBG_ECOM_CAN_BITRATE_250;
    case 500:  return SBG_ECOM_CAN_BITRATE_500;
    case 750:  return SBG_ECOM_CAN_BITRATE_750;
    case 1000: return SBG_ECOM_CAN_BITRATE_1000;
    default:   return SBG_ECOM_CAN_BITRATE_DISABLED;
    }
}

static void cmdAlign(SbgEComHandle *h, int axX, int axY, float r, float p, float y, float lx, float ly, float lz)
{
    SbgEComSensorAlignmentInfo a;
    float la[3] = { lx, ly, lz };
    a.axisDirectionX = (SbgEComAxisDirection)axX;
    a.axisDirectionY = (SbgEComAxisDirection)axY;
    a.misRoll  = sbgDegToRadf(r);
    a.misPitch = sbgDegToRadf(p);
    a.misYaw   = sbgDegToRadf(y);
    ackr("align", sbgEComCmdSensorSetAlignmentAndLeverArm(h, &a, la));
}

static void cmdInit(SbgEComHandle *h, double lat, double lon, double alt, int yr, int mo, int dy)
{
    SbgEComInitConditionConf c;
    c.latitude = lat; c.longitude = lon; c.altitude = alt;
    c.year = (uint16_t)yr; c.month = (uint8_t)mo; c.day = (uint8_t)dy;
    ackr("init", sbgEComCmdSensorSetInitCondition(h, &c));
}

static void cmdGnssModel(SbgEComHandle *h, int id)   { ackr("gnssmodel", sbgEComCmdGnss1SetModelId(h, (SbgEComGnssModelsStdIds)id)); }
static void cmdMagModel(SbgEComHandle *h, int id)    { ackr("magmodel",  sbgEComCmdMagSetModelId(h, (SbgEComMagModelsStdId)id)); }

static void cmdGnssReject(SbgEComHandle *h, int pos, int vel, int hdt)
{
    SbgEComGnssRejectionConf r;
    r.position = (SbgEComRejectionMode)pos;
    r.velocity = (SbgEComRejectionMode)vel;
    r.hdt      = (SbgEComRejectionMode)hdt;
    ackr("gnssreject", sbgEComCmdGnss1SetRejection(h, &r));
}

static void cmdMagReject(SbgEComHandle *h, int mode)
{
    SbgEComMagRejectionConf r;
    r.magneticField = (SbgEComRejectionMode)mode;
    ackr("magreject", sbgEComCmdMagSetRejection(h, &r));
}

static void cmdOdo(SbgEComHandle *h, float gain, int gainErr, int reverse)
{
    SbgEComOdoConf c;
    c.gain = gain;
    c.gainError = (uint8_t)gainErr;
    c.reverseMode = reverse ? true : false;
    ackr("odo", sbgEComCmdOdoSetConf(h, &c));
}
static void cmdOdoLever(SbgEComHandle *h, float x, float y, float z) { float la[3]={x,y,z}; ackr("odolever", sbgEComCmdOdoSetLeverArm(h, la)); }
static void cmdOdoReject(SbgEComHandle *h, int mode)
{
    SbgEComOdoRejectionConf r;
    r.velocity = (SbgEComRejectionMode)mode;
    ackr("odoreject", sbgEComCmdOdoSetRejection(h, &r));
}

static void cmdSyncIn(SbgEComHandle *h, int id, int sens, int delay)
{
    SbgEComSyncInConf c;
    c.sensitivity = (SbgEComSyncInSensitivity)sens;
    c.delay = delay;
    ackr("syncin", sbgEComCmdSyncInSetConf(h, (SbgEComSyncInId)id, &c));
}
static void cmdSyncOut(SbgEComHandle *h, int id, int func, int pol, int dur)
{
    SbgEComSyncOutConf c;
    c.outputFunction = (SbgEComSyncOutFunction)func;
    c.polarity = (SbgEComSyncOutPolarity)pol;
    c.duration = (uint32_t)dur;
    ackr("syncout", sbgEComCmdSyncOutSetConf(h, (SbgEComSyncOutId)id, &c));
}

static void cmdUart(SbgEComHandle *h, int portId, int baud, int mode)
{
    SbgEComInterfaceConf c;
    c.baudRate = (uint32_t)baud;
    c.mode = (SbgEComPortMode)mode;
    ackr("uart", sbgEComCmdInterfaceSetUartConf(h, (SbgEComPortId)portId, &c));
}
static void cmdCan(SbgEComHandle *h, int kbit, int mode)
{
    ackr("can", sbgEComCmdInterfaceSetCanConf(h, canBitRate(kbit), (SbgEComCanMode)mode));
}

static void cmdAdvanced(SbgEComHandle *h, int timeRef, unsigned gnssOpt, unsigned nmeaOpt)
{
    SbgEComAdvancedConf c;
    c.timeReference = (SbgEComTimeReferenceSrc)timeRef;
    c.gnssOptions = gnssOpt;
    c.nmeaOptions = nmeaOpt;
    ackr("advanced", sbgEComCmdAdvancedSetConf(h, &c));
}
static void cmdThresholds(SbgEComHandle *h, float pos, float vel, float att, float head)
{
    SbgEComValidityThresholds t;
    t.positionThreshold = pos;
    t.velocityThreshold = vel;
    t.attitudeThreshold = sbgDegToRadf(att);
    t.headingThreshold  = sbgDegToRadf(head);
    ackr("thresholds", sbgEComCmdAdvancedSetThresholds(h, &t));
}

static void cmdAiding(SbgEComHandle *h, int gps1Port, int gps1Sync, int rtcm, int airData, int dvl, int odoPins)
{
    SbgEComAidingAssignConf a;
    if (sbgEComCmdSensorGetAidingAssignment(h, &a) != SBG_NO_ERROR) { ackr("aiding", SBG_ERROR); return; }
    a.gps1Port         = (SbgEComModulePortAssignment)gps1Port;
    a.gps1Sync         = (SbgEComModuleSyncAssignment)gps1Sync;
    a.rtcmPort         = (SbgEComModulePortAssignment)rtcm;
    a.airDataPort      = (SbgEComModulePortAssignment)airData;
    a.dvlPort          = (SbgEComModulePortAssignment)dvl;
    a.odometerPinsConf = (SbgEComOdometerPinAssignment)odoPins;
    ackr("aiding", sbgEComCmdSensorSetAidingAssignment(h, &a));
}

static void cmdNmeaTalker(SbgEComHandle *h, int port, const char *id)
{
    ackr("nmeatalker", sbgEComCmdOutputSetNmeaTalkerId(h, (SbgEComOutputPort)port, id));
}
static void cmdClassEnable(SbgEComHandle *h, int port, int cls, int en)
{
    ackr("classenable", sbgEComCmdOutputClassSetEnable(h, (SbgEComOutputPort)port, (SbgEComClass)cls, en ? true : false));
}
static void cmdLog(SbgEComHandle *h, int port, int cls, int msg, int mode)
{
    ackr("log", sbgEComCmdOutputSetConf(h, (SbgEComOutputPort)port, (SbgEComClass)cls, (SbgEComMsgId)msg, (SbgEComOutputMode)mode));
}

static void cmdExport(SbgEComHandle *h, const char *path)
{
    static uint8_t buf[32768];
    size_t size = 0;
    SbgErrorCode e = sbgEComCmdExportSettings(h, buf, &size, sizeof(buf));
    if (e == SBG_NO_ERROR)
    {
        FILE *f = fopen(path, "wb");
        if (f) { fwrite(buf, 1, size, f); fclose(f); }
        else   { e = SBG_ERROR; }
    }
    emit("{\"t\":\"ack\",\"cmd\":\"export\",\"ok\":%s,\"bytes\":%zu}", e == SBG_NO_ERROR ? "true" : "false", size);
}
static void cmdImport(SbgEComHandle *h, const char *path)
{
    static uint8_t buf[32768];
    FILE *f = fopen(path, "rb");
    if (!f) { ackr("import", SBG_ERROR); return; }
    size_t size = fread(buf, 1, sizeof(buf), f);
    fclose(f);
    ackr("import", sbgEComCmdImportSettings(h, buf, size));
}

static void handleCommand(SbgEComHandle *h, char *line, const char *port, const char *baud)
{
    int   i1, i2, i3, i4, i5, i6;
    float f1, f2, f3, f4, f5, f6, f7, f8;
    double d1, d2, d3;
    unsigned u1, u2;
    char    str[64];

    if      (strncmp(line, "outputs", 7) == 0)                              { enableDefaultOutputs(h); }
    else if (sscanf(line, "motion %d", &i1) == 1)                           { setMotion(h, i1); }
    else if (sscanf(line, "dual %f %f %f %f %f %f", &f1,&f2,&f3,&f4,&f5,&f6) == 6) { setDual(h, f1,f2,f3,f4,f5,f6); }
    else if (sscanf(line, "align %d %d %f %f %f %f %f %f", &i1,&i2,&f1,&f2,&f3,&f4,&f5,&f6) == 8) { cmdAlign(h,i1,i2,f1,f2,f3,f4,f5,f6); }
    else if (sscanf(line, "init %lf %lf %lf %d %d %d", &d1,&d2,&d3,&i1,&i2,&i3) == 6) { cmdInit(h,d1,d2,d3,i1,i2,i3); }
    else if (sscanf(line, "gnssmodel %d", &i1) == 1)                        { cmdGnssModel(h, i1); }
    else if (sscanf(line, "gnssreject %d %d %d", &i1,&i2,&i3) == 3)         { cmdGnssReject(h, i1,i2,i3); }
    else if (sscanf(line, "magmodel %d", &i1) == 1)                         { cmdMagModel(h, i1); }
    else if (sscanf(line, "magreject %d", &i1) == 1)                        { cmdMagReject(h, i1); }
    else if (sscanf(line, "odo %f %d %d", &f1,&i1,&i2) == 3)                { cmdOdo(h, f1, i1, i2); }
    else if (sscanf(line, "odolever %f %f %f", &f1,&f2,&f3) == 3)           { cmdOdoLever(h, f1,f2,f3); }
    else if (sscanf(line, "odoreject %d", &i1) == 1)                        { cmdOdoReject(h, i1); }
    else if (sscanf(line, "syncin %d %d %d", &i1,&i2,&i3) == 3)             { cmdSyncIn(h, i1,i2,i3); }
    else if (sscanf(line, "syncout %d %d %d %d", &i1,&i2,&i3,&i4) == 4)     { cmdSyncOut(h, i1,i2,i3,i4); }
    else if (sscanf(line, "uart %d %d %d", &i1,&i2,&i3) == 3)               { cmdUart(h, i1,i2,i3); }
    else if (sscanf(line, "can %d %d", &i1,&i2) == 2)                       { cmdCan(h, i1, i2); }
    else if (sscanf(line, "advanced %d %u %u", &i1,&u1,&u2) == 3)           { cmdAdvanced(h, i1, u1, u2); }
    else if (sscanf(line, "thresholds %f %f %f %f", &f1,&f2,&f3,&f4) == 4)  { cmdThresholds(h, f1,f2,f3,f4); }
    else if (sscanf(line, "aiding %d %d %d %d %d %d", &i1,&i2,&i3,&i4,&i5,&i6) == 6) { cmdAiding(h, i1,i2,i3,i4,i5,i6); }
    else if (sscanf(line, "nmeatalker %d %63s", &i1, str) == 2)             { cmdNmeaTalker(h, i1, str); }
    else if (sscanf(line, "classenable %d %d %d", &i1,&i2,&i3) == 3)        { cmdClassEnable(h, i1,i2,i3); }
    else if (sscanf(line, "log %d %d %d %d", &i1,&i2,&i3,&i4) == 4)         { cmdLog(h, i1,i2,i3,i4); }
    else if (sscanf(line, "export %63s", str) == 1)                        { cmdExport(h, str); }
    else if (sscanf(line, "import %63s", str) == 1)                        { cmdImport(h, str); }
    else if (strncmp(line, "save", 4) == 0)                                 { settingsAction(h, SBG_ECOM_SAVE_SETTINGS, "save"); }
    else if (strncmp(line, "restore", 7) == 0)                              { settingsAction(h, SBG_ECOM_RESTORE_DEFAULT_SETTINGS, "restore"); }
    else if (strncmp(line, "reboot", 6) == 0)                               { settingsAction(h, SBG_ECOM_REBOOT_ONLY, "reboot"); }
    else if (strncmp(line, "info", 4) == 0)                                 { emitInfo(h, port, baud); }
    else if (line[0] != '\0')                                               { emit("{\"t\":\"ack\",\"cmd\":\"unknown\",\"ok\":false}"); }
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
