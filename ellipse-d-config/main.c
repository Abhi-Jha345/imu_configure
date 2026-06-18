/*!
 * \file    main.c
 * \brief   Interactive connect + COMPLETE configuration tool for the SBG Ellipse-D.
 *
 * Built entirely on the binary sbgECom command API (no extra libraries).
 * The Ellipse-D belongs to the ELLIPSE series, configured through sbgEComCmd*
 * binary commands (NOT the REST API used by Ekinox/Apogee/Quanta).
 *
 * Usage:   ellipse-d-config <SERIAL_DEVICE> <BAUDRATE>
 * Example: ellipse-d-config /dev/ttyUSB0 921600
 */

#include <signal.h>

#include <sbgCommon.h>
#include <version/sbgVersion.h>
#include <sbgEComLib.h>

//======================================================================//
//  Console input helpers                                               //
//======================================================================//

static void flushLine(void) { int c; while ((c = getchar()) != '\n' && c != EOF) {} }

static int promptInt(const char *label, int def)
{
    char b[80];
    printf("%s [%d]: ", label, def);
    if (fgets(b, sizeof(b), stdin) && b[0] != '\n') return atoi(b);
    return def;
}
static float promptFloat(const char *label, float def)
{
    char b[80];
    printf("%s [%.4f]: ", label, def);
    if (fgets(b, sizeof(b), stdin) && b[0] != '\n') return (float)atof(b);
    return def;
}
static double promptDouble(const char *label, double def)
{
    char b[80];
    printf("%s [%.7f]: ", label, def);
    if (fgets(b, sizeof(b), stdin) && b[0] != '\n') return atof(b);
    return def;
}
static void promptStr(const char *label, const char *def, char *out, size_t n)
{
    char b[80];
    printf("%s [%s]: ", label, def);
    if (fgets(b, sizeof(b), stdin) && b[0] != '\n')
    {
        b[strcspn(b, "\r\n")] = '\0';
        snprintf(out, n, "%s", b);
    }
    else snprintf(out, n, "%s", def);
}
static void result(SbgErrorCode e) { printf(e == SBG_NO_ERROR ? "  -> OK\n" : "  -> FAILED (%d)\n", e); }

//======================================================================//
//  Live monitor                                                        //
//======================================================================//

static volatile sig_atomic_t gStop = 0;
static void onSigInt(int s) { SBG_UNUSED_PARAMETER(s); gStop = 1; }

static SbgErrorCode onLog(SbgEComHandle *h, SbgEComClass cls, SbgEComMsgId msg,
                          const SbgEComLogUnion *p, void *u)
{
    SBG_UNUSED_PARAMETER(h); SBG_UNUSED_PARAMETER(u);
    if (cls != SBG_ECOM_CLASS_LOG_ECOM_0) return SBG_NO_ERROR;
    switch (msg)
    {
    case SBG_ECOM_LOG_EKF_EULER:
        printf("EULER r/p/y: %7.2f %7.2f %7.2f\n",
               sbgRadToDegf(p->ekfEulerData.euler[0]), sbgRadToDegf(p->ekfEulerData.euler[1]),
               sbgRadToDegf(p->ekfEulerData.euler[2])); break;
    case SBG_ECOM_LOG_GPS1_HDT:
        printf("GPS HDT heading: %7.2f  pitch: %6.2f\n", p->gpsHdtData.heading, p->gpsHdtData.pitch); break;
    default: break;
    }
    return SBG_NO_ERROR;
}

//======================================================================//
//  Configuration actions                                              //
//======================================================================//

static void showInfo(SbgEComHandle *h)
{
    SbgEComDeviceInfo info;
    if (sbgEComCmdGetInfo(h, &info) == SBG_NO_ERROR)
    {
        char hw[32], fw[32], cal[32];
        sbgVersionToStringEncoded(info.hardwareRev, hw, sizeof(hw));
        sbgVersionToStringEncoded(info.firmwareRev, fw, sizeof(fw));
        sbgVersionToStringEncoded(info.calibationRev, cal, sizeof(cal));
        printf("\n  Product : %s\n  Serial  : %09" PRIu32 "\n  Hardware: %s\n  Firmware: %s\n  Calib   : %s\n\n",
               info.productCode, info.serialNumber, hw, fw, cal);
    }
    else printf("  unable to read device info\n");
}

static void cfgMotion(SbgEComHandle *h)
{
    printf("\n 1 General 2 Automotive 3 Marine 4 Airplane 5 Helicopter\n"
           " 6 Pedestrian 7 UAV 8 HeavyMachinery 9 Static 10 Truck\n");
    result(sbgEComCmdSensorSetMotionProfileId(h, (SbgEComMotionProfileStdIds)promptInt("Motion profile", 1)));
}

static void cfgAlign(SbgEComHandle *h)
{
    SbgEComSensorAlignmentInfo a;
    float la[3];
    printf("\nAxis dir: 0=Fwd 1=Back 2=Left 3=Right 4=Up 5=Down\n");
    a.axisDirectionX = (SbgEComAxisDirection)promptInt("  IMU X points", SBG_ECOM_ALIGNMENT_FORWARD);
    a.axisDirectionY = (SbgEComAxisDirection)promptInt("  IMU Y points", SBG_ECOM_ALIGNMENT_RIGHT);
    a.misRoll  = sbgDegToRadf(promptFloat("  misalign roll  [deg]", 0));
    a.misPitch = sbgDegToRadf(promptFloat("  misalign pitch [deg]", 0));
    a.misYaw   = sbgDegToRadf(promptFloat("  misalign yaw   [deg]", 0));
    la[0] = promptFloat("  lever arm X [m]", 0);
    la[1] = promptFloat("  lever arm Y [m]", 0);
    la[2] = promptFloat("  lever arm Z [m]", 0);
    result(sbgEComCmdSensorSetAlignmentAndLeverArm(h, &a, la));
}

static void cfgInit(SbgEComHandle *h)
{
    SbgEComInitConditionConf c;
    printf("\nInitial conditions (helps fast startup alignment):\n");
    c.latitude  = promptDouble("  latitude  [deg]", 48.0);
    c.longitude = promptDouble("  longitude [deg]", 11.0);
    c.altitude  = promptDouble("  altitude  [m]",   0.0);
    c.year  = (uint16_t)promptInt("  year",  2026);
    c.month = (uint8_t) promptInt("  month", 1);
    c.day   = (uint8_t) promptInt("  day",   1);
    result(sbgEComCmdSensorSetInitCondition(h, &c));
}

static void cfgDual(SbgEComHandle *h)
{
    SbgEComGnssInstallation g;
    if (sbgEComCmdGnss1InstallationGet(h, &g) != SBG_NO_ERROR) { printf("  read failed\n"); return; }
    printf("\nPrimary antenna lever arm (IMU->primary, m):\n");
    g.leverArmPrimary[0] = promptFloat("  X", g.leverArmPrimary[0]);
    g.leverArmPrimary[1] = promptFloat("  Y", g.leverArmPrimary[1]);
    g.leverArmPrimary[2] = promptFloat("  Z", g.leverArmPrimary[2]);
    printf("Secondary antenna lever arm (IMU->secondary, m) — measure accurately for heading:\n");
    g.leverArmSecondary[0] = promptFloat("  X", g.leverArmSecondary[0]);
    g.leverArmSecondary[1] = promptFloat("  Y", g.leverArmSecondary[1]);
    g.leverArmSecondary[2] = promptFloat("  Z", g.leverArmSecondary[2]);
    g.leverArmSecondaryMode = SBG_ECOM_GNSS_INSTALLATION_MODE_DUAL_PRECISE;
    g.leverArmPrimaryPrecise = true;
    result(sbgEComCmdGnss1InstallationSet(h, &g));
}

static void cfgGnssModel(SbgEComHandle *h)
{
    printf("\nGNSS model id (Ellipse-D internal = check sbgEComCmdGnss.h; INTERNAL typical)\n");
    result(sbgEComCmdGnss1SetModelId(h, (SbgEComGnssModelsStdIds)promptInt("GNSS model id", SBG_ECOM_GNSS_MODEL_INTERNAL)));
}

static void cfgGnssReject(SbgEComHandle *h)
{
    SbgEComGnssRejectionConf r;
    printf("\nRejection mode: 0=NeverAccept 1=Automatic 2=AlwaysAccept\n");
    r.position = (SbgEComRejectionMode)promptInt("  position", SBG_ECOM_AUTOMATIC_MODE);
    r.velocity = (SbgEComRejectionMode)promptInt("  velocity", SBG_ECOM_AUTOMATIC_MODE);
    r.hdt      = (SbgEComRejectionMode)promptInt("  heading (hdt)", SBG_ECOM_AUTOMATIC_MODE);
    result(sbgEComCmdGnss1SetRejection(h, &r));
}

static void cfgMagModel(SbgEComHandle *h)
{
    printf("\nMag model: 201=INTERNAL_NORMAL (Ellipse-D) 203=ECOM_NORMAL (external)\n");
    result(sbgEComCmdMagSetModelId(h, (SbgEComMagModelsStdId)promptInt("Mag model id", SBG_ECOM_MAG_MODEL_INTERNAL_NORMAL)));
}
static void cfgMagReject(SbgEComHandle *h)
{
    SbgEComMagRejectionConf r;
    printf("\nMag rejection: 0=NeverAccept 1=Automatic 2=AlwaysAccept\n");
    r.magneticField = (SbgEComRejectionMode)promptInt("  magnetic field", SBG_ECOM_AUTOMATIC_MODE);
    result(sbgEComCmdMagSetRejection(h, &r));
}

static void cfgOdo(SbgEComHandle *h)
{
    SbgEComOdoConf c;
    printf("\nOdometer / DMI configuration:\n");
    c.gain        = promptFloat("  gain [pulses/m]", 1000.0f);
    c.gainError   = (uint8_t)promptInt("  gain error [%]", 1);
    c.reverseMode = promptInt("  reverse mode (0/1)", 0) ? true : false;
    result(sbgEComCmdOdoSetConf(h, &c));
}
static void cfgOdoLever(SbgEComHandle *h)
{
    float la[3];
    printf("\nOdometer lever arm (IMU->odometer, m):\n");
    la[0] = promptFloat("  X", 0); la[1] = promptFloat("  Y", 0); la[2] = promptFloat("  Z", 0);
    result(sbgEComCmdOdoSetLeverArm(h, la));
}
static void cfgOdoReject(SbgEComHandle *h)
{
    SbgEComOdoRejectionConf r;
    printf("\nOdometer rejection: 0=NeverAccept 1=Automatic 2=AlwaysAccept\n");
    r.velocity = (SbgEComRejectionMode)promptInt("  velocity", SBG_ECOM_AUTOMATIC_MODE);
    result(sbgEComCmdOdoSetRejection(h, &r));
}

static void cfgAiding(SbgEComHandle *h)
{
    SbgEComAidingAssignConf a;
    if (sbgEComCmdSensorGetAidingAssignment(h, &a) != SBG_NO_ERROR) { printf("  read failed\n"); return; }
    printf("\nPort: 0=A 1=B 2=C 4=E 5=Internal 255=Disabled    Sync: 5=Internal 255=Disabled\n");
    a.gps1Port    = (SbgEComModulePortAssignment)promptInt("  GNSS port",    a.gps1Port);
    a.gps1Sync    = (SbgEComModuleSyncAssignment)promptInt("  GNSS sync",    a.gps1Sync);
    a.rtcmPort    = (SbgEComModulePortAssignment)promptInt("  RTCM port",    a.rtcmPort);
    a.airDataPort = (SbgEComModulePortAssignment)promptInt("  AirData port", a.airDataPort);
    a.dvlPort     = (SbgEComModulePortAssignment)promptInt("  DVL port",     a.dvlPort);
    a.odometerPinsConf = (SbgEComOdometerPinAssignment)promptInt("  Odometer pins conf", a.odometerPinsConf);
    result(sbgEComCmdSensorSetAidingAssignment(h, &a));
}

static void cfgSyncIn(SbgEComHandle *h)
{
    SbgEComSyncInConf c;
    printf("\nSync IN id: 0=A 1=B 2=C 3=D\n");
    int id = promptInt("  sync in id", SBG_ECOM_SYNC_IN_A);
    printf("Sensitivity: 0=Disabled 1=FallingEdge 2=RisingEdge 3=BothEdges\n");
    c.sensitivity = (SbgEComSyncInSensitivity)promptInt("  sensitivity", SBG_ECOM_SYNC_IN_RISING_EDGE);
    c.delay = promptInt("  delay [ns]", 0);
    result(sbgEComCmdSyncInSetConf(h, (SbgEComSyncInId)id, &c));
}
static void cfgSyncOut(SbgEComHandle *h)
{
    SbgEComSyncOutConf c;
    printf("\nSync OUT id: 0=A 1=B\n");
    int id = promptInt("  sync out id", SBG_ECOM_SYNC_OUT_A);
    printf("Function: 0=Disabled 1=MainLoop(200Hz) 2=Div2 4=Div4 8=Div8 40=Div40 ...\n");
    c.outputFunction = (SbgEComSyncOutFunction)promptInt("  function", SBG_ECOM_SYNC_OUT_MODE_DISABLED);
    printf("Polarity: 0=FallingEdge 1=RisingEdge 2=Toggle\n");
    c.polarity = (SbgEComSyncOutPolarity)promptInt("  polarity", SBG_ECOM_SYNC_OUT_RISING_EDGE);
    c.duration = (uint32_t)promptInt("  pulse width [ns]", 100000);
    result(sbgEComCmdSyncOutSetConf(h, (SbgEComSyncOutId)id, &c));
}

static void cfgUart(SbgEComHandle *h)
{
    SbgEComInterfaceConf c;
    printf("\nUART port: 0=COM_A 1=COM_B 2=COM_C 3=COM_D 4=COM_E\n");
    SbgEComPortId pid = (SbgEComPortId)promptInt("  port", SBG_ECOM_IF_COM_A);
    if (sbgEComCmdInterfaceGetUartConf(h, pid, &c) != SBG_NO_ERROR) { printf("  read failed\n"); return; }
    c.baudRate = (uint32_t)promptInt("  baud rate", (int)c.baudRate);
    printf("Mode: 0=OFF 1=RS-232 2=RS-422\n");
    c.mode = (SbgEComPortMode)promptInt("  mode", c.mode);
    result(sbgEComCmdInterfaceSetUartConf(h, pid, &c));
}
static void cfgCan(SbgEComHandle *h)
{
    printf("\nCAN bitrate [kbit/s]: 0=disabled 10 20 25 50 100 125 250 500 750 1000\n");
    int kbit = promptInt("  bitrate", 500);
    SbgEComCanBitRate br;
    switch (kbit) {
    case 10: br=SBG_ECOM_CAN_BITRATE_10; break; case 20: br=SBG_ECOM_CAN_BITRATE_20; break;
    case 25: br=SBG_ECOM_CAN_BITRATE_25; break; case 50: br=SBG_ECOM_CAN_BITRATE_50; break;
    case 100: br=SBG_ECOM_CAN_BITRATE_100; break; case 125: br=SBG_ECOM_CAN_BITRATE_125; break;
    case 250: br=SBG_ECOM_CAN_BITRATE_250; break; case 500: br=SBG_ECOM_CAN_BITRATE_500; break;
    case 750: br=SBG_ECOM_CAN_BITRATE_750; break; case 1000: br=SBG_ECOM_CAN_BITRATE_1000; break;
    default: br=SBG_ECOM_CAN_BITRATE_DISABLED; break; }
    printf("Mode: 0=Undefined 1=Normal 2=Spy\n");
    SbgEComCanMode mode = (SbgEComCanMode)promptInt("  mode", SBG_ECOM_CAN_MODE_NORMAL);
    result(sbgEComCmdInterfaceSetCanConf(h, br, mode));
}

static void cfgAdvanced(SbgEComHandle *h)
{
    SbgEComAdvancedConf c;
    printf("\nTime reference: 0=Disabled 1=SyncInA 2=UTC/GPS1\n");
    c.timeReference = (SbgEComTimeReferenceSrc)promptInt("  time reference", SBG_ECOM_TIME_REF_UTC_GPS_1);
    c.gnssOptions = (uint32_t)promptInt("  GNSS options bitmask (0 default)", 0);
    c.nmeaOptions = (uint32_t)promptInt("  NMEA options bitmask (0 default)", 0);
    result(sbgEComCmdAdvancedSetConf(h, &c));
}
static void cfgThresholds(SbgEComHandle *h)
{
    SbgEComValidityThresholds t;
    printf("\nValidity thresholds (raise valid flags below these std-dev values):\n");
    t.positionThreshold = promptFloat("  position [m]",     1.0f);
    t.velocityThreshold = promptFloat("  velocity [m/s]",   0.5f);
    t.attitudeThreshold = sbgDegToRadf(promptFloat("  attitude [deg]", 1.0f));
    t.headingThreshold  = sbgDegToRadf(promptFloat("  heading  [deg]", 2.0f));
    result(sbgEComCmdAdvancedSetThresholds(h, &t));
}

static void cfgOutputsDefault(SbgEComHandle *h)
{
    struct { SbgEComMsgId id; SbgEComOutputMode m; const char *n; } o[] = {
        { SBG_ECOM_LOG_IMU_DATA,  SBG_ECOM_OUTPUT_MODE_DIV_8,  "IMU_DATA 25Hz" },
        { SBG_ECOM_LOG_EKF_EULER, SBG_ECOM_OUTPUT_MODE_DIV_8,  "EKF_EULER 25Hz" },
        { SBG_ECOM_LOG_EKF_NAV,   SBG_ECOM_OUTPUT_MODE_DIV_8,  "EKF_NAV 25Hz" },
        { SBG_ECOM_LOG_GPS1_POS,  SBG_ECOM_OUTPUT_MODE_DIV_40, "GPS1_POS 5Hz" },
        { SBG_ECOM_LOG_GPS1_HDT,  SBG_ECOM_OUTPUT_MODE_DIV_40, "GPS1_HDT 5Hz" },
        { SBG_ECOM_LOG_STATUS,    SBG_ECOM_OUTPUT_MODE_DIV_40, "STATUS 5Hz" },
        { SBG_ECOM_LOG_UTC_TIME,  SBG_ECOM_OUTPUT_MODE_DIV_40, "UTC_TIME 5Hz" },
    };
    printf("\nEnabling default output set on PORT_A:\n");
    for (size_t i = 0; i < SBG_ARRAY_SIZE(o); i++)
    {
        SbgErrorCode e = sbgEComCmdOutputSetConf(h, SBG_ECOM_OUTPUT_PORT_A, SBG_ECOM_CLASS_LOG_ECOM_0, o[i].id, o[i].m);
        printf("  %-16s %s\n", o[i].n, e == SBG_NO_ERROR ? "OK" : "FAIL");
    }
}
static void cfgOutputSingle(SbgEComHandle *h)
{
    printf("\nOutput port: 0=A 2=C 3=D 4=E\n");
    int port = promptInt("  port", SBG_ECOM_OUTPUT_PORT_A);
    int msg  = promptInt("  log message id (e.g. 6=IMU_DATA, 1=STATUS, 6..)", SBG_ECOM_LOG_IMU_DATA);
    printf("Mode: 0=disabled, or divider 8=25Hz 40=5Hz 200=1Hz, 1ms/2ms/4ms via enum values\n");
    int mode = promptInt("  output mode", SBG_ECOM_OUTPUT_MODE_DIV_8);
    result(sbgEComCmdOutputSetConf(h, (SbgEComOutputPort)port, SBG_ECOM_CLASS_LOG_ECOM_0, (SbgEComMsgId)msg, (SbgEComOutputMode)mode));
}
static void cfgNmeaTalker(SbgEComHandle *h)
{
    char id[8];
    int port = promptInt("\nOutput port (0=A 2=C 3=D 4=E)", SBG_ECOM_OUTPUT_PORT_A);
    promptStr("  NMEA talker id (2 chars, e.g. GP)", "GP", id, sizeof(id));
    result(sbgEComCmdOutputSetNmeaTalkerId(h, (SbgEComOutputPort)port, id));
}
static void cfgClassEnable(SbgEComHandle *h)
{
    int port = promptInt("\nOutput port (0=A 2=C 3=D 4=E)", SBG_ECOM_OUTPUT_PORT_A);
    int cls  = promptInt("  class id (0=ECOM_0 logs, 1=ECOM_1, 2=NMEA, ...)", SBG_ECOM_CLASS_LOG_ECOM_0);
    int en   = promptInt("  enable (1) / disable (0)", 1);
    result(sbgEComCmdOutputClassSetEnable(h, (SbgEComOutputPort)port, (SbgEComClass)cls, en ? true : false));
}

static void cfgExport(SbgEComHandle *h)
{
    static uint8_t buf[32768];
    char path[128];
    size_t size = 0;
    promptStr("\nExport settings to file", "ellipse-d-settings.bin", path, sizeof(path));
    SbgErrorCode e = sbgEComCmdExportSettings(h, buf, &size, sizeof(buf));
    if (e == SBG_NO_ERROR)
    {
        FILE *f = fopen(path, "wb");
        if (f) { fwrite(buf, 1, size, f); fclose(f); printf("  -> wrote %zu bytes to %s\n", size, path); }
        else printf("  -> cannot write %s\n", path);
    }
    else result(e);
}
static void cfgImport(SbgEComHandle *h)
{
    static uint8_t buf[32768];
    char path[128];
    promptStr("\nImport settings from file", "ellipse-d-settings.bin", path, sizeof(path));
    FILE *f = fopen(path, "rb");
    if (!f) { printf("  -> cannot open %s\n", path); return; }
    size_t size = fread(buf, 1, sizeof(buf), f);
    fclose(f);
    printf("  read %zu bytes; importing...\n", size);
    result(sbgEComCmdImportSettings(h, buf, size));
}

static void settingsAction(SbgEComHandle *h, SbgEComSettingsAction a, const char *warn)
{
    printf("\n%s (y/N): ", warn);
    int c = getchar();
    flushLine();
    if (c == 'y' || c == 'Y') result(sbgEComCmdSettingsAction(h, a));
    else printf("  cancelled.\n");
}

static void liveMonitor(SbgEComHandle *h)
{
    printf("\nLive monitor — Ctrl-C to stop.\n");
    gStop = 0;
    signal(SIGINT, onSigInt);
    sbgEComSetReceiveLogCallback(h, onLog, NULL);
    while (!gStop) { if (sbgEComHandle(h) == SBG_NOT_READY) sbgSleep(1); }
    sbgEComSetReceiveLogCallback(h, NULL, NULL);
    signal(SIGINT, SIG_DFL);
    printf("\nstopped.\n");
}

//======================================================================//
//  Menu                                                                //
//======================================================================//

static void menu(SbgEComHandle *h)
{
    for (;;)
    {
        printf(
        "\n==================== Ellipse-D Configuration ====================\n"
        " DEVICE        1) Show info        2) Live monitor\n"
        " SENSOR        3) Motion profile   4) IMU alignment+lever  5) Initial conditions\n"
        " GNSS          6) Dual-antenna     7) GNSS model           8) GNSS rejection\n"
        " MAG           9) Mag model       10) Mag rejection\n"
        " ODOMETER     11) Odo config      12) Odo lever arm       13) Odo rejection\n"
        " AIDING       14) Aiding assignment (GNSS/RTCM/AirData/DVL/Odo)\n"
        " SYNC         15) Sync IN         16) Sync OUT\n"
        " INTERFACES   17) UART port       18) CAN bus\n"
        " ADVANCED     19) Time/options    20) Validity thresholds\n"
        " OUTPUTS      21) Default logs    22) Single log  23) NMEA talker  24) Enable/disable class\n"
        " SETTINGS     25) Export to file  26) Import from file\n"
        "              27) SAVE+reboot     28) Restore defaults    29) Reboot only\n"
        "               0) Quit\n"
        "=================================================================\n");
        switch (promptInt("Select", 0))
        {
        case 1:  showInfo(h);          break;
        case 2:  liveMonitor(h);       break;
        case 3:  cfgMotion(h);         break;
        case 4:  cfgAlign(h);          break;
        case 5:  cfgInit(h);           break;
        case 6:  cfgDual(h);           break;
        case 7:  cfgGnssModel(h);      break;
        case 8:  cfgGnssReject(h);     break;
        case 9:  cfgMagModel(h);       break;
        case 10: cfgMagReject(h);      break;
        case 11: cfgOdo(h);            break;
        case 12: cfgOdoLever(h);       break;
        case 13: cfgOdoReject(h);      break;
        case 14: cfgAiding(h);         break;
        case 15: cfgSyncIn(h);         break;
        case 16: cfgSyncOut(h);        break;
        case 17: cfgUart(h);           break;
        case 18: cfgCan(h);            break;
        case 19: cfgAdvanced(h);       break;
        case 20: cfgThresholds(h);     break;
        case 21: cfgOutputsDefault(h); break;
        case 22: cfgOutputSingle(h);   break;
        case 23: cfgNmeaTalker(h);     break;
        case 24: cfgClassEnable(h);    break;
        case 25: cfgExport(h);         break;
        case 26: cfgImport(h);         break;
        case 27: settingsAction(h, SBG_ECOM_SAVE_SETTINGS, "Save settings to flash and reboot?"); break;
        case 28: settingsAction(h, SBG_ECOM_RESTORE_DEFAULT_SETTINGS, "RESTORE FACTORY DEFAULTS and reboot?"); break;
        case 29: settingsAction(h, SBG_ECOM_REBOOT_ONLY, "Reboot the device now?"); break;
        case 0:  return;
        default: printf("  unknown choice\n"); break;
        }
    }
}

//======================================================================//
//  Main                                                                //
//======================================================================//

int main(int argc, char **argv)
{
    SbgErrorCode  err;
    SbgInterface  iface;
    SbgEComHandle handle;

    if (argc != 3)
    {
        printf("Usage: %s <SERIAL_DEVICE> <BAUDRATE>\nExample: %s /dev/ttyUSB0 921600\n", argv[0], argv[0]);
        return EXIT_FAILURE;
    }

    err = sbgInterfaceSerialCreate(&iface, argv[1], atoi(argv[2]));
    if (err != SBG_NO_ERROR) { SBG_LOG_ERROR(err, "unable to open serial interface %s", argv[1]); return EXIT_FAILURE; }

    err = sbgEComInit(&handle, &iface);
    if (err != SBG_NO_ERROR) { SBG_LOG_ERROR(err, "unable to initialize sbgECom"); sbgInterfaceDestroy(&iface); return EXIT_FAILURE; }

    printf("Connected to %s @ %s baud (sbgECom %s)\n", argv[1], argv[2], SBG_E_COM_VERSION_STR);
    showInfo(&handle);
    menu(&handle);

    sbgEComClose(&handle);
    sbgInterfaceDestroy(&iface);
    printf("Disconnected.\n");
    return EXIT_SUCCESS;
}
