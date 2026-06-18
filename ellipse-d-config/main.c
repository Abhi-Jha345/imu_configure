/*!
 * \file    main.c
 * \brief   Interactive connect + full configuration tool for the SBG Ellipse-D (dual-antenna GNSS/INS).
 *
 * Built entirely on the binary sbgECom command API (no extra libraries).
 *
 * Usage:   ellipse-d-config <SERIAL_DEVICE> <BAUDRATE>
 * Example: ellipse-d-config /dev/ttyUSB0 921600
 *
 * The Ellipse-D belongs to the ELLIPSE series, so it is configured through the
 * sbgEComCmd* binary commands (NOT the REST API used by Ekinox/Apogee/Quanta).
 */

#include <signal.h>

#include <sbgCommon.h>
#include <version/sbgVersion.h>
#include <sbgEComLib.h>

//======================================================================//
//  Small console input helpers                                         //
//======================================================================//

static void flushLine(void)
{
    int c;
    while ((c = getchar()) != '\n' && c != EOF) { }
}

static int promptInt(const char *label, int defValue)
{
    char buffer[64];

    printf("%s [%d]: ", label, defValue);
    if (fgets(buffer, sizeof(buffer), stdin) && (buffer[0] != '\n'))
    {
        return atoi(buffer);
    }
    return defValue;
}

static float promptFloat(const char *label, float defValue)
{
    char buffer[64];

    printf("%s [%.4f]: ", label, defValue);
    if (fgets(buffer, sizeof(buffer), stdin) && (buffer[0] != '\n'))
    {
        return (float)atof(buffer);
    }
    return defValue;
}

//======================================================================//
//  Live data callback (used by the monitor menu entry)                 //
//======================================================================//

static volatile sig_atomic_t   gStopMonitor = 0;

static void onSigInt(int sig)
{
    SBG_UNUSED_PARAMETER(sig);
    gStopMonitor = 1;
}

static SbgErrorCode onLogReceived(SbgEComHandle *pHandle, SbgEComClass msgClass,
                                  SbgEComMsgId msg, const SbgEComLogUnion *pLogData, void *pUserArg)
{
    SBG_UNUSED_PARAMETER(pHandle);
    SBG_UNUSED_PARAMETER(pUserArg);

    if (msgClass != SBG_ECOM_CLASS_LOG_ECOM_0)
    {
        return SBG_NO_ERROR;
    }

    switch (msg)
    {
    case SBG_ECOM_LOG_EKF_EULER:
        printf("EULER  r/p/y[deg]: % 7.2f % 7.2f % 7.2f\n",
               sbgRadToDegf(pLogData->ekfEulerData.euler[0]),
               sbgRadToDegf(pLogData->ekfEulerData.euler[1]),
               sbgRadToDegf(pLogData->ekfEulerData.euler[2]));
        break;

    case SBG_ECOM_LOG_EKF_NAV:
        printf("NAV    lat/lon/alt: % 11.7f % 11.7f % 8.2f   solution=0x%08x\n",
               pLogData->ekfNavData.position[0],
               pLogData->ekfNavData.position[1],
               pLogData->ekfNavData.position[2],
               pLogData->ekfNavData.status);
        break;

    case SBG_ECOM_LOG_GPS1_HDT:
        printf("GPS HDT heading/pitch[deg]: % 7.2f % 7.2f   (dual antenna)\n",
               pLogData->gpsHdtData.heading, pLogData->gpsHdtData.pitch);
        break;

    case SBG_ECOM_LOG_IMU_DATA:
        printf("IMU    accel[m/s2]: % 7.3f % 7.3f % 7.3f\n",
               pLogData->imuData.accelerometers[0],
               pLogData->imuData.accelerometers[1],
               pLogData->imuData.accelerometers[2]);
        break;

    default:
        break;
    }

    return SBG_NO_ERROR;
}

//======================================================================//
//  Configuration actions                                               //
//======================================================================//

static void showDeviceInfo(SbgEComHandle *pHandle)
{
    SbgEComDeviceInfo   info;
    SbgErrorCode        err;

    err = sbgEComCmdGetInfo(pHandle, &info);
    if (err == SBG_NO_ERROR)
    {
        char hw[32], fw[32], cal[32];

        sbgVersionToStringEncoded(info.hardwareRev, hw,  sizeof(hw));
        sbgVersionToStringEncoded(info.firmwareRev, fw,  sizeof(fw));
        sbgVersionToStringEncoded(info.calibationRev, cal, sizeof(cal));

        printf("\n  Product Code : %s\n", info.productCode);
        printf("  Serial Number: %09" PRIu32 "\n", info.serialNumber);
        printf("  Hardware Rev : %s\n", hw);
        printf("  Firmware Rev : %s\n", fw);
        printf("  Calib. Rev   : %s\n\n", cal);
    }
    else
    {
        SBG_LOG_ERROR(err, "unable to read device info");
    }
}

static void configMotionProfile(SbgEComHandle *pHandle)
{
    int sel;

    printf("\nMotion profiles:\n"
           "  1 General  2 Automotive  3 Marine  4 Airplane  5 Helicopter\n"
           "  6 Pedestrian  7 UAV(rotary)  8 Heavy machinery  9 Static  10 Truck\n");
    sel = promptInt("Select motion profile", SBG_ECOM_MOTION_PROFILE_GENERAL_PURPOSE);

    SbgErrorCode err = sbgEComCmdSensorSetMotionProfileId(pHandle, (SbgEComMotionProfileStdIds)sel);
    printf(err == SBG_NO_ERROR ? "  OK\n" : "  FAILED (%d)\n", err);
}

static void configImuAlignment(SbgEComHandle *pHandle)
{
    SbgEComSensorAlignmentInfo  align;
    float                       leverArm[3];
    SbgErrorCode                err;

    printf("\nIMU axis directions: 0=Fwd 1=Back 2=Left 3=Right 4=Up 5=Down\n");
    align.axisDirectionX = (SbgEComAxisDirection)promptInt("  IMU X axis points", SBG_ECOM_ALIGNMENT_FORWARD);
    align.axisDirectionY = (SbgEComAxisDirection)promptInt("  IMU Y axis points", SBG_ECOM_ALIGNMENT_RIGHT);
    align.misRoll  = sbgDegToRadf(promptFloat("  Fine misalignment roll  [deg]", 0.0f));
    align.misPitch = sbgDegToRadf(promptFloat("  Fine misalignment pitch [deg]", 0.0f));
    align.misYaw   = sbgDegToRadf(promptFloat("  Fine misalignment yaw   [deg]", 0.0f));

    printf("IMU lever arm = position of IMU relative to vehicle reference point (meters):\n");
    leverArm[0] = promptFloat("  Lever arm X [m]", 0.0f);
    leverArm[1] = promptFloat("  Lever arm Y [m]", 0.0f);
    leverArm[2] = promptFloat("  Lever arm Z [m]", 0.0f);

    err = sbgEComCmdSensorSetAlignmentAndLeverArm(pHandle, &align, leverArm);
    printf(err == SBG_NO_ERROR ? "  OK\n" : "  FAILED (%d)\n", err);
}

static void configDualAntenna(SbgEComHandle *pHandle)
{
    SbgEComGnssInstallation     gnss;
    SbgErrorCode                err;

    //
    // Read current installation first so we keep anything we don't touch.
    //
    err = sbgEComCmdGnss1InstallationGet(pHandle, &gnss);
    if (err != SBG_NO_ERROR)
    {
        SBG_LOG_ERROR(err, "unable to read GNSS installation");
        return;
    }

    printf("\nPRIMARY GNSS antenna lever arm (IMU -> primary antenna, meters):\n");
    gnss.leverArmPrimary[0] = promptFloat("  Primary X [m]", gnss.leverArmPrimary[0]);
    gnss.leverArmPrimary[1] = promptFloat("  Primary Y [m]", gnss.leverArmPrimary[1]);
    gnss.leverArmPrimary[2] = promptFloat("  Primary Z [m]", gnss.leverArmPrimary[2]);

    printf("SECONDARY GNSS antenna lever arm (IMU -> secondary antenna, meters):\n");
    printf("  (For the Ellipse-D dual-antenna heading, measure this accurately.)\n");
    gnss.leverArmSecondary[0] = promptFloat("  Secondary X [m]", gnss.leverArmSecondary[0]);
    gnss.leverArmSecondary[1] = promptFloat("  Secondary Y [m]", gnss.leverArmSecondary[1]);
    gnss.leverArmSecondary[2] = promptFloat("  Secondary Z [m]", gnss.leverArmSecondary[2]);

    //
    // Dual precise = use both antennas with an accurately known secondary lever arm.
    //
    gnss.leverArmSecondaryMode = SBG_ECOM_GNSS_INSTALLATION_MODE_DUAL_PRECISE;
    gnss.leverArmPrimaryPrecise = true;

    err = sbgEComCmdGnss1InstallationSet(pHandle, &gnss);
    printf(err == SBG_NO_ERROR ? "  OK (dual-antenna precise mode)\n" : "  FAILED (%d)\n", err);
}

static void configAidingAssignment(SbgEComHandle *pHandle)
{
    SbgEComAidingAssignConf     aiding;
    SbgErrorCode                err;

    err = sbgEComCmdSensorGetAidingAssignment(pHandle, &aiding);
    if (err != SBG_NO_ERROR)
    {
        SBG_LOG_ERROR(err, "unable to read aiding assignment");
        return;
    }

    //
    // The Ellipse-D has an internal GNSS receiver -> assign GNSS module to INTERNAL.
    //
    aiding.gps1Port = SBG_ECOM_MODULE_INTERNAL;

    printf("\nRTCM (RTK corrections) input port: 0=PORT_A 1=PORT_B 2=PORT_C 4=PORT_E 255=disabled\n");
    aiding.rtcmPort = (SbgEComModulePortAssignment)promptInt("  RTCM port", aiding.rtcmPort);

    err = sbgEComCmdSensorSetAidingAssignment(pHandle, &aiding);
    printf(err == SBG_NO_ERROR ? "  OK (GNSS=internal)\n" : "  FAILED (%d)\n", err);
}

static void configUart(SbgEComHandle *pHandle)
{
    SbgEComInterfaceConf    conf;
    SbgErrorCode            err;
    int                     portSel;
    SbgEComPortId           portId;

    printf("\nUART port to configure: 0=COM_A 1=COM_B 2=COM_C 3=COM_D 4=COM_E\n");
    portSel = promptInt("  Port", SBG_ECOM_IF_COM_A);
    portId  = (SbgEComPortId)portSel;

    err = sbgEComCmdInterfaceGetUartConf(pHandle, portId, &conf);
    if (err != SBG_NO_ERROR)
    {
        SBG_LOG_ERROR(err, "unable to read UART config");
        return;
    }

    conf.baudRate = (uint32_t)promptInt("  Baud rate", (int)conf.baudRate);
    printf("  Mode: 0=OFF 1=RS-232 2=RS-422\n");
    conf.mode = (SbgEComPortMode)promptInt("  Mode", conf.mode);

    err = sbgEComCmdInterfaceSetUartConf(pHandle, portId, &conf);
    printf(err == SBG_NO_ERROR ? "  OK (re-open at new baud after save+reboot)\n" : "  FAILED (%d)\n", err);
}

static void configOutputs(SbgEComHandle *pHandle)
{
    //
    // Apply a sensible default output set on PORT_A at 25 Hz (DIV_8 of 200 Hz),
    // GPS logs at their own rate. Adjust to taste.
    //
    struct { SbgEComMsgId id; SbgEComOutputMode mode; const char *name; } outs[] = {
        { SBG_ECOM_LOG_IMU_DATA,  SBG_ECOM_OUTPUT_MODE_DIV_8,  "IMU_DATA  (25 Hz)" },
        { SBG_ECOM_LOG_EKF_EULER, SBG_ECOM_OUTPUT_MODE_DIV_8,  "EKF_EULER (25 Hz)" },
        { SBG_ECOM_LOG_EKF_NAV,   SBG_ECOM_OUTPUT_MODE_DIV_8,  "EKF_NAV   (25 Hz)" },
        { SBG_ECOM_LOG_GPS1_POS,  SBG_ECOM_OUTPUT_MODE_DIV_40, "GPS1_POS  (5 Hz)"  },
        { SBG_ECOM_LOG_GPS1_VEL,  SBG_ECOM_OUTPUT_MODE_DIV_40, "GPS1_VEL  (5 Hz)"  },
        { SBG_ECOM_LOG_GPS1_HDT,  SBG_ECOM_OUTPUT_MODE_DIV_40, "GPS1_HDT  (5 Hz, dual-antenna heading)" },
        { SBG_ECOM_LOG_STATUS,    SBG_ECOM_OUTPUT_MODE_DIV_40, "STATUS    (5 Hz)"  },
        { SBG_ECOM_LOG_UTC_TIME,  SBG_ECOM_OUTPUT_MODE_DIV_40, "UTC_TIME  (5 Hz)"  },
    };

    printf("\nEnabling default output set on PORT_A:\n");
    for (size_t i = 0; i < SBG_ARRAY_SIZE(outs); i++)
    {
        SbgErrorCode err = sbgEComCmdOutputSetConf(pHandle, SBG_ECOM_OUTPUT_PORT_A,
                                                   SBG_ECOM_CLASS_LOG_ECOM_0, outs[i].id, outs[i].mode);
        printf("  %-45s %s\n", outs[i].name, err == SBG_NO_ERROR ? "OK" : "FAILED");
    }
}

static void liveMonitor(SbgEComHandle *pHandle)
{
    printf("\nStreaming live data — press Ctrl-C to stop and return to the menu.\n\n");
    gStopMonitor = 0;
    signal(SIGINT, onSigInt);

    sbgEComSetReceiveLogCallback(pHandle, onLogReceived, NULL);

    while (!gStopMonitor)
    {
        SbgErrorCode err = sbgEComHandle(pHandle);
        if (err == SBG_NOT_READY)
        {
            sbgSleep(1);
        }
    }

    sbgEComSetReceiveLogCallback(pHandle, NULL, NULL);
    signal(SIGINT, SIG_DFL);
    printf("\nStopped monitoring.\n");
}

static void saveAndReboot(SbgEComHandle *pHandle)
{
    printf("\nSave settings to non-volatile memory and reboot? (y/N): ");
    if (getchar() == 'y')
    {
        SbgErrorCode err = sbgEComCmdSettingsAction(pHandle, SBG_ECOM_SAVE_SETTINGS);
        printf(err == SBG_NO_ERROR ? "  Saved + rebooting.\n" : "  FAILED (%d)\n", err);
    }
    else
    {
        printf("  Cancelled.\n");
    }
    flushLine();
}

static void restoreDefaults(SbgEComHandle *pHandle)
{
    printf("\nRESTORE FACTORY DEFAULTS and reboot? This erases your config. (y/N): ");
    if (getchar() == 'y')
    {
        SbgErrorCode err = sbgEComCmdSettingsAction(pHandle, SBG_ECOM_RESTORE_DEFAULT_SETTINGS);
        printf(err == SBG_NO_ERROR ? "  Restored + rebooting.\n" : "  FAILED (%d)\n", err);
    }
    else
    {
        printf("  Cancelled.\n");
    }
    flushLine();
}

//======================================================================//
//  Menu loop                                                           //
//======================================================================//

static void runMenu(SbgEComHandle *pHandle)
{
    int running = 1;

    while (running)
    {
        int choice;

        printf("\n================ Ellipse-D Configuration ================\n"
               "  1) Show device info\n"
               "  2) Set motion profile\n"
               "  3) Set IMU alignment & lever arm\n"
               "  4) Set GNSS dual-antenna installation\n"
               "  5) Set aiding assignment (internal GNSS / RTCM port)\n"
               "  6) Configure UART port (baud / RS-232 / RS-422)\n"
               "  7) Apply default output logs on PORT_A\n"
               "  8) Live data monitor\n"
               "  9) SAVE settings + reboot\n"
               " 10) Restore factory defaults + reboot\n"
               "  0) Quit\n"
               "=========================================================\n");
        choice = promptInt("Select", 0);

        switch (choice)
        {
        case 1:  showDeviceInfo(pHandle);        break;
        case 2:  configMotionProfile(pHandle);   break;
        case 3:  configImuAlignment(pHandle);    break;
        case 4:  configDualAntenna(pHandle);     break;
        case 5:  configAidingAssignment(pHandle);break;
        case 6:  configUart(pHandle);            break;
        case 7:  configOutputs(pHandle);         break;
        case 8:  liveMonitor(pHandle);           break;
        case 9:  saveAndReboot(pHandle);         break;
        case 10: restoreDefaults(pHandle);       break;
        case 0:  running = 0;                    break;
        default: printf("  Unknown choice.\n");  break;
        }
    }
}

//======================================================================//
//  Main                                                                //
//======================================================================//

int main(int argc, char **argv)
{
    SbgErrorCode    err;
    SbgInterface    iface;
    SbgEComHandle   handle;

    if (argc != 3)
    {
        printf("Usage: %s <SERIAL_DEVICE> <BAUDRATE>\n", argv[0]);
        printf("Example: %s /dev/ttyUSB0 921600\n", argv[0]);
        return EXIT_FAILURE;
    }

    err = sbgInterfaceSerialCreate(&iface, argv[1], atoi(argv[2]));
    if (err != SBG_NO_ERROR)
    {
        SBG_LOG_ERROR(err, "unable to open serial interface %s", argv[1]);
        return EXIT_FAILURE;
    }

    err = sbgEComInit(&handle, &iface);
    if (err != SBG_NO_ERROR)
    {
        SBG_LOG_ERROR(err, "unable to initialize sbgECom");
        sbgInterfaceDestroy(&iface);
        return EXIT_FAILURE;
    }

    printf("Connected to %s @ %s baud (sbgECom %s)\n", argv[1], argv[2], SBG_E_COM_VERSION_STR);
    showDeviceInfo(&handle);

    runMenu(&handle);

    sbgEComClose(&handle);
    sbgInterfaceDestroy(&iface);
    printf("Disconnected.\n");
    return EXIT_SUCCESS;
}
