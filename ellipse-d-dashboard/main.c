/*!
 * \file    main.c
 * \brief   Full-screen live dashboard + configuration for the SBG Ellipse-D.
 *
 * A single binary that:
 *   - draws an auto-refreshing terminal dashboard of the live INS solution
 *     (orientation, position, velocity, GNSS/RTK, dual-antenna heading, IMU,
 *      EKF solution mode, health flags, per-log data rates), and
 *   - lets you reconfigure the device on the fly (press 'c').
 *
 * Implemented with plain ANSI/VT100 escape codes (no ncurses dependency) and
 * the binary sbgECom command API.
 *
 * Usage:   ellipse-d-dashboard <SERIAL_DEVICE> <BAUDRATE>
 * Example: ellipse-d-dashboard /dev/ttyUSB0 921600
 */

#include <signal.h>
#include <termios.h>
#include <unistd.h>
#include <fcntl.h>
#include <time.h>

#include <sbgCommon.h>
#include <version/sbgVersion.h>
#include <sbgEComLib.h>

//======================================================================//
//  ANSI / VT100 helpers                                                //
//======================================================================//

#define ANSI_CLEAR      "\033[2J"
#define ANSI_HOME       "\033[H"
#define ANSI_HIDE_CUR   "\033[?25l"
#define ANSI_SHOW_CUR   "\033[?25h"
#define ANSI_RESET      "\033[0m"
#define ANSI_BOLD       "\033[1m"
#define ANSI_DIM        "\033[2m"
#define C_RED           "\033[31m"
#define C_GREEN         "\033[32m"
#define C_YELLOW        "\033[33m"
#define C_BLUE          "\033[34m"
#define C_CYAN          "\033[36m"
#define C_WHITE         "\033[37m"

#define C_DIM_LINE ANSI_DIM "  ─────────────────────────────────────────────────────────────────────────" ANSI_RESET

static void moveTo(int row, int col)   { printf("\033[%d;%dH", row, col); }
static void clearEol(void)             { printf("\033[K"); }

//======================================================================//
//  Terminal raw-mode management (for non-blocking single-key input)    //
//======================================================================//

static struct termios   gSavedTermios;

static void rawModeEnable(void)
{
    struct termios raw;

    tcgetattr(STDIN_FILENO, &gSavedTermios);
    raw = gSavedTermios;
    raw.c_lflag &= ~(ICANON | ECHO);            // no line buffering, no echo
    raw.c_cc[VMIN]  = 0;                        // non-blocking read
    raw.c_cc[VTIME] = 0;
    tcsetattr(STDIN_FILENO, TCSANOW, &raw);
}

static void rawModeDisable(void)
{
    tcsetattr(STDIN_FILENO, TCSANOW, &gSavedTermios);
}

static int readKey(void)
{
    unsigned char c;
    if (read(STDIN_FILENO, &c, 1) == 1)
    {
        return c;
    }
    return -1;
}

//======================================================================//
//  Live state — the callback only stores; drawing happens in the loop  //
//======================================================================//

typedef struct
{
    // Orientation
    float       euler[3], eulerStd[3];
    uint32_t    ekfStatus;
    bool        haveEuler;

    // Navigation
    double      navPos[3];
    float       navVel[3], navPosStd[3];
    bool        haveNav;

    // GNSS position / RTK
    int         gnssPosType;
    uint8_t     gnssNumSvUsed;
    double      gnssLat, gnssLon, gnssAlt;
    bool        haveGnss;

    // Dual-antenna heading
    int         hdtStatus;
    float       hdtHeading, hdtPitch, hdtHeadingAcc;
    uint8_t     hdtNumSvUsed;
    bool        haveHdt;

    // IMU
    float       accel[3], gyro[3], imuTemp;
    bool        haveImu;

    // Device status
    uint16_t    generalStatus;
    bool        haveStatus;

    // Counters / rates
    uint32_t    cnt[8];          // indexed by an internal id
    uint32_t    cntPrev[8];
    double      rate[8];
} LiveState;

enum { K_EULER = 0, K_NAV, K_GNSS, K_HDT, K_IMU, K_STATUS, K_TOTAL };

static LiveState               gState;
static volatile sig_atomic_t   gResize = 0;

static double nowSeconds(void)
{
    struct timespec ts;
    clock_gettime(CLOCK_MONOTONIC, &ts);
    return (double)ts.tv_sec + (double)ts.tv_nsec * 1e-9;
}

static SbgErrorCode onLog(SbgEComHandle *pHandle, SbgEComClass cls, SbgEComMsgId msg,
                          const SbgEComLogUnion *pLog, void *pUser)
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
        for (int i = 0; i < 3; i++)
        {
            gState.euler[i]    = pLog->ekfEulerData.euler[i];
            gState.eulerStd[i] = pLog->ekfEulerData.eulerStdDev[i];
        }
        gState.ekfStatus = pLog->ekfEulerData.status;
        gState.haveEuler = true;
        gState.cnt[K_EULER]++;
        break;

    case SBG_ECOM_LOG_EKF_NAV:
        for (int i = 0; i < 3; i++)
        {
            gState.navPos[i]    = pLog->ekfNavData.position[i];
            gState.navVel[i]    = pLog->ekfNavData.velocity[i];
            gState.navPosStd[i] = pLog->ekfNavData.positionStdDev[i];
        }
        gState.ekfStatus = pLog->ekfNavData.status;
        gState.haveNav   = true;
        gState.cnt[K_NAV]++;
        break;

    case SBG_ECOM_LOG_GPS1_POS:
        gState.gnssPosType   = (int)sbgEComLogGnssPosGetType(&pLog->gpsPosData);
        gState.gnssNumSvUsed = pLog->gpsPosData.numSvUsed;
        gState.gnssLat       = pLog->gpsPosData.latitude;
        gState.gnssLon       = pLog->gpsPosData.longitude;
        gState.gnssAlt       = pLog->gpsPosData.altitude;
        gState.haveGnss      = true;
        gState.cnt[K_GNSS]++;
        break;

    case SBG_ECOM_LOG_GPS1_HDT:
        gState.hdtStatus     = (int)sbgEComLogGnssHdtGetStatus(&pLog->gpsHdtData);
        gState.hdtHeading    = pLog->gpsHdtData.heading;
        gState.hdtPitch      = pLog->gpsHdtData.pitch;
        gState.hdtHeadingAcc = pLog->gpsHdtData.headingAccuracy;
        gState.hdtNumSvUsed  = pLog->gpsHdtData.numSvUsed;
        gState.haveHdt       = true;
        gState.cnt[K_HDT]++;
        break;

    case SBG_ECOM_LOG_IMU_DATA:
        for (int i = 0; i < 3; i++)
        {
            gState.accel[i] = pLog->imuData.accelerometers[i];
            gState.gyro[i]  = pLog->imuData.gyroscopes[i];
        }
        gState.imuTemp = pLog->imuData.temperature;
        gState.haveImu = true;
        gState.cnt[K_IMU]++;
        break;

    case SBG_ECOM_LOG_STATUS:
        gState.generalStatus = pLog->statusData.generalStatus;
        gState.haveStatus    = true;
        gState.cnt[K_STATUS]++;
        break;

    default:
        break;
    }

    return SBG_NO_ERROR;
}

//======================================================================//
//  Decoders -> human-readable strings                                  //
//======================================================================//

static const char *solutionModeStr(uint32_t status)
{
    switch (sbgEComLogEkfGetSolutionMode(status))
    {
    case SBG_ECOM_SOL_MODE_UNINITIALIZED: return "UNINITIALIZED";
    case SBG_ECOM_SOL_MODE_VERTICAL_GYRO: return "VERTICAL GYRO";
    case SBG_ECOM_SOL_MODE_AHRS:          return "AHRS";
    case SBG_ECOM_SOL_MODE_NAV_VELOCITY:  return "NAV VELOCITY";
    case SBG_ECOM_SOL_MODE_NAV_POSITION:  return "NAV POSITION";
    default:                              return "?";
    }
}

static const char *gnssTypeStr(int t)
{
    switch (t)
    {
    case SBG_ECOM_GNSS_POS_TYPE_NO_SOLUTION: return "NO SOLUTION";
    case SBG_ECOM_GNSS_POS_TYPE_UNKNOWN:     return "UNKNOWN";
    case SBG_ECOM_GNSS_POS_TYPE_SINGLE:      return "SINGLE";
    case SBG_ECOM_GNSS_POS_TYPE_PSRDIFF:     return "PSRDIFF/DGPS";
    case SBG_ECOM_GNSS_POS_TYPE_SBAS:        return "SBAS";
    case SBG_ECOM_GNSS_POS_TYPE_OMNISTAR:    return "OMNISTAR";
    case SBG_ECOM_GNSS_POS_TYPE_RTK_FLOAT:   return "RTK FLOAT";
    case SBG_ECOM_GNSS_POS_TYPE_RTK_INT:     return "RTK FIXED";
    case SBG_ECOM_GNSS_POS_TYPE_PPP_FLOAT:   return "PPP FLOAT";
    case SBG_ECOM_GNSS_POS_TYPE_PPP_INT:     return "PPP FIXED";
    case SBG_ECOM_GNSS_POS_TYPE_FIXED:       return "FIXED POS";
    default:                                 return "?";
    }
}

// Colorize a "valid/ok" boolean flag.
static const char *flag(bool ok) { return ok ? C_GREEN "OK " ANSI_RESET : C_RED "-- " ANSI_RESET; }
static const char *chk(bool ok)  { return ok ? C_GREEN "YES" ANSI_RESET : C_RED "no " ANSI_RESET; }

//======================================================================//
//  Dashboard drawing                                                   //
//======================================================================//

static char gDevProduct[33] = "?";
static char gDevFw[32]      = "?";
static char gDevSn[16]      = "?";
static const char *gPort;
static const char *gBaud;

static void drawStaticFrame(void)
{
    printf(ANSI_CLEAR ANSI_HOME ANSI_HIDE_CUR);

    moveTo(1, 1);
    printf(ANSI_BOLD C_CYAN "  SBG Ellipse-D  —  Live Dashboard" ANSI_RESET
           ANSI_DIM "   (sbgECom %s)" ANSI_RESET, SBG_E_COM_VERSION_STR);

    moveTo(2, 1);
    printf(ANSI_DIM "  %s  SN:%s  FW:%s   |   %s @ %s baud" ANSI_RESET,
           gDevProduct, gDevSn, gDevFw, gPort, gBaud);

    moveTo(3, 1);
    printf(C_DIM_LINE);
}

static void drawDashboard(void)
{
    // --- Solution mode line (color by quality) -------------------------
    moveTo(4, 3);
    clearEol();
    if (gState.haveEuler || gState.haveNav)
    {
        SbgEComSolutionMode mode = sbgEComLogEkfGetSolutionMode(gState.ekfStatus);
        const char *col = (mode >= SBG_ECOM_SOL_MODE_NAV_VELOCITY) ? C_GREEN
                        : (mode >= SBG_ECOM_SOL_MODE_AHRS)         ? C_YELLOW : C_RED;
        printf(ANSI_BOLD "EKF SOLUTION: %s%-14s" ANSI_RESET "  "
               "ATT[%s] HDG[%s] VEL[%s] POS[%s]",
               col, solutionModeStr(gState.ekfStatus),
               chk(gState.ekfStatus & SBG_ECOM_SOL_ATTITUDE_VALID),
               chk(gState.ekfStatus & SBG_ECOM_SOL_HEADING_VALID),
               chk(gState.ekfStatus & SBG_ECOM_SOL_VELOCITY_VALID),
               chk(gState.ekfStatus & SBG_ECOM_SOL_POSITION_VALID));
    }
    else
    {
        printf(ANSI_DIM "waiting for EKF data… (enable outputs via 'c' if blank)" ANSI_RESET);
    }

    // --- ORIENTATION ---------------------------------------------------
    moveTo(6, 3);  printf(ANSI_BOLD C_BLUE "ORIENTATION" ANSI_RESET);
    moveTo(7, 5);  clearEol();
    moveTo(8, 5);  clearEol();
    moveTo(9, 5);  clearEol();
    if (gState.haveEuler)
    {
        const char *lbl[3] = { "Roll ", "Pitch", "Yaw  " };
        for (int i = 0; i < 3; i++)
        {
            moveTo(7 + i, 5);
            printf("%s : % 8.2f °   ±%5.2f", lbl[i],
                   sbgRadToDegf(gState.euler[i]), sbgRadToDegf(gState.eulerStd[i]));
        }
    }
    else { moveTo(7, 5); printf(ANSI_DIM "no data" ANSI_RESET); }

    // --- POSITION ------------------------------------------------------
    moveTo(6, 42); printf(ANSI_BOLD C_BLUE "POSITION (EKF)" ANSI_RESET);
    for (int i = 0; i < 3; i++) { moveTo(7 + i, 44); clearEol(); }
    if (gState.haveNav)
    {
        moveTo(7, 44); printf("Lat : % 12.7f °", gState.navPos[0]);
        moveTo(8, 44); printf("Lon : % 12.7f °", gState.navPos[1]);
        moveTo(9, 44); printf("Alt : % 9.2f m   ±%4.2f", gState.navPos[2], gState.navPosStd[2]);
    }
    else { moveTo(7, 44); printf(ANSI_DIM "no data" ANSI_RESET); }

    // --- VELOCITY ------------------------------------------------------
    moveTo(11, 3); printf(ANSI_BOLD C_BLUE "VELOCITY (NED, m/s)" ANSI_RESET);
    moveTo(12, 5); clearEol();
    if (gState.haveNav)
    {
        moveTo(12, 5);
        printf("N: % 7.2f   E: % 7.2f   D: % 7.2f", gState.navVel[0], gState.navVel[1], gState.navVel[2]);
    }
    else { moveTo(12, 5); printf(ANSI_DIM "no data" ANSI_RESET); }

    // --- GNSS / RTK ----------------------------------------------------
    moveTo(11, 42); printf(ANSI_BOLD C_BLUE "GNSS / RTK" ANSI_RESET);
    moveTo(12, 44); clearEol();
    moveTo(13, 44); clearEol();
    if (gState.haveGnss)
    {
        const char *col = (gState.gnssPosType == SBG_ECOM_GNSS_POS_TYPE_RTK_INT) ? C_GREEN
                        : (gState.gnssPosType == SBG_ECOM_GNSS_POS_TYPE_RTK_FLOAT) ? C_YELLOW
                        : (gState.gnssPosType <= SBG_ECOM_GNSS_POS_TYPE_UNKNOWN) ? C_RED : C_WHITE;
        moveTo(12, 44); printf("Fix : %s%-12s" ANSI_RESET "  SV used: %u",
                               col, gnssTypeStr(gState.gnssPosType), gState.gnssNumSvUsed);
        moveTo(13, 44); printf(ANSI_DIM "%.6f, %.6f" ANSI_RESET, gState.gnssLat, gState.gnssLon);
    }
    else { moveTo(12, 44); printf(ANSI_DIM "no data" ANSI_RESET); }

    // --- DUAL-ANTENNA HEADING -----------------------------------------
    moveTo(15, 3); printf(ANSI_BOLD C_BLUE "DUAL-ANTENNA HEADING" ANSI_RESET);
    moveTo(16, 5); clearEol();
    if (gState.haveHdt)
    {
        const char *col = (gState.hdtStatus == SBG_ECOM_GNSS_HDT_STATUS_SOL_COMPUTED) ? C_GREEN : C_RED;
        moveTo(16, 5);
        printf("Hdg: %s% 7.2f °" ANSI_RESET "  ±%4.2f   Pitch: % 6.2f °  SV:%u",
               col, gState.hdtHeading, gState.hdtHeadingAcc, gState.hdtPitch, gState.hdtNumSvUsed);
    }
    else { moveTo(16, 5); printf(ANSI_DIM "no data (needs GPS1_HDT output enabled)" ANSI_RESET); }

    // --- IMU -----------------------------------------------------------
    moveTo(15, 42); printf(ANSI_BOLD C_BLUE "IMU" ANSI_RESET);
    moveTo(16, 44); clearEol();
    moveTo(17, 44); clearEol();
    if (gState.haveImu)
    {
        moveTo(16, 44); printf("a[m/s2]: % 6.2f % 6.2f % 6.2f", gState.accel[0], gState.accel[1], gState.accel[2]);
        moveTo(17, 44); printf("g[°/s] : % 6.1f % 6.1f % 6.1f  %.1f°C",
                               sbgRadToDegf(gState.gyro[0]), sbgRadToDegf(gState.gyro[1]),
                               sbgRadToDegf(gState.gyro[2]), gState.imuTemp);
    }
    else { moveTo(16, 44); printf(ANSI_DIM "no data" ANSI_RESET); }

    // --- HEALTH FLAGS --------------------------------------------------
    moveTo(18, 3); clearEol();
    if (gState.haveStatus)
    {
        uint16_t g = gState.generalStatus;
        printf(ANSI_BOLD "HEALTH " ANSI_RESET "MainPwr[%s] IMU[%s] GPSpwr[%s] Settings[%s] Temp[%s]",
               flag(g & SBG_ECOM_GENERAL_MAIN_POWER_OK),
               flag(g & SBG_ECOM_GENERAL_IMU_POWER_OK),
               flag(g & SBG_ECOM_GENERAL_GPS_POWER_OK),
               flag(g & SBG_ECOM_GENERAL_SETTINGS_OK),
               flag(g & SBG_ECOM_GENERAL_TEMPERATURE_OK));
    }

    // --- DATA RATES ----------------------------------------------------
    moveTo(20, 3); clearEol();
    printf(ANSI_DIM "RATES [Hz]  " ANSI_RESET
           "IMU %5.1f  Euler %5.1f  Nav %5.1f  GPSpos %5.1f  HDT %5.1f  Status %5.1f",
           gState.rate[K_IMU], gState.rate[K_EULER], gState.rate[K_NAV],
           gState.rate[K_GNSS], gState.rate[K_HDT], gState.rate[K_STATUS]);

    // --- FOOTER --------------------------------------------------------
    moveTo(22, 1); printf(C_DIM_LINE);
    moveTo(23, 3);
    printf(ANSI_BOLD "[c]" ANSI_RESET " configure   "
           ANSI_BOLD "[z]" ANSI_RESET " zero rates   "
           ANSI_BOLD "[q]" ANSI_RESET " quit");
    moveTo(24, 1);  // park cursor
    fflush(stdout);
}

//======================================================================//
//  Configuration submenu (drops to cooked mode)                        //
//======================================================================//

static int promptInt(const char *label, int defValue)
{
    char buf[64];
    printf("%s [%d]: ", label, defValue);
    fflush(stdout);
    if (fgets(buf, sizeof(buf), stdin) && buf[0] != '\n') return atoi(buf);
    return defValue;
}
static float promptFloat(const char *label, float defValue)
{
    char buf[64];
    printf("%s [%.4f]: ", label, defValue);
    fflush(stdout);
    if (fgets(buf, sizeof(buf), stdin) && buf[0] != '\n') return (float)atof(buf);
    return defValue;
}

static void cfgMotionProfile(SbgEComHandle *h)
{
    printf("\nProfiles: 1 General 2 Automotive 3 Marine 4 Airplane 5 Helicopter\n"
           "          6 Pedestrian 7 UAV 8 HeavyMachinery 9 Static 10 Truck\n");
    int sel = promptInt("Motion profile", SBG_ECOM_MOTION_PROFILE_GENERAL_PURPOSE);
    SbgErrorCode e = sbgEComCmdSensorSetMotionProfileId(h, (SbgEComMotionProfileStdIds)sel);
    printf(e == SBG_NO_ERROR ? "  OK\n" : "  FAILED (%d)\n", e);
}

static void cfgDualAntenna(SbgEComHandle *h)
{
    SbgEComGnssInstallation g;
    if (sbgEComCmdGnss1InstallationGet(h, &g) != SBG_NO_ERROR) { printf("  read failed\n"); return; }
    printf("\nPrimary antenna lever arm (IMU->primary, m):\n");
    g.leverArmPrimary[0] = promptFloat("  X", g.leverArmPrimary[0]);
    g.leverArmPrimary[1] = promptFloat("  Y", g.leverArmPrimary[1]);
    g.leverArmPrimary[2] = promptFloat("  Z", g.leverArmPrimary[2]);
    printf("Secondary antenna lever arm (IMU->secondary, m):\n");
    g.leverArmSecondary[0] = promptFloat("  X", g.leverArmSecondary[0]);
    g.leverArmSecondary[1] = promptFloat("  Y", g.leverArmSecondary[1]);
    g.leverArmSecondary[2] = promptFloat("  Z", g.leverArmSecondary[2]);
    g.leverArmSecondaryMode = SBG_ECOM_GNSS_INSTALLATION_MODE_DUAL_PRECISE;
    g.leverArmPrimaryPrecise = true;
    SbgErrorCode e = sbgEComCmdGnss1InstallationSet(h, &g);
    printf(e == SBG_NO_ERROR ? "  OK (dual precise)\n" : "  FAILED (%d)\n", e);
}

static void cfgOutputs(SbgEComHandle *h)
{
    struct { SbgEComMsgId id; SbgEComOutputMode mode; const char *n; } o[] = {
        { SBG_ECOM_LOG_IMU_DATA,  SBG_ECOM_OUTPUT_MODE_DIV_8,  "IMU_DATA 25Hz" },
        { SBG_ECOM_LOG_EKF_EULER, SBG_ECOM_OUTPUT_MODE_DIV_8,  "EKF_EULER 25Hz" },
        { SBG_ECOM_LOG_EKF_NAV,   SBG_ECOM_OUTPUT_MODE_DIV_8,  "EKF_NAV 25Hz" },
        { SBG_ECOM_LOG_GPS1_POS,  SBG_ECOM_OUTPUT_MODE_DIV_40, "GPS1_POS 5Hz" },
        { SBG_ECOM_LOG_GPS1_HDT,  SBG_ECOM_OUTPUT_MODE_DIV_40, "GPS1_HDT 5Hz" },
        { SBG_ECOM_LOG_STATUS,    SBG_ECOM_OUTPUT_MODE_DIV_40, "STATUS 5Hz" },
    };
    printf("\nEnabling default outputs on PORT_A:\n");
    for (size_t i = 0; i < SBG_ARRAY_SIZE(o); i++)
    {
        SbgErrorCode e = sbgEComCmdOutputSetConf(h, SBG_ECOM_OUTPUT_PORT_A,
                                                 SBG_ECOM_CLASS_LOG_ECOM_0, o[i].id, o[i].mode);
        printf("  %-16s %s\n", o[i].n, e == SBG_NO_ERROR ? "OK" : "FAIL");
    }
}

static void cfgSave(SbgEComHandle *h)
{
    printf("\nSave to flash + reboot? (y/N): ");
    int c = getchar();
    while (getchar() != '\n' && c != EOF) { }
    if (c == 'y')
    {
        SbgErrorCode e = sbgEComCmdSettingsAction(h, SBG_ECOM_SAVE_SETTINGS);
        printf(e == SBG_NO_ERROR ? "  Saved + rebooting.\n" : "  FAILED (%d)\n", e);
    }
    else printf("  Cancelled.\n");
}

static void configMenu(SbgEComHandle *h)
{
    rawModeDisable();
    printf(ANSI_SHOW_CUR ANSI_CLEAR ANSI_HOME);

    int run = 1;
    while (run)
    {
        printf("\n--- CONFIGURE Ellipse-D ---\n"
               "  1) Motion profile\n"
               "  2) GNSS dual-antenna installation\n"
               "  3) Apply default output logs (PORT_A)\n"
               "  4) SAVE settings + reboot\n"
               "  0) Back to dashboard\n");
        int sel = promptInt("Select", 0);
        switch (sel)
        {
        case 1: cfgMotionProfile(h); break;
        case 2: cfgDualAntenna(h);   break;
        case 3: cfgOutputs(h);       break;
        case 4: cfgSave(h);          break;
        case 0: run = 0;             break;
        default: printf("  ?\n");    break;
        }
    }

    rawModeEnable();
    drawStaticFrame();
}

//======================================================================//
//  Main                                                                //
//======================================================================//

static SbgEComHandle  *gHandleForCleanup = NULL;
static SbgInterface   *gIfaceForCleanup  = NULL;

static void cleanupTerminal(void)
{
    rawModeDisable();
    printf(ANSI_SHOW_CUR ANSI_RESET "\n");
    fflush(stdout);
}

static void onSig(int sig) { SBG_UNUSED_PARAMETER(sig); cleanupTerminal(); _exit(0); }

int main(int argc, char **argv)
{
    SbgErrorCode    err;
    SbgInterface    iface;
    SbgEComHandle   handle;
    SbgEComDeviceInfo info;

    if (argc != 3)
    {
        printf("Usage: %s <SERIAL_DEVICE> <BAUDRATE>\n", argv[0]);
        printf("Example: %s /dev/ttyUSB0 921600\n", argv[0]);
        return EXIT_FAILURE;
    }
    gPort = argv[1];
    gBaud = argv[2];

    err = sbgInterfaceSerialCreate(&iface, argv[1], atoi(argv[2]));
    if (err != SBG_NO_ERROR) { SBG_LOG_ERROR(err, "cannot open %s", argv[1]); return EXIT_FAILURE; }

    err = sbgEComInit(&handle, &iface);
    if (err != SBG_NO_ERROR) { SBG_LOG_ERROR(err, "sbgEComInit failed"); sbgInterfaceDestroy(&iface); return EXIT_FAILURE; }

    gHandleForCleanup = &handle;
    gIfaceForCleanup  = &iface;

    if (sbgEComCmdGetInfo(&handle, &info) == SBG_NO_ERROR)
    {
        char fw[32];
        sbgVersionToStringEncoded(info.firmwareRev, fw, sizeof(fw));
        snprintf(gDevProduct, sizeof(gDevProduct), "%s", info.productCode);
        snprintf(gDevFw, sizeof(gDevFw), "%s", fw);
        snprintf(gDevSn, sizeof(gDevSn), "%09" PRIu32, info.serialNumber);
    }

    signal(SIGINT, onSig);
    signal(SIGTERM, onSig);
    atexit(cleanupTerminal);

    rawModeEnable();
    drawStaticFrame();

    sbgEComSetReceiveLogCallback(&handle, onLog, NULL);

    double lastDraw = 0.0, lastRate = nowSeconds();

    for (;;)
    {
        err = sbgEComHandle(&handle);
        if (err == SBG_NOT_READY)
        {
            sbgSleep(1);
        }

        // Non-blocking key handling
        int key = readKey();
        if (key == 'q' || key == 'Q') { break; }
        else if (key == 'c' || key == 'C') { configMenu(&handle); }
        else if (key == 'z' || key == 'Z')
        {
            for (int i = 0; i < K_TOTAL; i++) { gState.cnt[i] = gState.cntPrev[i] = 0; }
        }

        double t = nowSeconds();

        // Recompute per-log rates once per second
        if (t - lastRate >= 1.0)
        {
            double dt = t - lastRate;
            for (int i = 0; i < K_TOTAL; i++)
            {
                gState.rate[i]    = (gState.cnt[i] - gState.cntPrev[i]) / dt;
                gState.cntPrev[i] = gState.cnt[i];
            }
            lastRate = t;
        }

        // Redraw at ~12 Hz
        if (t - lastDraw >= 0.08)
        {
            drawDashboard();
            lastDraw = t;
        }
    }

    sbgEComSetReceiveLogCallback(&handle, NULL, NULL);
    sbgEComClose(&handle);
    sbgInterfaceDestroy(&iface);
    cleanupTerminal();
    printf("Disconnected.\n");
    return EXIT_SUCCESS;
}
