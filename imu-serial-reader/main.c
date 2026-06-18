/*!
 * \file    main.c
 * \brief   Minimal example: open an SBG IMU over serial and print IMU + Euler data.
 *
 * Usage:   imu-serial-reader <SERIAL_DEVICE> <BAUDRATE>
 * Example: imu-serial-reader /dev/ttyUSB0 921600
 */

// sbgCommonLib headers
#include <sbgCommon.h>

// sbgECom headers
#include <sbgEComLib.h>

//----------------------------------------------------------------------//
//- Log reception callback                                             -//
//----------------------------------------------------------------------//

/*!
 * Called every time a new log frame is decoded from the device.
 */
static SbgErrorCode onLogReceived(SbgEComHandle *pHandle, SbgEComClass msgClass,
                                  SbgEComMsgId msg, const SbgEComLogUnion *pLogData,
                                  void *pUserArg)
{
    SBG_UNUSED_PARAMETER(pHandle);
    SBG_UNUSED_PARAMETER(pUserArg);
    assert(pLogData);

    if (msgClass == SBG_ECOM_CLASS_LOG_ECOM_0)
    {
        switch (msg)
        {
        case SBG_ECOM_LOG_IMU_DATA:
            //
            // Raw inertial data: 3-axis accelerometer (m/s^2) and gyroscope (rad/s)
            //
            printf("IMU   accel[m/s2]: % 8.3f % 8.3f % 8.3f   gyro[deg/s]: % 8.3f % 8.3f % 8.3f\n",
                   pLogData->imuData.accelerometers[0],
                   pLogData->imuData.accelerometers[1],
                   pLogData->imuData.accelerometers[2],
                   sbgRadToDegf(pLogData->imuData.gyroscopes[0]),
                   sbgRadToDegf(pLogData->imuData.gyroscopes[1]),
                   sbgRadToDegf(pLogData->imuData.gyroscopes[2]));
            break;

        case SBG_ECOM_LOG_EKF_EULER:
            //
            // Fused orientation estimate: roll / pitch / yaw (deg)
            //
            printf("EULER roll/pitch/yaw[deg]: % 7.2f % 7.2f % 7.2f\n",
                   sbgRadToDegf(pLogData->ekfEulerData.euler[0]),
                   sbgRadToDegf(pLogData->ekfEulerData.euler[1]),
                   sbgRadToDegf(pLogData->ekfEulerData.euler[2]));
            break;

        default:
            break;
        }
    }

    return SBG_NO_ERROR;
}

//----------------------------------------------------------------------//
//- Main                                                               -//
//----------------------------------------------------------------------//

int main(int argc, char **argv)
{
    SbgErrorCode    errorCode;
    SbgInterface    sbgInterface;
    SbgEComHandle   comHandle;

    if (argc != 3)
    {
        printf("Usage: %s <SERIAL_DEVICE> <BAUDRATE>\n", argv[0]);
        printf("Example: %s /dev/ttyUSB0 921600\n", argv[0]);
        return EXIT_FAILURE;
    }

    //
    // Open the serial port (device path + baudrate)
    //
    errorCode = sbgInterfaceSerialCreate(&sbgInterface, argv[1], atoi(argv[2]));

    if (errorCode != SBG_NO_ERROR)
    {
        SBG_LOG_ERROR(errorCode, "unable to open serial interface %s", argv[1]);
        return EXIT_FAILURE;
    }

    //
    // Bind the sbgECom protocol stack to the serial interface
    //
    errorCode = sbgEComInit(&comHandle, &sbgInterface);

    if (errorCode != SBG_NO_ERROR)
    {
        SBG_LOG_ERROR(errorCode, "unable to initialize the sbgECom library");
        sbgInterfaceDestroy(&sbgInterface);
        return EXIT_FAILURE;
    }

    printf("Connected to %s @ %s baud (sbgECom %s)\n\n", argv[1], argv[2], SBG_E_COM_VERSION_STR);

    //
    // Ask the device to stream IMU data and Euler angles on port A at ~25 Hz.
    // (DIV_8 of the 200 Hz main loop = 25 Hz). Warnings only — keep going on failure.
    //
    errorCode = sbgEComCmdOutputSetConf(&comHandle, SBG_ECOM_OUTPUT_PORT_A,
                                        SBG_ECOM_CLASS_LOG_ECOM_0, SBG_ECOM_LOG_IMU_DATA,
                                        SBG_ECOM_OUTPUT_MODE_DIV_8);
    if (errorCode != SBG_NO_ERROR)
    {
        SBG_LOG_WARNING(errorCode, "unable to configure SBG_ECOM_LOG_IMU_DATA");
    }

    errorCode = sbgEComCmdOutputSetConf(&comHandle, SBG_ECOM_OUTPUT_PORT_A,
                                        SBG_ECOM_CLASS_LOG_ECOM_0, SBG_ECOM_LOG_EKF_EULER,
                                        SBG_ECOM_OUTPUT_MODE_DIV_8);
    if (errorCode != SBG_NO_ERROR)
    {
        SBG_LOG_WARNING(errorCode, "unable to configure SBG_ECOM_LOG_EKF_EULER");
    }

    //
    // Register our callback and pump the receive loop forever.
    //
    sbgEComSetReceiveLogCallback(&comHandle, onLogReceived, NULL);

    while (1)
    {
        errorCode = sbgEComHandle(&comHandle);

        if (errorCode == SBG_NOT_READY)
        {
            sbgSleep(1);    // no frame available, yield the CPU briefly
        }
        else if (errorCode != SBG_NO_ERROR)
        {
            SBG_LOG_ERROR(errorCode, "unable to process incoming sbgECom logs");
        }
    }

    sbgEComClose(&comHandle);
    sbgInterfaceDestroy(&sbgInterface);

    return EXIT_SUCCESS;
}
