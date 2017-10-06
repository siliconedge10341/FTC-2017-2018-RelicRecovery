package org.firstinspires.ftc.teamcode.classes;

import com.qualcomm.hardware.bosch.BNO055IMU;
import com.qualcomm.hardware.bosch.JustLoggingAccelerationIntegrator;

/**
 * Created by Pawan,
 */


public class AdafruitIMU {
    BNO055IMU imu;

    public AdafruitIMU(BNO055IMU hardwaremap){
        imu = hardwaremap;

    }


}
