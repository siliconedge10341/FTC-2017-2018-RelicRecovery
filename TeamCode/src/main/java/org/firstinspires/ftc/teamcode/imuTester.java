package org.firstinspires.ftc.teamcode;

import com.qualcomm.hardware.bosch.BNO055IMU;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;

import org.firstinspires.ftc.teamcode.classes.AdafruitIMU;

/**
 * Created by vatty on 9/16/2017.
 */

@Autonomous(name = "imuTester")
public class imuTester extends LinearOpMode{
    AdafruitIMU imu = new AdafruitIMU(hardwareMap.get(BNO055IMU.class, "imu"));

    public void runOpMode(){

        waitForStart();
        imu.start();

        while (opModeIsActive()){
            telemetry.addData("Heading: ", imu.getHeading());
            telemetry.update();
        }
    }
}
