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
    AdafruitIMU imu = new AdafruitIMU();

    public void runOpMode(){
        imu = new AdafruitIMU(hardwareMap.get(BNO055IMU.class, "imu"));

        imu.init();

        waitForStart();
        imu.start();

        while (opModeIsActive()){
            telemetry.addData("Heading: ", imu.getHeading());
            telemetry.addData("Velocity X" , imu.getVelocityX());
            telemetry.addData("Velocity Y" , imu.getVelocityY());
            telemetry.addData("Velocity Z" , imu.getVelocityZ());
            telemetry.addData("Acceleration X" , imu.getAccelX());
            telemetry.addData("Acceleration Z" , imu.getAccelZ());
            telemetry.addData("Gravity" , imu.getGravity());
            telemetry.update();
        }
    }
}
