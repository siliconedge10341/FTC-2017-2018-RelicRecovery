package org.firstinspires.ftc.teamcode;

import com.qualcomm.hardware.bosch.BNO055IMU;
import com.qualcomm.hardware.modernrobotics.ModernRoboticsI2cRangeSensor;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.Servo;

import com.qualcomm.robotcore.hardware.HardwareDevice;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.teamcode.classes.AdafruitIMU;


@TeleOp(name = "MecanumDrive", group = "Drive")
public class driver extends OpMode{
    // instance variables
    // private variables
    // Motors

    private DcMotor motorFR;
    private DcMotor motorFL;
    private DcMotor motorBR;
    private DcMotor motorBL;

    private Servo armServoL1;
    private Servo armServoL2;
    private Servo armServoR1;
    private Servo armServoR2;

    double ServoposL = 1;
    double ServoposL1 = 1;
    double ServoposR = .7;
    double ServoposR1 = .7;

    //AdafruitIMU imu;

    double Ch1;
    double Ch3;
    double Ch4 ;
    double accel;
    double speedv = 2;
    int endtime = 0;
    boolean pressed;

    double speedcoef;

    // Servos

    // constructors
    public driver() {
        // default constructor

    }

    @Override
    public void init() {
        //
        // Initialize everything
        //
        // Motors
        motorFL = hardwareMap.dcMotor.get("fl");
        motorFR = hardwareMap.dcMotor.get("fr");
        motorBL = hardwareMap.dcMotor.get("bl");
        motorBR = hardwareMap.dcMotor.get("br");

        armServoL1 = hardwareMap.servo.get("arm_servoL1");
        //armServoL1.setPosition(ServoposL);

        armServoL2 = hardwareMap.servo.get("arm_servoL2");
       // armServoL2.setPosition(ServoposL);

        armServoR2 = hardwareMap.servo.get("arm_servoR2");
        //armServoR2.setPosition(ServoposL);

        armServoR1 = hardwareMap.servo.get("arm_servoR1");
        //armServoR1.setPosition(ServoposL);

        armServoL1.setPosition(ServoposL-.29);
        armServoL2.setPosition(ServoposL1-.25);
        armServoR1.setPosition(ServoposR+.29);
        armServoR2.setPosition(ServoposR1+.25);

       // imu = new AdafruitIMU(hardwareMap.get(BNO055IMU.class, "imu"));
       // imu.init();

        pressed = false;
        endtime = 0;
        speedcoef = 1.0;

        accel = 0;

    }

    @Override
    public void start() {
        //imu.start();`
    }

    // loop
    @Override
    public void loop() {
        //accel = Math.sqrt(imu.getAccelX()*imu.getAccelX() + imu.getAccelZ()*imu.getAccelZ() + imu.getAccelY()*imu.getAccelY());

        if (gamepad1.a){
            speedv = 2;                   //fast
        }
        if (gamepad1.b){
            speedv = 1;                   //slow
        }
        Ch1 = gamepad1.right_stick_x;
        Ch3 = gamepad1.left_stick_y;
        Ch4 = gamepad1.left_stick_x;

        motorFR.setPower( speedcoef* -(Ch3 - Ch1 - Ch4));
        motorFL.setPower( speedcoef * (Ch3 + Ch1 + Ch4));


        motorBR.setPower(speedcoef * -(Ch3 - Ch1 + Ch4));
        motorBL.setPower(speedcoef * (Ch3 + Ch1 - Ch4));

        if(gamepad1.left_bumper){
            ServoposL = 1;
            ServoposR = 0;
        }else if(gamepad1.right_bumper){
            ServoposL = .2;
            ServoposR = 1.0;
        }if(gamepad1.left_trigger>.8){
            ServoposL1 = .95;
            ServoposR1 = .08;
        }else if(gamepad1.right_trigger>.8){
            ServoposL1 = .32;
            ServoposR1 = .75;
        }
        armServoL1.setPosition(ServoposL-.29);
        armServoL2.setPosition(ServoposL1-.29);
        armServoR1.setPosition(ServoposR+.35);
        armServoR2.setPosition(ServoposR1+.29);

        if (speedv == 1){
            speedcoef = .5;
        }
        if (speedv == 2){
            speedcoef = 1;
        }
       /*if(gamepad1.a){
            speedcoef = .5;
        }else{
           speedcoef = 1;
       }*/

        telemetry.addData("Servopos Left:" , ServoposL);
        telemetry.addData("Servopos Right: " , ServoposR);
        telemetry.addData("Speed coeff" , speedcoef);
        //telemetry.addData("Acceleration" , accel);
        telemetry.update();

        // Runs the collector
        //



    }

    // functions
    @Override
    public void stop() {

        // set to zero so the power doesn't influence any motion or rotation in the robot

    }

}

