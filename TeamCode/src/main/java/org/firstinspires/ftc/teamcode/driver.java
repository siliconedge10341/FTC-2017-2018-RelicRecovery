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
    double ServoposR = 0;
    double ServoposR1 = 1;

    ElapsedTime time = new ElapsedTime();

    double Ch1;
    double Ch3;
    double Ch4 ;

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
        armServoL1.setPosition(ServoposL);

        armServoL2 = hardwareMap.servo.get("arm_servoL2");
        armServoL2.setPosition(ServoposL);

        armServoR2 = hardwareMap.servo.get("arm_servoR2");
        armServoR2.setPosition(ServoposL);

        armServoR1 = hardwareMap.servo.get("arm_servoR1");
        armServoR1.setPosition(ServoposL);

        pressed = false;
        endtime = 0;
        speedcoef = 1.0;

    }

    // loop
    @Override
    public void loop() {


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
            ServoposL = .38;
            ServoposR = .7;
        }if(gamepad1.left_trigger>.8){
            ServoposL1 = 1;
            ServoposR1 = 0;
        }else if(gamepad1.right_trigger>.8){
            ServoposL1 = .38;
            ServoposR1 = .7;
        }
        armServoL1.setPosition(ServoposL);
        armServoL2.setPosition(ServoposL1);
        armServoR1.setPosition(ServoposR);
        armServoR2.setPosition(ServoposR1);

       if(gamepad1.a){
            speedcoef = .5;
        }else{
           speedcoef = 1;
       }

        telemetry.addData("Servopos Left:" , ServoposL);
        telemetry.addData("Servopos Right: " , ServoposR);
        telemetry.addData("Speed coeff" , speedcoef);
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

