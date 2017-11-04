package org.firstinspires.ftc.teamcode;

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

    private Servo armServoL;
    private Servo armServoR;
    double ServoposL = (Servo.MIN_POSITION);
    double ServoposR = (Servo.MAX_POSITION);


    double Ch1;
    double Ch3;
    double Ch4 ;


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

        armServoL = hardwareMap.servo.get("arm_servoL");
        armServoL.setPosition(ServoposL);

        armServoR = hardwareMap.servo.get("arm_servoR");
        armServoR.setPosition(ServoposR);


        // Servos


    }

    // loop
    @Override
    public void loop() {
        //-------------KEY--------------//
        //   Start-B Runs the Program   //
        //----GAMEPAD 2 FUNCTION KEY----//
        //          -GAMEPAD-           //
        // UP: none;					//
        // RIGHT: Move Servo Right      //
        // LEFT: Move Servo Left        //
        // DOWN: None;                  //
        //          -BUTTONS-           //
        // A: Runs Conveyor 	        //
        // B: Runs Shooter              //
        // X: Raise Slide               //
        // Y: Lower Slide               //
        //         -TRIGGERED-          //
        // LEFT: None;                  //
        // RIGHT: None;                 //
        //------------------------------//
        //   Start-A Runs the Program   //
        //----GAMEPAD 1 FUNCTION KEY----//
        //         -GAMEPAD L-          //
        // UP: Forward					//
        // RIGHT: Strafe Right		    //
        // LEFT: Strafe Left	        //
        // DOWN: Backward				//
        //         -GAMEPAD R-          //
        // UP: none;					//
        // RIGHT: Turn right			//
        // LEFT: Turn left				//
        // DOWN: none;					//
        //          -BUTTONS-           //
        // A: Runs the Collector		//
        // B: none;             		//
        // X: none;      		        //
        // Y: none;		                //
        //         -TRIGGERED-          //
        // LEFT: Button goes down       //
        // RIGHT: Button goes up        //
        //------------------------------//
        //
        // Power Settings
        //

        // runs the robot
        //

        Ch1 = gamepad1.right_stick_x;
        Ch3 = gamepad1.left_stick_y;
        Ch4 = gamepad1.left_stick_x;

        motorFR.setPower( -(Ch3 - Ch1 - Ch4));
        motorFL.setPower( -(Ch3 + Ch1 + Ch4));


        motorBR.setPower(-(Ch3 - Ch1 + Ch4));
        motorBL.setPower( -(Ch3 + Ch1 - Ch4));

        if(gamepad1.dpad_left){
            ServoposL += .01;
            ServoposR -= .01;
        }else if(gamepad1.dpad_right){
            ServoposL -= .01;
            ServoposR += .01;
        }
        if (ServoposL > Servo.MAX_POSITION){
            ServoposL = Servo.MAX_POSITION;
        }
        if (ServoposL < Servo.MIN_POSITION){
            ServoposL = Servo.MIN_POSITION;
        }
        if (ServoposR > Servo.MAX_POSITION){
            ServoposR = Servo.MAX_POSITION;
        }
        if (ServoposR < Servo.MIN_POSITION){
            ServoposR = Servo.MIN_POSITION;
        }
        armServoL.setPosition(ServoposL);
        armServoR.setPosition(ServoposR);

        telemetry.addData("Servopos Left:" , ServoposL);
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