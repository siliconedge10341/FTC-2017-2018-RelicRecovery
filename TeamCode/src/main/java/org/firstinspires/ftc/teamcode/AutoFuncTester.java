package org.firstinspires.ftc.teamcode;

import com.qualcomm.hardware.bosch.BNO055IMU;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.DcMotor;

import org.firstinspires.ftc.robotcore.external.navigation.RelicRecoveryVuMark;
import org.firstinspires.ftc.robotcore.external.navigation.VuforiaLocalizer;
import org.firstinspires.ftc.robotcore.external.navigation.VuforiaTrackable;
import org.firstinspires.ftc.robotcore.external.navigation.VuforiaTrackables;
import org.firstinspires.ftc.teamcode.classes.AdafruitIMU;
import org.firstinspires.ftc.teamcode.classes.Mecanum;


/**
 * Created by vatty on 9/15/2017.
 */
@Autonomous(name="Auto Fund tester", group="Push")

public class AutoFuncTester extends LinearOpMode {

    private DcMotor motorFR;
    private DcMotor motorFL;
    private DcMotor motorBR;
    private DcMotor motorBL;

    //Mecanum
    Mecanum bot = new Mecanum();

    //Camera initialize
    VuforiaLocalizer vuforia;

    //Gyro Initialize
    AdafruitIMU imu = new AdafruitIMU();

    private static final Double ticks_per_inch = 510 / (3.1415 * 4);
    private static final Double CORRECTION = .04;
    private static final Double THRESHOLD = 2.0;

    public void runOpMode(){
        //motors
        motorFL = hardwareMap.dcMotor.get("fl");
        motorFR = hardwareMap.dcMotor.get("fr");
        motorBL = hardwareMap.dcMotor.get("bl");
        motorBR = hardwareMap.dcMotor.get("br");

        //Camera setup

        //Mecanum
        bot = new Mecanum(motorFR,motorFL,motorBR,motorBL);

        //IMU
        AdafruitIMU imu = new AdafruitIMU(hardwareMap.get(BNO055IMU.class, "imu"));

        imu.init();

        waitForStart();

        imu.start();


        encoderDrive(5,"forward",.5);



        //gyroTurn(90,"left",.4);

        //STATE FIVE: GO TO MOUNTAIN

        //gyroDrive(20,"forward",.4);

        //STATE SIX: STACK BLOCK



    }

    public void encoderDrive(double inches, String direction , double power ) {
        int encoderval;
        //
        // Sets the encoders
        //
        bot.reset_encoders();
        encoderval = ticks_per_inch.intValue() * (int) inches;
        bot.run_using_encoders();
        //
        // Uses the encoders and motors to set the specific position
        //
        bot.setPosition(encoderval,encoderval,encoderval,encoderval);
        //
        // Sets the power and direction
        //
        bot.setPowerD(power);
        if (direction == "forward"){
            bot.run_forward();
        } else if(direction == "backward"){
            bot.run_backward();
        } else if (direction == "left"){
            bot.run_left();
        } else if (direction == "right"){
            bot.run_right();
        } else if (direction == "diagonal_left_up"){
            bot.run_diagonal_left_up();
        }

        bot.brake();
        bot.reset_encoders();
    }

    public void gyroDrive(double inches, String direction , double power){
        int encoderval;
        double firstheading = imu.getHeading();
        double headingChange= 0;

        // Sets the encoders
        bot.reset_encoders();
        encoderval = ticks_per_inch.intValue() * (int) inches;
        bot.run_to_position();
        // Uses the encoders and motors to set the specific position

        bot.setPosition(encoderval,encoderval,encoderval,encoderval);

        // Sets the power and direction

        bot.setPowerD(power);

        if (direction == "forward"){
            bot.run_forward();
        } else if(direction == "backward") {
            bot.run_backward();
        }
        while(bot.isBusy()){
            headingChange = imu.getHeading() - firstheading;
            if(headingChange<-THRESHOLD){
                bot.drive_forward_gyro(power + (Math.abs(headingChange * CORRECTION)), power);
            }else if(headingChange>THRESHOLD){
                bot.drive_forward_gyro(power, power + (Math.abs(headingChange * CORRECTION)));
            }

        }

        bot.brake();
        bot.reset_encoders();
    }

    public void gyroTurn(double angle, String direction, double power){
        double aheading = Math.abs(imu.getHeading()) + angle;
        bot.setPowerD(power);

        if (direction == "left"){
            bot.turn_left();
        }else if(direction == "right"){
            bot.turn_right();
        }
        while(bot.isBusy()){
            if(Math.abs(imu.getHeading()) >= (aheading - THRESHOLD)&&(Math.abs(imu.getHeading()) <= (aheading + THRESHOLD))){
                bot.brake();
            }
        }

    }

}
