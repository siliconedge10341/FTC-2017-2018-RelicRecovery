package org.firstinspires.ftc.teamcode;

import com.qualcomm.hardware.bosch.BNO055IMU;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.ColorSensor;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.Servo;

import org.firstinspires.ftc.robotcore.external.ClassFactory;
import org.firstinspires.ftc.robotcore.external.navigation.RelicRecoveryVuMark;
import org.firstinspires.ftc.robotcore.external.navigation.VuforiaLocalizer;
import org.firstinspires.ftc.robotcore.external.navigation.VuforiaTrackable;
import org.firstinspires.ftc.robotcore.external.navigation.VuforiaTrackables;
import org.firstinspires.ftc.teamcode.classes.AdafruitIMU;
import org.firstinspires.ftc.teamcode.classes.Mecanum;

@Autonomous(name="Blue Auto 2", group="Pushbot")
public class BlueAuto2 extends LinearOpMode {

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

    //Color sensor
    ColorSensor sensorColor;

    //Servos
    Servo jewelHitter;

    private static final Double ticks_per_inch = 510 / (3.1415 * 4);
    private static final Double CORRECTION = .04;
    private static final Double THRESHOLD = 2.0;
    Double driveDistance;

    public void runOpMode(){
        //motors
        motorFL = hardwareMap.dcMotor.get("fl");
        motorFR = hardwareMap.dcMotor.get("fr");
        motorBL = hardwareMap.dcMotor.get("bl");
        motorBR = hardwareMap.dcMotor.get("br");

        //Camera setup
        int cameraMonitorViewId = hardwareMap.appContext.getResources().getIdentifier("cameraMonitorViewId", "id", hardwareMap.appContext.getPackageName());
        VuforiaLocalizer.Parameters parameters = new VuforiaLocalizer.Parameters(cameraMonitorViewId);
        parameters.cameraDirection = VuforiaLocalizer.CameraDirection.FRONT;
        this.vuforia = ClassFactory.createVuforiaLocalizer(parameters);

        //Vuforia
        parameters.vuforiaLicenseKey = "AQXLr5v/////AAAAGXJCbi1ut0+SmeEkH1vkZG0NpkUylv0BWeZ4GaNc7LTEne9mZKpKMgWbCuVD61ge5I21IinkJ2L4JTKau7uw5jlPtkvH/PVDQ2EYv9UGJ6d0ml/iI2pWhuv4wDNbuOwWtB3/kuepar8zRVCOI0Ec05z766KFRLyb6ldCuMdQ04hQOn/02RYZRv43IBcMhOJiY3gs0oEiTwb+I4yVBa7qp8bQTGEjdysybOicD9JnswAzF4i0qSVh9WRGbkY8rFlkA+THheK72syEnD9iVCfXjgKaPuUZ95XpR4V7eUl/LnSm3uQ/FrXjRkyYU0TsaWROhRrdpMNSZSt17A/RDL7lE0K6iil9wMEo/7UT/jO5cqOn";
        VuforiaTrackables relicTrackables = this.vuforia.loadTrackablesFromAsset("RelicVuMark");
        VuforiaTrackable relicTemplate = relicTrackables.get(0);
        relicTemplate.setName("relicVuMarkTemplate"); // can help in debugging; otherwise not necessary

        //Mecanum
        bot = new Mecanum(motorFR,motorFL,motorBR,motorBL);

        //IMU
        imu = new AdafruitIMU(hardwareMap.get(BNO055IMU.class, "imu"));

        //Color Sensor
        sensorColor = hardwareMap.get(ColorSensor.class, "sensor_color_distance");

        //Servo
        jewelHitter = hardwareMap.servo.get("servo_hitter");
        jewelHitter.setPosition(0);

        driveDistance = 15.0;

        waitForStart();

//////////////////////////////////////////////////////////////////////////play!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!
        imu.start();
        //STATE ONE: MOVE FORWARD
        encoderDrive(2,"forward",.5);

        //STATE TWO: DETECT BALLS
        jewelHitter.setPosition(.75);
        if(sensorColor.blue()>sensorColor.red()){
            gyroTurnLeft(10,"oof",.3);
            gyroTurnRight(10,"oof",.3);
        }else{
            gyroTurnRight(10,"oof",.3);
            gyroTurnLeft(10,"oof",.3);
        }
        jewelHitter.setPosition(0.0);

        //STATE THREE: SCAN VUMARK
        encoderDrive(2.0,"left",.4);

        RelicRecoveryVuMark vuMark = RelicRecoveryVuMark.from(relicTemplate);
        telemetry.addData("VuMark", "%s visible", vuMark);

        telemetry.update();
        if (vuMark == RelicRecoveryVuMark.LEFT){
            driveDistance = 0.0;
        }else if (vuMark == RelicRecoveryVuMark.CENTER){
            driveDistance = 12.0;
        }else if (vuMark == RelicRecoveryVuMark.RIGHT){
            driveDistance = 24.0;
        }else{
            driveDistance = 12.0;
        }
        //STATE FOUR: MOVE RIGHT
        encoderDrive(36.0,"right",.4);

        //TURN 180 degrees
        gyroTurnLeft(180,"oof",0.3);

        //STATE FIVE: MOVE FORWARD
        encoderDrive(driveDistance,"forward",.4);

        //STATE SIX: STACK BLOCK


    }

    public void encoderDrive(double inches, String direction , double power ) {
        int encoderval;
        //
        // Sets the encoders
        //
        bot.reset_encoders();
        encoderval = ticks_per_inch.intValue() * (int) inches;
        bot.run_to_position();
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



    public void gyroTurnRight(double angle, String direction, double power){
        double aheading = imu.getHeading() + angle;
        if(aheading>360){
            aheading=aheading-360;
        }
        boolean gua = false;
        bot.run_without_encoders();
        bot.setPowerD(power);

        while(opModeIsActive() && gua==false) {
            //aheading = Math.abs(imu.getHeading()) + angle;
            bot.turn_right();

            telemetry.addData("Heading", imu.getHeading());
            telemetry.addData("Target Angle", aheading);
            telemetry.update();
            if (imu.getHeading() >= (aheading - THRESHOLD) && (imu.getHeading() <= (aheading + THRESHOLD))) {
                bot.brake();
                gua=true;
            }

        }

        bot.brake();

    }

    public void gyroTurnLeft(double angle, String direction, double power){
        double aheading = imu.getHeading() - angle;
        boolean gua = false;
        bot.run_without_encoders();
        bot.setPowerD(power);

        while(opModeIsActive() && gua==false) {
            //aheading = Math.abs(imu.getHeading()) + angle;
            bot.turn_left();

            telemetry.addData("Heading", imu.getHeading());
            telemetry.addData("Target Angle", aheading);
            telemetry.update();
            if (imu.getHeading() >= (aheading - THRESHOLD) && (imu.getHeading() <= (aheading + THRESHOLD))) {
                bot.brake();
                gua=true;
            }

        }

        bot.brake();

    }
}
