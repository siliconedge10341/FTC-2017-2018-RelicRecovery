package org.firstinspires.ftc.teamcode.classes;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.hardware.DcMotor;
/**
 * Created by adars on 9/23/2017.
 */
@TeleOp(name = "MecanumDrive", group = "Drive")
public class nigboi2 extends OpMode{
    private DcMotor motorFR;
    private DcMotor motorFL;
    private DcMotor motorBR;
    private DcMotor motorBL;
    private boolean forwardtest;
    private boolean backwardtest;
    private boolean rightwardtest;
    private boolean leftwardtest;
    private double power;
    private double powerT1;
    private double powerT2;
    public void init(){
        motorFL = hardwareMap.dcMotor.get("fl");
        motorFR = hardwareMap.dcMotor.get("fr");
        motorBL = hardwareMap.dcMotor.get("bl");
        motorBR = hardwareMap.dcMotor.get("br");
    }
    public void loop(){
        forwardtest = gamepad1.y;
        backwardtest = gamepad1.a;
        rightwardtest = gamepad1.b;
        leftwardtest = gamepad1.x;
        if (forwardtest){
            power = 1;
            motorFL.setPower(power);
            motorFR.setPower(power);
            motorBL.setPower(power);
            motorBR.setPower(power);
        }
        else if (backwardtest){
            power = -1;
            motorBL.setPower(power);
            motorBR.setPower(power);
            motorFL.setPower(power);
            motorFR.setPower(power);
        }
        else if (rightwardtest){
            powerT1 = -1;
            powerT2 = 1;
            motorBL.setPower(powerT2);
            motorBR.setPower(powerT1);
            motorFL.setPower(powerT2);
            motorFR.setPower(powerT1);
        }
        else if (leftwardtest){
            powerT1 = 1;
            powerT2 = -1;
            motorBL.setPower(powerT2);
            motorBR.setPower(powerT1);
            motorFL.setPower(powerT2);
            motorFR.setPower(powerT1);
        }
        power = 0;
        powerT1 = 0;
        powerT2 = 0;
    }
    }
