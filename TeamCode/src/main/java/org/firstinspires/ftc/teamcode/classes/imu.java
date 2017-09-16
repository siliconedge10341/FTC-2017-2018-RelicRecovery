package org.firstinspires.ftc.teamcode.classes;


/**
 * Created by vatty on 9/15/2017.
 */
import com.qualcomm.robotcore.hardware.I2cDeviceSynch;
import com.qualcomm.robotcore.hardware.I2cDeviceSynchDevice;
import com.qualcomm.robotcore.hardware.configuration.I2cSensor;

@I2cSensor(name = "IMU BNO055" , description = "Gyro from adafruit" , xmlTag = "BNO055")
public class imu extends I2cDeviceSynchDevice<I2cDeviceSynch> {
    @Override
    public Manufacturer getManufacturer(){

        return Manufacturer.Adafruit;
    }
    @Override
    protected synchronized boolean doInitialize()
    {
        return true;
    }

    @Override
    public String getDeviceName()
    {

        return "Adafruit IMU sensor";
    }

    public imu(I2cDeviceSynch deviceClient){
        super(deviceClient,true);

        super.registerArmingStateCallback(false);
        this.deviceClient.engage();
    }




}
