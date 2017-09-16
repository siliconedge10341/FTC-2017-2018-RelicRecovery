package org.firstinspires.ftc.teamcode.classes;


/**
 * Created by vatty on 9/15/2017.
 */
import com.qualcomm.robotcore.hardware.I2cDeviceSynch;
import com.qualcomm.robotcore.hardware.I2cDeviceSynchDevice;

public class imu extends I2cDeviceSynchDevice<I2cDeviceSynch> {

    public Manufacturer getManufacturer(){

        return Manufacturer.Adafruit;
    }
    protected synchronized boolean doInitialize()
    {
        return true;
    }

    @Override
    public String getDeviceName()
    {

        return "Adafruit IMU sensor";
    }

    public imu(I2cDeviceSynch){
        super(deviceClient,true);

        super.registerArmingStateCallback(false);
        this.deviceClient.engage();
    }

}
