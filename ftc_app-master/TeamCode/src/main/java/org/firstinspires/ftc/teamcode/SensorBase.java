package org.firstinspires.ftc.teamcode;

import com.kauailabs.navx.ftc.AHRS;
import com.qualcomm.robotcore.hardware.DeviceInterfaceModule;
import com.qualcomm.robotcore.hardware.HardwareMap;

/**
 * Created by Eric on 9/16/2016.
 */
public class SensorBase {

    //Device Interface Board
    private DeviceInterfaceModule dim;

    //Navx micro gyro
    private AHRS gyro;

    //I2C port on Device Interface Module for the navx micro gyro
    private final int GYRO_I2C_Port = 0;

    /**
     * SensorBase()
     * @param hm Instance of the HardwareMap of the Robot
     */
    public SensorBase(HardwareMap hm){
        //initialize device interface module
        dim = hm.deviceInterfaceModule.get("Sensor Board");

        //initialize navx gyro
        gyro = AHRS.getInstance(dim, GYRO_I2C_Port, AHRS.DeviceDataType.kProcessedData);
    }

    /**
     * resetSensors()
     * resets all the sensors on the robot
     */
    public void resetSensors(){
        this.resetGyro();
    }

    /**
     * getAngle()
     * This method returns the current angle of the robot
     * @return angle
     */
    public double getAngle(){
        return gyro.getRawGyroX();
    }

    /**
     * resetGyro()
     * This method resets the Gyro angle back to zero
     */
    public void resetGyro(){
        gyro.zeroYaw();
    }
}
