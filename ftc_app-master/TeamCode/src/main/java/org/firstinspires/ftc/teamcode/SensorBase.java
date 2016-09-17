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
    protected AHRS gyro;

    /**
     * SensorBase()
     * Empty Constructor
     */
    public SensorBase(){

    }

    /**
     * init_SensorBase()
     * Initializes the whole SensorBase, including all the sensors
     * @param hm Instance of the HardwareMap of the Robot
     */
    public void init_SensorBase(HardwareMap hm){
        //initialize device interface module
        dim = hm.deviceInterfaceModule.get("Sensor Board");

        //initialize navx gyro
        gyro = AHRS.getInstance(dim, Constants.GYRO_I2C_PORT, AHRS.DeviceDataType.kProcessedData, Constants.NAVX_GYRO_UPDATE_HZ);

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
     * @return angle current angle from the gyro
     */
    public double getAngle(){
        return gyro.getYaw();
    }

    /**
     * resetGyro()
     * This method resets the Gyro angle back to zero
     */
    public void resetGyro(){
        gyro.zeroYaw();
    }

    /**
     * gyroIsConnected()
     * checks if the gyro is connected to the device interface module
     * @return gyro isConnected
     */
    public boolean gyroIsConnected(){
        return gyro.isConnected();
    }
}
