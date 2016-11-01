package org.firstinspires.ftc.teamcode;

import com.kauailabs.navx.ftc.AHRS;
import com.qualcomm.robotcore.hardware.ColorSensor;
import com.qualcomm.robotcore.hardware.DeviceInterfaceModule;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.OpticalDistanceSensor;

import org.firstinspires.ftc.robotcontroller.internal.testcode.TestColorSensors;

/**
 * Created by Eric on 9/16/2016.
 */
public class SensorBase {

    //Device Interface Board
    private DeviceInterfaceModule dim;

    //Navx micro gyro
    private AHRS gyro;
    private ColorSensor colorSensorL;
    private ColorSensor colorSensorR;

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
        colorSensorL = hm.colorSensor.get("colorSensorL");
        colorSensorR = hm.colorSensor.get("colorSensorR");
        colorSensorL.enableLed(false);
        colorSensorR.enableLed(false);
    }

    /**
     * resetSensors()
     * resets all the sensors on the robot
     */
    public void resetSensors(){
        this.resetGyro();
    }

    public AHRS getGryo(){ return gyro;}
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
    private void resetGyro(){
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

    public int getLRed(){
        return colorSensorL.red();
    }

    public int getLBlue(){
        return colorSensorL.blue();
    }

    public int getRRed(){
        return colorSensorR.red();
    }

    public int getRBlue(){
        return colorSensorR.blue();
    }
    public boolean isLBlue() {
        return getLBlue() > getLRed();
    }

    public boolean isLRed() {
        return getLRed() > getLBlue();
    }

    public boolean isRBlue(){
        return getRBlue() > getRRed();
    }

    public boolean isRRed(){
        return getRRed() > getRBlue();
    }
}
