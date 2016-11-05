package org.firstinspires.ftc.teamcode;

import android.graphics.Color;

import com.kauailabs.navx.ftc.AHRS;
import com.qualcomm.robotcore.hardware.ColorSensor;
import com.qualcomm.robotcore.hardware.DeviceInterfaceModule;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.I2cAddr;
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
    private ColorSensor bottomL;
    private ColorSensor bottomR;

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
        bottomL = hm.colorSensor.get("bottomL");
        bottomL.setI2cAddress(I2cAddr.create7bit(0x26));
        bottomL.enableLed(false);
        bottomL.enableLed(true);
        bottomR = hm.colorSensor.get("bottomR");
        bottomR.setI2cAddress(I2cAddr.create7bit(0x1E));
        bottomR.enableLed(false);
        bottomR.enableLed(true);
        hm.logDevices();
    }

    public void createGyro(){
        gyro = AHRS.getInstance(dim, Constants.GYRO_I2C_PORT, AHRS.DeviceDataType.kProcessedData, Constants.NAVX_GYRO_UPDATE_HZ);
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
    double offset = 0;
    public double getAngle(){
        return gyro.getYaw()-offset;
    }

    /**
     * resetGyro()
     * This method resets the Gyro angle back to zero
     */
    public void resetGyro(){
        gyro.zeroYaw();
        offset = gyro.getYaw();
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

    public int getLWhite(){
        return bottomL.alpha();
    }

    public boolean isLWhite(){
        return getLWhite()>10;
    }

    public int getRWhite() {return bottomR.alpha();}

    public boolean isRWhite() { return getRWhite()>10;}
}
