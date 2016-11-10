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
    private OpticalDistanceSensor ods;
    private ColorSensor beacon_sensor;

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
        ods = hm.opticalDistanceSensor.get("ods");
        beacon_sensor = hm.colorSensor.get("beacon");
        beacon_sensor.enableLed(false);
        beacon_sensor.enableLed(false);
        hm.logDevices();
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
    public void resetGyro(){
        gyro.zeroYaw();
    }

    public double getOds(){
        return ods.getLightDetected();
    }

    public boolean isBlue(){
        return beacon_sensor.blue()>=2;
    }

    public void disableBeaconLED(){
        beacon_sensor.enableLed(false);
    }

    public int getBlue(){
        return beacon_sensor.blue();
    }

    public int getRed(){
        return beacon_sensor.red();
    }
}
