package org.firstinspires.ftc.teamcode.base;

import com.kauailabs.navx.ftc.AHRS;
import com.qualcomm.robotcore.hardware.ColorSensor;
import com.qualcomm.robotcore.hardware.DeviceInterfaceModule;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.OpticalDistanceSensor;

/**
 * Created by Team 6696 on 11/11/2016.
 */
public class SensorBase{
    private DeviceInterfaceModule deviceInterfaceModule;
    //sensors
    private AHRS gyro;
    private OpticalDistanceSensor opticalDistanceSensor;
    private ColorSensor beaconColorSensor;
    //singleton
    private static SensorBase sensorBase = new SensorBase();
    
    private SensorBase() {
        
    }
    
    public static SensorBase getInstance() {
        return sensorBase;
    }
    
    public void initSensorBase(HardwareMap hardwareMap) {
        deviceInterfaceModule = hardwareMap.deviceInterfaceModule.get("Sensor Board");
        gyro = AHRS.getInstance(deviceInterfaceModule, Constants.GYRO_I2C_PORT, AHRS.DeviceDataType.kProcessedData, Constants.NAVX_GYRO_UPDATE_HZ);
        opticalDistanceSensor = hardwareMap.opticalDistanceSensor.get("ods");
        beaconColorSensor = hardwareMap.colorSensor.get("beacon");
        beaconColorSensor.enableLed(false);
        beaconColorSensor.enableLed(false);
        hardwareMap.logDevices();
    }

    public void resetSensors() {
        this.resetGyro();
    }

    public AHRS getGryo() {
        return gyro;
    }
    
    public double getAngle() {
        return gyro.getYaw();
    }

    public double getPitch() {
        return gyro.getPitch();
    }

    public double getRoll() {
        return gyro.getRoll();
    }

    public void resetGyro() {
        gyro.zeroYaw();
    }

    public double getOds() {
        return opticalDistanceSensor.getLightDetected();
    }

    public boolean isBlue() {
        return beaconColorSensor.blue() >= 2 && beaconColorSensor.blue()!=255;
    }

    public void disableBeaconLED() {
        beaconColorSensor.enableLed(false);
    }

    public int getBlue() {
        return beaconColorSensor.blue();
    }

    public int getRed() {
        return beaconColorSensor.red();
    }

    public boolean gyroIsReady(){
        return !gyro.isCalibrating() && gyro.isConnected();
    }

}
