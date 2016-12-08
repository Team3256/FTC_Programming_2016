package org.firstinspires.ftc.teamcode.base;

import com.kauailabs.navx.ftc.AHRS;
import com.qualcomm.robotcore.hardware.ColorSensor;
import com.qualcomm.robotcore.hardware.DeviceInterfaceModule;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.OpticalDistanceSensor;

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
        beaconColorSensor = hardwareMap.colorSensor.get("colorSensor");
        beaconColorSensor.enableLed(true);
        beaconColorSensor.enableLed(false);
        hardwareMap.logDevices();
    }

    public void resetSensors() {
        this.resetGyro();
    }

    public AHRS getGyro() {
        return gyro;
    }
    
    public double getAngle() {
        return gyro.getYaw();
    }

    public double getPitch() {
        return gyro.getPitch();
    }

    public void resetGyro() {
        gyro.zeroYaw();
    }

    public double getOds() {
        return opticalDistanceSensor.getLightDetected();
    }

    public boolean isBlue() {
        beaconColorSensor.enableLed(false);
        return beaconColorSensor.blue() >= 2 && beaconColorSensor.blue()!=255;
    }

    public boolean isRed() {
        beaconColorSensor.enableLed(false);
        return beaconColorSensor.red() >= 2 && beaconColorSensor.red() != 255;
    }

    public void disableBeaconLED() {
        beaconColorSensor.enableLed(false);
    }

    public int getBlue() {
        beaconColorSensor.enableLed(false);
        return beaconColorSensor.blue();
    }

    public int getRed() {
        beaconColorSensor.enableLed(false);
        return beaconColorSensor.red();
    }

    public boolean gyroIsReady(){
        return true;//return !gyro.isCalibrating() && gyro.isConnected();
    }
}
