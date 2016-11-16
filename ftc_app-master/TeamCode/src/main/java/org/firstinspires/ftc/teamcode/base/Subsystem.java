package org.firstinspires.ftc.teamcode.base;

import com.qualcomm.robotcore.hardware.HardwareMap;

/**
 * Created by Team 6696 on 11/15/2016.
 */
public abstract class Subsystem {

    public SensorBase sensorBase = SensorBase.getInstance();
    public abstract void init(HardwareMap hardwareMap);
    protected double getAngle(){
        return sensorBase.getAngle();
    }
    protected void resetGyro(){
        sensorBase.resetGyro();
    }
    protected  int getBlue(){
        return sensorBase.getBlue();
    }

    protected  boolean isBlue(){
        return sensorBase.isBlue();
    }

    protected double getOds(){
        return sensorBase.getOds();
    }

    protected boolean gyroIsReady(){ return sensorBase.gyroIsReady();}
}
