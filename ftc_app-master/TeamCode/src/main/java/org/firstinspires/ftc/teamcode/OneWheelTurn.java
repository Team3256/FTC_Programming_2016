package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.hardware.HardwareMap;

/**
 * Created by Team 2891 on 11/4/2016.
 */
public class OneWheelTurn extends  Command{
    double power, setpoint;
    boolean right;

    DriveTrain driveTrain;
    SensorBase sensorBase;

    public OneWheelTurn(){

    }

    @Override
    public void initialize(HardwareMap hm) {
        driveTrain.init_Drive(hm,Robot.State.TELEOP);
        sensorBase.init_SensorBase(hm);
        sensorBase.resetSensors();

    }

    public void setParams(double power, double setpoint, boolean right){
        this.power = power;
        this.setpoint = setpoint;
        this.right = right;
    }

    @Override
    public void run() {
        if (right) driveTrain.runLeft(power);
        else driveTrain.runRight(power);
    }

    @Override
    public boolean isFinished() {
        return Math.abs(sensorBase.getAngle()-setpoint)<2;
    }
}
