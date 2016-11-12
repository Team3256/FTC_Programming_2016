package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.HardwareMap;

import org.firstinspires.ftc.teamcode.base.Robot;
import org.firstinspires.ftc.teamcode.base.SensorBase;
import org.firstinspires.ftc.teamcode.subsystems.DriveTrain;

/**
 * Created by Team 2891 on 11/4/2016.
 */
public class OneWheelTurn extends  Command{
    double power, setpoint;
    boolean right;

    DriveTrain driveTrain = DriveTrain.getInstance();
    SensorBase sensorBase = SensorBase.getInstance();

    public OneWheelTurn(){

    }

    @Override
    public void initialize(HardwareMap hm) {
        driveTrain.initDrive(hm, Robot.State.TELEOP);
        driveTrain.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        sensorBase.initSensorBase(hm);
        sensorBase.resetSensors();
    }

    public void setParams(double power, double setpoint, boolean right){
        this.power = power;
        this.setpoint = setpoint;
        this.right = right;
        sensorBase.resetGyro();
    }

    public void run() {
        if (right) driveTrain.runRight(power);
        else if (!right) driveTrain.runLeft(power);
        else {
            driveTrain.runLeft(0);
            driveTrain.runRight(0);
        }
    }

    public boolean isFinished() {
        return (setpoint-Math.abs(sensorBase.getAngle()))<2;
    }

    public void end(){
        driveTrain.runLeft(0);
        driveTrain.runRight(0);
    }
}
