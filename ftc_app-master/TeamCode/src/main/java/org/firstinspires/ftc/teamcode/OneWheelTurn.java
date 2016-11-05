package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.HardwareMap;

/**
 * Created by Team 2891 on 11/4/2016.
 */
public class OneWheelTurn extends  Command{
    double power, setpoint;
    boolean right;

    DriveTrain driveTrain = new DriveTrain();
    SensorBase sensorBase = new SensorBase();

    public OneWheelTurn(){

    }

    @Override
    public void initialize(HardwareMap hm) {
        driveTrain.init_Drive(hm,Robot.State.TELEOP);
        driveTrain.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        sensorBase.init_SensorBase(hm);
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
