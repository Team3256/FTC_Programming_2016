package org.firstinspires.ftc.teamcode;

import com.kauailabs.navx.ftc.navXPIDController;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.HardwareMap;


/**
 * Created by Team 2891 on 10/29/2016.
 */
public class Turn extends Command {
    DriveTrain drive = new DriveTrain();
    SensorBase sensorBase = new SensorBase();

    public void initialize(HardwareMap hm) {
        drive.init_Drive(hm, Robot.State.TELEOP);
        sensorBase.init_SensorBase(hm);
    }

    private double direction;
    double setpoint;
    double output;

    public void setParams(double setpoint, double output, boolean right){
        this.setpoint = setpoint;
        this.output = output;
        direction = (right?1:-1);
        drive.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        sensorBase.resetSensors();
    }
    public void run() {
        if (isFinished()) {
            end();
        } else {
            drive.runLeft(direction*-output);
            drive.runRight(direction*output);
        }
    }

    public boolean isFinished() {
        return (Math.abs(setpoint-Math.abs(sensorBase.getAngle()))<2);
    }
    public void end(){
        drive.runLeft(0);
        drive.runRight(0);
    }

}
