package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.hardware.HardwareMap;

import org.firstinspires.ftc.teamcode.base.Robot;
import org.firstinspires.ftc.teamcode.base.SensorBase;
import org.firstinspires.ftc.teamcode.subsystems.DriveTrain;

/**
 * Created by Team 2891 on 10/28/2016.
 */
public class PIDDriveForward extends Command {
    DriveTrain drive = DriveTrain.getInstance();
    SensorBase sensorBase = SensorBase.getInstance();

    private double power;

    public void setSetpoint(double ticks){
        drive.setTargetPos((int)ticks);
    }

    public void setPower(double power){
        this.power = power;
        drive.resetEncoders();
    }

    public void initialize(HardwareMap hm) {
        drive.initDrive(hm, Robot.State.AUTONOMOUS);
        drive.resetEncoders();
        sensorBase.initSensorBase(hm);
    }

    public void run() {
        drive.runLeft(power);
        drive.runRight(power);
    }

    public void end(){
        drive.runLeft(0);
        drive.runRight(0);
    }
    public boolean isFinished() {
        return Math.abs(drive.ticksToInches(drive.getTargetPos()) - Math.abs(drive.ticksToInches((drive.getRightEncoderValue()+drive.getLeftEncoderValue())/2))) <= 2;
    }
}
