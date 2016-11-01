package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.hardware.HardwareMap;
/**
 * Created by Team 2891 on 10/28/2016.
 */
public class PIDDriveForward extends Command {
    DriveTrain drive = new DriveTrain();
    SensorBase sensorBase = new SensorBase();

    private double power;

    public void setSetpoint(double ticks){
        drive.setTargetPos((int)ticks);
    }

    public void setPower(double power){
        this.power = power;
    }

    public void initialize(HardwareMap hm) {
        drive.init_Drive(hm, Robot.State.AUTONOMOUS);
        drive.resetEncoders();
        sensorBase.init_SensorBase(hm);
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
