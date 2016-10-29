package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.hardware.HardwareMap;


/**
 * Created by Team 2891 on 10/28/2016.
 */
public class BangBangDriveForward extends Command {
    DriveTrain drive = new DriveTrain();
    public void initialize(HardwareMap hm){
        drive.init_Drive(hm, Robot.State.AUTONOMOUS);
        drive.resetEncoders();
    }
    public void run(){
        double distanceRemaining = Constants.DRIVE_DISTANCE - drive.ticksToInches(drive.getRightEncoderValue());
        if (Math.abs(distanceRemaining) <= 1){
            drive.runRight(0.0);
            drive.runLeft(0.0);
        }
        else if (distanceRemaining > 1) {
            drive.runRight(0.4);
            drive.runLeft(0.4);
        }
        else {
            drive.runRight(-0.4);
            drive.runLeft(-0.4);
        }
    }

    @Override
    public boolean isFinished() {
        return Math.abs(Constants.DRIVE_DISTANCE - drive.ticksToInches(drive.getRightEncoderValue())) <= 1;
    }
}
