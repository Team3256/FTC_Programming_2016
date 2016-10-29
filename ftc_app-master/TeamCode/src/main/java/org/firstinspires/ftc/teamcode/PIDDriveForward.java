package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.hardware.HardwareMap;
/**
 * Created by Team 2891 on 10/28/2016.
 */
public class PIDDriveForward extends Command {
    DriveTrain drive = new DriveTrain();
    SensorBase sensorBase = new SensorBase();
    PIDController controller = new PIDController(Constants.kP_STRAIGHT, Constants.kI_STRAIGHT, Constants.kD_STRAIGHT);

    public void initialize(HardwareMap hm) {
        drive.init_Drive(hm, Robot.State.AUTONOMOUS);
        drive.resetEncoders();
        controller.reset();
        controller.setMinOutput(Constants.MIN_STRAIGHT_POWER);
        controller.setMaxOutput(Constants.MAX_STRAIGHT_POWER);
        controller.calculatePID(0, Constants.DRIVE_DISTANCE);
        sensorBase.init_SensorBase(hm);
    }

    public void run() {
        if (!isFinished()) {
            double output = controller.calculatePID(drive.ticksToInches(drive.getRightEncoderValue()), Constants.DRIVE_DISTANCE);
           /* if (Math.abs(Constants.DRIVE_DISTANCE - drive.ticksToInches(drive.getRightEncoderValue())) <= 1){
                if (sensorBase.getAngle()>3){
                    drive.runLeft(-0.3);
                    drive.runRight(0.3);
                }
                else if (sensorBase.getAngle()<-3){
                    drive.runLeft(0.3);
                    drive.runRight(-0.3);
                }
            }else {*/
            if (sensorBase.getAngle() > 2) {
                    drive.runLeft(0.5*output);
                    drive.runRight(output);
                } else if (sensorBase.getAngle() < -2) {
                    drive.runLeft(output);
                    drive.runRight(0.5*output);
                } else {
                    drive.runLeft(output);
                    drive.runRight(output);
                }
           //  }

        } else {
            drive.runLeft(0);
            drive.runRight(0);
        }
    }

    public boolean isFinished() {
        return Math.abs(Constants.DRIVE_DISTANCE - drive.ticksToInches(drive.getRightEncoderValue())) <= 2;
    }
}
