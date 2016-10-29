package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.hardware.HardwareMap;

/**
 * Created by Team 2891 on 10/28/2016.
 */
public class PIDDriveForward extends Command {
    DriveTrain drive = new DriveTrain();
    PIDController controller = new PIDController(Constants.kP_STRAIGHT, Constants.kI_STRAIGHT, Constants.kD_STRAIGHT);

    public void initialize(HardwareMap hm) {
        drive.init_Drive(hm, Robot.State.AUTONOMOUS);
        drive.resetEncoders();
        controller.reset();
        controller.setMinOutput(Constants.MIN_STRAIGHT_POWER);
        controller.setMaxOutput(Constants.MAX_STRAIGHT_POWER);
        controller.calculatePID(0, Constants.DRIVE_DISTANCE);
    }

    public void run() {
        double output = controller.calculatePID(drive.ticksToInches(drive.getEncoderValue()), Constants.DRIVE_DISTANCE);
        drive.runLeft(output);
        drive.runRight(output);
    }

    public boolean isFinished() {
        return controller.getError() <= 0.5;
    }
}
