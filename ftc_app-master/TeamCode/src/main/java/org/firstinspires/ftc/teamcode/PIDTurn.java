package org.firstinspires.ftc.teamcode;

import com.kauailabs.navx.ftc.navXPIDController;
import com.qualcomm.robotcore.hardware.HardwareMap;


/**
 * Created by Team 2891 on 10/29/2016.
 */
public class PIDTurn extends Command {
    DriveTrain drive = new DriveTrain();
    SensorBase sensorBase = new SensorBase();
    navXPIDController turnController;
    navXPIDController.PIDResult pidResult = new navXPIDController.PIDResult();

    public void initialize(HardwareMap hm) {
        drive.init_Drive(hm, Robot.State.AUTONOMOUS);
        sensorBase.init_SensorBase(hm);
        turnController = new navXPIDController(sensorBase.getGryo(), navXPIDController.navXTimestampedDataSource.YAW);
        turnController.setSetpoint(Constants.AUTO_TURN_ANGLE);
        turnController.setContinuous(true);
        turnController.setOutputRange(Constants.MIN_TURN_OUTPUT, Constants.MAX_TURN_OUTPUT);
        turnController.setPID(Constants.kP_TURN, Constants.kI_TURN, Constants.kD_TURN);
        turnController.enable(true);
    }

    public void run() {
        double output = 0;
        try {
            if (turnController.waitForNewUpdate(pidResult, 5)) {
                if (turnController.isOnTarget()) {
                    drive.runRight(0);
                    drive.runLeft(0);
                } else {
                    output = turnController.get();
                    if (output < 0) {
                        drive.runLeft(output);
                        drive.runRight(-output);
                    }
                    else {
                        drive.runRight(output);
                        drive.runLeft(-output);
                    }
                }
            }
        } catch (InterruptedException e) {
            e.printStackTrace();
        }
    }

    public boolean isFinished() {
        return turnController.isOnTarget();
    }
}
