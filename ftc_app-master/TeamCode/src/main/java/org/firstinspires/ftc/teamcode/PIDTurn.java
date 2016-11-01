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
    navXPIDController.PIDResult pidResult;

    public void initialize(HardwareMap hm) {
        drive.init_Drive(hm, Robot.State.TELEOP);
        sensorBase.init_SensorBase(hm);
        sensorBase.resetSensors();
        pidResult = new navXPIDController.PIDResult();
        turnController = new navXPIDController(sensorBase.getGryo(), navXPIDController.navXTimestampedDataSource.YAW);
        turnController.setSetpoint(Constants.AUTO_TURN_ANGLE);
        turnController.setContinuous(true);
        turnController.setOutputRange(Constants.MIN_TURN_OUTPUT, Constants.MAX_TURN_OUTPUT);
        turnController.setTolerance(navXPIDController.ToleranceType.ABSOLUTE,2);
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
                    turnController.enable(false);
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
            Thread.currentThread().interrupt();
        }
    }

    public boolean isFinished() {
        return turnController.isOnTarget();
    }
}
