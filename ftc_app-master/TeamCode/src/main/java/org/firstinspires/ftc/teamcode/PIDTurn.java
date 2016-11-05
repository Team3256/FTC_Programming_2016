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
    }

    private double direction;
    double setpoint;

    public void setParams(double setpoint, boolean right){
        turnController = new navXPIDController(sensorBase.getGryo(), navXPIDController.navXTimestampedDataSource.YAW);
        turnController.setContinuous(true);
        turnController.setOutputRange(Constants.MIN_TURN_OUTPUT, Constants.MAX_TURN_OUTPUT);
        turnController.setTolerance(navXPIDController.ToleranceType.ABSOLUTE, 2);
        turnController.setPID(Constants.kP_TURN, Constants.kI_TURN, Constants.kD_TURN);
        pidResult = new navXPIDController.PIDResult();
        this.setpoint = setpoint;
        turnController.setSetpoint(setpoint);
        direction = (right?1:-1);
        turnController.enable(true);
        turnController.yawReset();
        sensorBase.resetGyro();
    }
    public void run() {
        double output = 0;
        try {
            if (turnController.waitForNewUpdate(pidResult, 20)) {
                if (turnController.isOnTarget()) {
                    drive.runRight(0);
                    drive.runLeft(0);
                } else {
                    output = turnController.get();
                    if (output < 0) {
                        drive.runLeft(direction*output);
                        drive.runRight(direction*-output);
                    }
                    else {
                        drive.runRight(direction*output);
                        drive.runLeft(direction*-output);
                    }
                }
            }
        } catch (InterruptedException e) {
            Thread.currentThread().interrupt();
        }
    }

    public boolean isFinished() {
        return (Math.abs(setpoint-sensorBase.getAngle())<2);
    }
    public void end(){
        drive.runLeft(0);
        drive.runRight(0);
        turnController.reset();
        turnController.enable(false);
        turnController.close();
    }

}
