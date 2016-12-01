package org.firstinspires.ftc.teamcode.trajectory;

import android.util.Log;

import com.qualcomm.robotcore.hardware.DcMotor;

import org.firstinspires.ftc.teamcode.base.Constants;
import org.firstinspires.ftc.teamcode.opmodes.TelemetryHolder;
import org.firstinspires.ftc.teamcode.subsystems.DriveTrain;

import java.util.Timer;
import java.util.TimerTask;

/**
 * Created by Team 2891 on 11/30/2016.
 */

public class DriveController {

    DriveTrain driveTrain = DriveTrain.getInstance();

    public DriveController(){

    }

    Segment[] leftTraj;
    Segment[] rightTraj;

    TrajectoryGenerator leftGenerator = new TrajectoryGenerator();
    TrajectoryGenerator rightGenerator = new TrajectoryGenerator();

    TrajectoryFollower leftFollower = new TrajectoryFollower();
    TrajectoryFollower rightFollower = new TrajectoryFollower();

    double distance;

    public void setSetpoint(double setpoint){
        distance = setpoint;
    }

    public void init(){
        driveTrain.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);

        leftGenerator.setConfig(28, 50, Constants.dt);
        rightGenerator.setConfig(28, 50, Constants.dt);
        leftTraj = leftGenerator.generateTraj(0,0,distance);
        rightTraj = rightGenerator.generateTraj(0,0,distance);
        leftFollower.setTrajectory(leftTraj);
        rightFollower.setTrajectory(rightTraj);
        leftFollower.setLoopTime(Constants.dt);
        rightFollower.setLoopTime(Constants.dt);
        leftFollower.setGains(Constants.kV, Constants.kA, Constants.kP, Constants.kI, Constants.kD);
        rightFollower.setGains(Constants.kV,Constants.kA,Constants.kP,Constants.kI, Constants.kD);
    }

    public void reset(){
        leftFollower.resetController();
        rightFollower.resetController();
    }

    Timer t = new Timer();

    public void start(){
        reset();
        Updater loop = new Updater();
        long periodms = (long)(1000*Constants.dt);
        t.scheduleAtFixedRate(loop,0,periodms);
    }

    public void stop(){
        t.cancel();
        t.purge();
    }

    public boolean isFinished(){
        return leftFollower.isFinished() && rightFollower.isFinished();
    }

    public class Updater extends TimerTask{
        @Override
        public void run() {
            if (!isFinished()){
                double left = leftFollower.calcMotorOutput(driveTrain.ticksToInches(driveTrain.getLeftEncoderValue()));
                double right = rightFollower.calcMotorOutput(driveTrain.ticksToInches(driveTrain.getRightEncoderValue()));
                driveTrain.tankDrive(left, right);
            }
            else {
                driveTrain.setPower(0);
                stop();
            }
        }
    }
}
