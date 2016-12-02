package org.firstinspires.ftc.teamcode.trajectory;

import android.os.Handler;
import android.os.Looper;

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

        leftGenerator.setConfig(20, 30, Constants.dt);
        rightGenerator.setConfig(20, 30, Constants.dt);
        leftTraj = leftGenerator.generateTraj(0,0,distance);
        rightTraj = rightGenerator.generateTraj(0,0,distance);
        leftFollower.setTrajectory(leftTraj);
        rightFollower.setTrajectory(rightTraj);
        leftFollower.setLoopTime(Constants.dt);
        rightFollower.setLoopTime(Constants.dt);
        leftFollower.setGains(Constants.kV, Constants.kA, Constants.kP, Constants.kI, Constants.kD);
        rightFollower.setGains(Constants.kV, Constants.kA, Constants.kP, Constants.kI, Constants.kD);
    }

    public void reset(){
        leftFollower.resetController();
        rightFollower.resetController();
    }

    Handler t = new Handler(Looper.getMainLooper());

    public void start(){
        reset();
        t.post(r);
    }

    public void stop(){
        t.removeCallbacks(r);
    }

    public boolean isFinished(){
        return leftFollower.isFinished() && rightFollower.isFinished();
    }
    double left,right;
    Runnable r = new Runnable() {
        public void run() {
            if (!isFinished()){
                left = leftFollower.calcMotorOutput(driveTrain.ticksToInches(driveTrain.getLeftEncoderValue()));
                right = rightFollower.calcMotorOutput(driveTrain.ticksToInches(driveTrain.getRightEncoderValue()));
                driveTrain.tankDrive(left, right);
                t.postDelayed(r,(int)(1000*Constants.dt));

            }
            else {
                driveTrain.setPower(0);
                stop();
            }
        }
    };
}
