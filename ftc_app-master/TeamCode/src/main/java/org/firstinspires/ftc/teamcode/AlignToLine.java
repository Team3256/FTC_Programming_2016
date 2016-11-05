package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.hardware.HardwareMap;

/**
 * Created by Team 2891 on 11/4/2016.
 */
public class AlignToLine extends Command {
    DriveTrain driveTrain = new DriveTrain();
    SensorBase sensorBase = new SensorBase();
    @Override
    public void initialize(HardwareMap hm) {
        driveTrain.init_Drive(hm, Robot.State.TELEOP);
        sensorBase.init_SensorBase(hm);
    }

    boolean finished = false, isLeft = false, isRight = false;

    @Override
    public void run() {
        isLeft = true; //sensorBase.isLWhite();
        isRight = sensorBase.getOds()>0.5;
        if (isLeft && isRight){
            finished = true;
        }
        else if (isLeft){
            driveTrain.runLeft(0.1);
        }
        else if (isRight){
            driveTrain.runRight(0.1);
        }
        else {
            driveTrain.runLeft(0.1);
            driveTrain.runRight(0.1);
        }
    }

    @Override
    public boolean isFinished() {
        return finished;
    }

    public void end(){
        driveTrain.runLeft(0);
        driveTrain.runRight(0);
    }
}
