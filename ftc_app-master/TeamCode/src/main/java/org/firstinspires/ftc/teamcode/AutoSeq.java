package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.hardware.HardwareMap;

/**
 * Created by Team 2891 on 10/31/2016.
 */
public class AutoSeq extends Command{
    DriveTrain driveTrain = new DriveTrain();
    Intake intake = new Intake();
    SensorBase sensorBase = new SensorBase();
    Robot robot = new Robot();
    PIDDriveForward moveForwardOne;
    PIDDriveForward moveForwardTwo;
    WaitCommand waitOne;

    int curr_step = 0;

    public void initialize(HardwareMap hm) {
        robot.robotInit(hm, driveTrain, intake, sensorBase, "autonomous");
        sensorBase.resetSensors();
        driveTrain.resetEncoders();
        moveForwardOne = new PIDDriveForward();
        moveForwardOne.initialize(hm);
        moveForwardOne.setPower(0.5);
        moveForwardOne.setSetpoint(driveTrain.inchesToTicks(24));
        moveForwardTwo = new PIDDriveForward();
        moveForwardTwo.initialize(hm);
        waitOne = new WaitCommand(1000);
    }

    public void run() {
        if (curr_step == 0) {
            if (moveForwardOne.isFinished()) {
                ++curr_step;
                moveForwardOne.end();
                //Set next command params
                moveForwardTwo.setPower(-0.5);
                moveForwardTwo.setSetpoint(driveTrain.inchesToTicks(12));
            }
            else {
                moveForwardOne.run();
            }
        }
        //wait
        else if (curr_step == 1){
            waitOne.run();
            curr_step++;
        }
        else if (curr_step == 2){
            if (moveForwardTwo.isFinished()){
                moveForwardTwo.end();
                curr_step++;
            }
            else{
                moveForwardTwo.run();
            }
        }

    }

    public int getCurr_step(){
        return curr_step;
    }

    public boolean isFinished() {
        return true;
    }
}
