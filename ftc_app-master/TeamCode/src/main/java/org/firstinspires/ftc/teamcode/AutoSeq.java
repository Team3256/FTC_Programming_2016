package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.HardwareMap;

/**
 * Created by Team 2891 on 10/31/2016.
 */
public class AutoSeq extends Command{
    DriveTrain driveTrain = new DriveTrain();
    Intake intake = new Intake();
    SensorBase sensorBase = new SensorBase();
    Robot robot = new Robot();
    PIDTurn pidTurn;
    PIDTurn pidTurn2;
    PIDDriveForward moveForwardOne;
    PIDDriveForward moveForwardTwo;
    PIDDriveForward moveForwardThree;
    PIDDriveForward moveBackwardOne;
    WaitCommand waitOne;
    WaitCommand resetEncoders;

    int curr_step = 0;

    public void initialize(HardwareMap hm) {
        robot.robotInit(hm, driveTrain, intake, sensorBase, "autonomous");
        sensorBase.resetSensors();
        driveTrain.resetEncoders();
        moveForwardOne = new PIDDriveForward();
        moveForwardOne.initialize(hm);
        moveForwardOne.setPower(0.4);
        moveForwardOne.setSetpoint(driveTrain.inchesToTicks(18));
        moveForwardTwo = new PIDDriveForward();
        moveForwardThree = new PIDDriveForward();
        moveForwardThree.initialize(hm);
        moveBackwardOne = new PIDDriveForward();
        moveBackwardOne.initialize(hm);
        pidTurn = new PIDTurn();
        pidTurn.initialize(hm);
        pidTurn2 = new PIDTurn();
        pidTurn2.initialize(hm);
        moveForwardTwo.initialize(hm);
        waitOne = new WaitCommand(1000);
        resetEncoders = new WaitCommand(100);
    }

    public void run() {
        //drive forward
        if (curr_step == 0) {
            if (moveForwardOne.isFinished()) {
                ++curr_step;
                moveForwardOne.end();
                sensorBase.resetSensors();
                driveTrain.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
                pidTurn.setParams(Constants.AUTO_TURN_ANGLE, true);
            }
            else {
                driveTrain.setMode(DcMotor.RunMode.RUN_TO_POSITION);
                moveForwardOne.run();
            }
        }
        //wait
        else if (curr_step == 1){
            waitOne.run();
            curr_step++;
        }
        else if (curr_step == 2){
            if (pidTurn.isFinished()){
                pidTurn.end();
                curr_step++;
            }
            else{
                pidTurn.run();
            }
        }
        else if (curr_step == 3){
            //reset encoder mode
            driveTrain.resetEncoders();
            //wait for enc to reset
            resetEncoders.run();
            //set next mode for next state
            driveTrain.setMode(DcMotor.RunMode.RUN_TO_POSITION);
            curr_step++;
        }
        else if (curr_step == 4){
            //wait again
            waitOne.run();
            curr_step++;
            //Set next command params
            moveForwardTwo.setPower(0.5);
            moveForwardTwo.setSetpoint(driveTrain.inchesToTicks(60));
        }
        else if (curr_step == 5) {
            if (moveForwardTwo.isFinished()||sensorBase.isWhite()) {
                moveForwardTwo.end();
                curr_step++;
            } else moveForwardTwo.run();
        }
        else if (curr_step == 6) {
            //wait again
            waitOne.run();
            curr_step++;
            pidTurn2.setParams(Constants.AUTO_TURN_ANGLE_2, true);
            driveTrain.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
            sensorBase.resetSensors();
        }
        else if (curr_step == 7) {
            if (pidTurn2.isFinished()) {
                pidTurn2.end();
                curr_step++;
            } else pidTurn2.run();
        }
        else if (curr_step == 8){
            driveTrain.resetEncoders();
            driveTrain.setMode(DcMotor.RunMode.RUN_TO_POSITION);
            curr_step++;
            moveForwardThree.setPower(0.4);
            moveForwardThree.setSetpoint(driveTrain.inchesToTicks(8));
        }
        else if (curr_step == 9){
            if (moveForwardThree.isFinished()){
                moveForwardThree.end();
                curr_step++;
            }
            else moveForwardThree.run();
        }/**
        else if (curr_step == 10){

            driveTrain.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
            //wait for enc to reset
            resetEncoders.run();
            //set next mode for next state
            driveTrain.setMode(DcMotor.RunMode.RUN_TO_POSITION);
            curr_step++;
            moveBackwardOne.setPower(-0.4);
            moveBackwardOne.setSetpoint(14);
        }
        else if (curr_step == 11){
            if (moveBackwardOne.isFinished()){
                moveBackwardOne.end();
                curr_step++;
            }
            else moveBackwardOne.run();
        }*/
    }

    public int getCurr_step(){
        return curr_step;
    }

    public boolean isFinished() {
        return true;
    }
}
