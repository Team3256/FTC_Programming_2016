package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.HardwareMap;

/**
 * Created by Team 2891 on 10/31/2016.
 */
public class AutonSequence{
    DriveTrain driveTrain = new DriveTrain();
    Intake intake = new Intake();
    SensorBase sensorBase = new SensorBase();
    Robot robot = new Robot();
    PIDTurn pidTurn;
    OneWheelTurn oneWheelTurnOne;
    OneWheelTurn oneWheelTurnTwo;
    PIDDriveForward moveForwardOne;
    WaitCommand waitOne;
    AlignToLine alignToLineOne;

    int curr_step = 0;

    public void initialize(HardwareMap hm) {
        robot.robotInit(hm, driveTrain, intake, sensorBase, "autonomous");
        sensorBase.resetSensors();
        driveTrain.resetEncoders();
        waitOne = new WaitCommand(1000);
        oneWheelTurnOne = new OneWheelTurn();
        oneWheelTurnOne.initialize(hm);
        oneWheelTurnOne.setParams(0.3, 40, true);
        moveForwardOne = new PIDDriveForward();
        moveForwardOne.initialize(hm);
        oneWheelTurnTwo = new OneWheelTurn();
        oneWheelTurnTwo.initialize(hm);
        alignToLineOne = new AlignToLine();
        alignToLineOne.initialize(hm);
    }

    public void run(HardwareMap hm) {
        //drive forward
        if (curr_step == 0) {
            if (oneWheelTurnOne.isFinished()) {
                oneWheelTurnOne.end();
                moveForwardOne.setSetpoint(driveTrain.inchesToTicks(50));
                moveForwardOne.setPower(0.5);
                curr_step++;
            }
            else {
                oneWheelTurnOne.run();
            }
        }
        else if (curr_step == 1){
            if (moveForwardOne.isFinished()){
                moveForwardOne.end();
                oneWheelTurnTwo.setParams(0.3,45,false);
                curr_step++;
            }
            else moveForwardOne.run();
        }
        else if (curr_step == 2){
            if (oneWheelTurnTwo.isFinished()){
                oneWheelTurnTwo.end();
                curr_step++;
            }
            else oneWheelTurnTwo.run();
        }
        else if (curr_step == 3) {
            if (alignToLineOne.isFinished()) {
                alignToLineOne.end();
                curr_step++;
            } else alignToLineOne.run();
        }
    }

    public boolean isFinished() {
        return true;
    }
}
