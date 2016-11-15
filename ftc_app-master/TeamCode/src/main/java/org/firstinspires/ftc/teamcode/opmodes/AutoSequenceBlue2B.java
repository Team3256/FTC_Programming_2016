package org.firstinspires.ftc.teamcode.opmodes;

import com.qualcomm.robotcore.hardware.HardwareMap;

import org.firstinspires.ftc.teamcode.base.Robot;
import org.firstinspires.ftc.teamcode.base.SensorBase;
import org.firstinspires.ftc.teamcode.subsystems.Beacon;
import org.firstinspires.ftc.teamcode.subsystems.DriveTrain;
import org.firstinspires.ftc.teamcode.subsystems.Intake;

/**
 * Created by Team 2891 on 10/31/2016.
 */
public class AutoSequenceBlue2B {
    DriveTrain driveTrain = DriveTrain.getInstance();
    Intake intake = Intake.getIntake();
    SensorBase sensorBase = SensorBase.getInstance();
    Beacon beacon = Beacon.getBeacon();
    Robot robot = Robot.getInstance();
    OneWheelTurn oneWheelTurnOne;
    DriveForward moveForwardOne;
    DriveForward bangWall;
    DriveForward unBangWall;
    DriveForward drive2SecondBeaconPart2;
    DriveForward compensateOvershoot;
    Turn turn1;
    Turn turn2DriveSecondBeacon;
    Turn turn2SecondBeacon;
    ServoSetPosition beaconOne;
    WaitCommand waitOne;
    WaitCommand longWait;

    HardwareMap hardwareMap;
    int currentStep = 0;

    public void initialize(HardwareMap hardwareMap) {
        this.hardwareMap = hardwareMap;
        robot.robotInit(hardwareMap, driveTrain, intake, beacon, sensorBase, "autonomous");
        sensorBase.resetSensors();
        driveTrain.resetEncoders();
        waitOne = new WaitCommand(1000);
        oneWheelTurnOne = new OneWheelTurn(hardwareMap, 0.3, 32, true);
        moveForwardOne = new DriveForward(hardwareMap, 0.6, 60);
        drive2SecondBeaconPart2 = new DriveForward(hardwareMap, 0.6, 60);
        turn1 = new Turn(hardwareMap, 0.38, 18, true);
        bangWall = new DriveForward(hardwareMap, 0.3, 15);
        unBangWall = new DriveForward(hardwareMap, -0.3, 6);
        turn2DriveSecondBeacon = new Turn(hardwareMap, 0.2, 81, false);
        turn2SecondBeacon = new Turn(hardwareMap, 0.23, 80, true);
        compensateOvershoot = new DriveForward(hardwareMap, -0.15, 3);
        beaconOne = new ServoSetPosition();
        beaconOne.initialize(hardwareMap);
        longWait = new WaitCommand(5000);
    }

    public void run() {
        //drive forward
        if (currentStep == 0) {
            if (oneWheelTurnOne.isFinished()) {
                oneWheelTurnOne.end();
                sensorBase.resetSensors();
                currentStep++;
            }
            else {
                oneWheelTurnOne.run();
            }
        }
        else if (currentStep == 1){
            if (moveForwardOne.isFinished()||sensorBase.getOds()>0.5){
                moveForwardOne.end();
                sensorBase.resetSensors();
                currentStep++;
            }
            else moveForwardOne.run();
        }

        else if (currentStep == 2){
            if (turn1.isFinished()){
                turn1.end();
                boolean blue = sensorBase.isBlue();
                beaconOne.setBlueSeen(blue);
                currentStep++;
            }
            else turn1.run();
        }
        else if (currentStep == 3){
            if (beaconOne.isFinished()){
                longWait.run();
                currentStep++;
            }
            else beaconOne.run();
        }
        else if (currentStep == 4){
            if (bangWall.isFinished()){
                bangWall.end();
                currentStep++;
            }
            else bangWall.run();
        }

        else if (currentStep == 5){
            if (unBangWall.isFinished()){
                unBangWall.end();
                currentStep++;
            }
            else unBangWall.run();
        }
        else if (currentStep == 6){
            if (turn2DriveSecondBeacon.isFinished()){
                turn2DriveSecondBeacon.end();
                sensorBase.resetSensors();
                currentStep++;
            }
            else turn2DriveSecondBeacon.run();
        }
        else if (currentStep == 7){
            if (drive2SecondBeaconPart2.isFinished()||(sensorBase.getOds()>0.5&&driveTrain.ticksToInches(driveTrain.getRightEncoderValue())>12)){
                drive2SecondBeaconPart2.end();
                currentStep++;
            }
            else drive2SecondBeaconPart2.run();
        }
        else if (currentStep == 8){
            if (compensateOvershoot.isFinished()||sensorBase.getOds()>0.5){
                compensateOvershoot.end();
                sensorBase.resetSensors();
                currentStep++;
            }
            else compensateOvershoot.run();
        }
        else if (currentStep == 9){
            if (turn2SecondBeacon.isFinished()){
                turn2SecondBeacon.end();
                bangWall = new DriveForward(hardwareMap, 0.3, 14);
                currentStep++;
            } else turn2SecondBeacon.run();
        }
        else if (currentStep == 10){
            if (bangWall.isFinished()){
                bangWall.end();
                currentStep++;
            }
            else bangWall.run();
        }
    }

    public boolean isFinished() {
        return true;
    }
}
