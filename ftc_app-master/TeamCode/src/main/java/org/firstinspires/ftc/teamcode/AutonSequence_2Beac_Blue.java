package org.firstinspires.ftc.teamcode;

import android.hardware.Camera;
import android.media.CamcorderProfile;
import android.util.Log;

import com.qualcomm.robotcore.hardware.HardwareMap;

import org.firstinspires.ftc.teamcode.base.Robot;
import org.firstinspires.ftc.teamcode.base.SensorBase;
import org.firstinspires.ftc.teamcode.subsystems.Beacon;
import org.firstinspires.ftc.teamcode.subsystems.DriveTrain;
import org.firstinspires.ftc.teamcode.subsystems.Intake;

/**
 * Created by Team 2891 on 10/31/2016.
 */
public class AutonSequence_2Beac_Blue{
    DriveTrain driveTrain = DriveTrain.getInstance();
    Intake intake = Intake.getIntake();
    SensorBase sensorBase = SensorBase.getInstance();
    Beacon beacon = Beacon.getBeacon();
    Robot robot = Robot.getInstance();
    OneWheelTurn oneWheelTurnOne;
    PIDDriveForward moveForwardOne;
    PIDDriveForward bangWall;
    PIDDriveForward unBangWall;
    PIDDriveForward drive2SecondBeaconPart1;
    PIDDriveForward drive2SecondBeaconPart2;
    PIDDriveForward compensateOvershoot;
    Turn reAlign;
    Turn turn1;
    Turn turn2DriveSecondBeacon;
    Turn turn2SecondBeacon;
    ServoSetPosition beaconOne;
    WaitCommand waitOne;
    WaitCommand longWait;

    int curr_step = 0;

    public void initialize(HardwareMap hm) {
        robot.robotInit(hm, driveTrain, intake, beacon, sensorBase, "autonomous");
        sensorBase.resetSensors();
        driveTrain.resetEncoders();
        waitOne = new WaitCommand(1000);
        oneWheelTurnOne = new OneWheelTurn();
        oneWheelTurnOne.initialize(hm);
        oneWheelTurnOne.setParams(0.3, 32, true);
        moveForwardOne = new PIDDriveForward();
        moveForwardOne.initialize(hm);
        drive2SecondBeaconPart1 = new PIDDriveForward();
        drive2SecondBeaconPart1.initialize(hm);
        drive2SecondBeaconPart2 = new PIDDriveForward();
        drive2SecondBeaconPart2.initialize(hm);
        turn1 = new Turn();
        turn1.initialize(hm);
        bangWall = new PIDDriveForward();
        bangWall.initialize(hm);
        unBangWall = new PIDDriveForward();
        unBangWall.initialize(hm);
        turn2DriveSecondBeacon = new Turn();
        turn2DriveSecondBeacon.initialize(hm);
        reAlign = new Turn();
        reAlign.initialize(hm);
        turn2SecondBeacon = new Turn();
        turn2SecondBeacon.initialize(hm);
        compensateOvershoot = new PIDDriveForward();
        compensateOvershoot.initialize(hm);
        beaconOne = new ServoSetPosition();
        beaconOne.initialize(hm);
        longWait = new WaitCommand(5000);
    }

    double angle_offset;
    public void run(HardwareMap hm) {
        //drive forward
        if (curr_step == 0) {
            if (oneWheelTurnOne.isFinished()) {
                oneWheelTurnOne.end();
                moveForwardOne.setSetpoint(driveTrain.inchesToTicks(60));
                moveForwardOne.setPower(0.6);
                sensorBase.resetSensors();
                curr_step++;
            }
            else {
                oneWheelTurnOne.run();
            }
        }
        else if (curr_step == 1){
            if (moveForwardOne.isFinished()||sensorBase.getOds()>0.5){
                moveForwardOne.end();
                sensorBase.resetSensors();
                turn1.setParams(38, 0.18, true);
                curr_step++;
            }
            else moveForwardOne.run();
        }

        else if (curr_step == 2){
            if (turn1.isFinished()){
                turn1.end();
                boolean blue = (sensorBase.isBlue()?true:false);
                beaconOne.seeBlue(blue);
                curr_step++;
            }
            else turn1.run();
        }
        else if (curr_step == 3){
            if (beaconOne.isFinished()){
                bangWall.setSetpoint(driveTrain.inchesToTicks(15));
                bangWall.setPower(0.3);
                longWait.run();
                curr_step++;
            }
            else beaconOne.run();
        }
        else if (curr_step == 4){
            if (bangWall.isFinished()){
                bangWall.end();
                unBangWall.setPower(-0.3);
                unBangWall.setSetpoint(driveTrain.inchesToTicks(6));
                curr_step++;
            }
            else bangWall.run();
        }

        else if (curr_step == 5){
            if (unBangWall.isFinished()){
                unBangWall.end();
                turn2DriveSecondBeacon.setParams(81, 0.2, false);
                curr_step++;
            }
            else unBangWall.run();
        }
        else if (curr_step == 6){
            if (turn2DriveSecondBeacon.isFinished()){
                turn2DriveSecondBeacon.end();
                drive2SecondBeaconPart2.setPower(0.6);
                drive2SecondBeaconPart2.setSetpoint(driveTrain.inchesToTicks(60));
                sensorBase.resetSensors();
                curr_step++;
            }
            else turn2DriveSecondBeacon.run();
        }
        else if (curr_step == 7){
            if (drive2SecondBeaconPart2.isFinished()||(sensorBase.getOds()>0.5&&driveTrain.ticksToInches(driveTrain.getRightEncoderValue())>12)){
                drive2SecondBeaconPart2.end();
                compensateOvershoot.setPower(-0.15);
                compensateOvershoot.setSetpoint(driveTrain.inchesToTicks(3));
                curr_step++;
            }
            else drive2SecondBeaconPart2.run();
        }
        else if (curr_step == 8){
            if (compensateOvershoot.isFinished()||sensorBase.getOds()>0.5){
                compensateOvershoot.end();
                turn2SecondBeacon.setParams(80, 0.23, true);
                sensorBase.resetSensors();
                curr_step++;
            }
            else compensateOvershoot.run();
        }
        else if (curr_step == 9){
            if (turn2SecondBeacon.isFinished()){
                turn2SecondBeacon.end();
                bangWall.initialize(hm);
                bangWall.setPower(0.3);
                bangWall.setSetpoint(driveTrain.inchesToTicks(14));
                curr_step++;
            } else turn2SecondBeacon.run();
        }
        else if (curr_step == 10){
            if (bangWall.isFinished()){
                bangWall.end();
                curr_step++;
            }
            else bangWall.run();
        }
    }

    public boolean isFinished() {
        return true;
    }
}
