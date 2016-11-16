package org.firstinspires.ftc.teamcode.opmodes;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;

import org.firstinspires.ftc.robotcore.external.Telemetry;
import org.firstinspires.ftc.teamcode.base.Robot;

/**
 * Created by Team 6696 on 11/15/2016.
 */
@Autonomous(name = "auton")
public class AutonTest extends LinearOpMode{

    Robot robot = Robot.getInstance();
    boolean beac1blue = false, beac2blue = false;
    @Override
    public void runOpMode() throws InterruptedException {
        robot.autonomousInit(hardwareMap);

        while(!robot.gyroIsReady()){
            telemetry.addData("gyro not ready", 1);
        }
        telemetry.addData("gyro is ready", 0);
        super.waitForStart();

        robot.driveTrain.oneWheelTurn(32,0.15,true);
        robot.driveTrain.driveToLine(60,0.4);
        robot.driveTrain.turn(36,0.15,true);
        robot.driveTrain.driveToDistance(8,0.2);
        beac1blue = robot.isBlue()?true:false;
        robot.driveTrain.driveToDistance(-6,0.2);
        robot.beacon.setServoPosition(beac1blue);
        robot.driveTrain.driveToDistance(6,0.2);
        robot.beacon.initPos();
        robot.driveTrain.turn(80,0.15,false);
        robot.driveTrain.driveToDistance(40,0.5);
        robot.driveTrain.driveToLine(24,0.2);
        robot.driveTrain.turn(80,0.15,true);
        robot.driveTrain.driveToDistance(8,0.2);
        beac2blue = robot.isBlue()?true:false;
        robot.driveTrain.driveToDistance(-6,0.2);
        robot.driveTrain.driveToDistance(6,0.2);
    }
}
