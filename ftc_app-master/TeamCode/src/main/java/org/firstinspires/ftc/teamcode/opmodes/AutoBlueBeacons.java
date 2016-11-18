package org.firstinspires.ftc.teamcode.opmodes;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;

import org.firstinspires.ftc.robotcore.external.Telemetry;
import org.firstinspires.ftc.teamcode.base.Robot;

/**
 * Created by Team 6696 on 11/15/2016.
 */
@Autonomous(name = "AutoBlueBeacons")
public class AutoBlueBeacons extends LinearOpMode{

    private Robot robot = Robot.getInstance();
    public static Telemetry telemetryPass;
    
    @Override
    public void runOpMode() throws InterruptedException {
        telemetryPass = telemetry;
        robot.autonomousInit(hardwareMap);
        while(!robot.gyroIsReady()&&robot.getBlue()==255){
            telemetry.addData("gyro not ready", "");
            telemetry.addData("blueval ", robot.getBlue());
            telemetry.update();
        }
        telemetry.addData("gyro is ready", "");
        telemetry.addData("color sens is ready", robot.getBlue());
        telemetry.addData("ods", robot.getOds());
        telemetry.update();
        //wait for play button
        super.waitForStart();
        //one wheel turn towards beacon
        robot.driveTrain.oneWheelTurn(34, 0.275, true);
        //drive to white line
        robot.driveTrain.driveToLine(60, 0.9);
        //wait half a second
        //drive a tiny bit forward so we are centered when we turn
        robot.driveTrain.driveToDistance(2.75, 0.4, true);
        //wait half a second
        //turn to the beacon
        robot.driveTrain.turn(37,0.25,true);
        //drive forward to see color sensor
        //hit beacon
        robot.driveTrain.driveToDistance(8,0.35, true);
        //drive backward
        robot.driveTrain.driveToDistance(6,0.5, false);

        robot.beacon.initPos();
        robot.driveTrain.turn(77,0.4,false);
        robot.driveTrain.driveToDistance(40,0.8,true);
        robot.driveTrain.driveToLine(12,0.6);
        robot.driveTrain.driveToDistance(2.5,0.4,true);
        robot.driveTrain.turn(80,0.3,true);
        robot.driveTrain.driveToDistance(8,0.35,true);
        /*
        beac2blue = robot.isBlue();
        robot.driveTrain.driveToDistance(-6,0.2);
        robot.driveTrain.driveToDistance(6,0.2);*/
    }
}
