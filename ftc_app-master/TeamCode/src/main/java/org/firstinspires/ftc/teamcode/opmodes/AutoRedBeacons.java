package org.firstinspires.ftc.teamcode.opmodes;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;

import org.firstinspires.ftc.robotcore.external.Telemetry;
import org.firstinspires.ftc.teamcode.base.Robot;

@Autonomous(name = "AutoRedBeacons")

public class AutoRedBeacons extends LinearOpMode{
    private Robot robot = Robot.getInstance();
    public static Telemetry telemetryPass;

    @Override
    public void runOpMode() throws InterruptedException {
        TelemetryHolder.telemetry = telemetry;
        robot.autonomousInit(hardwareMap);
        while(!robot.gyroIsReady()){
            telemetry.addData("gyro ready", "no");
            telemetry.addData("redval", robot.getRed());
            telemetry.update();
        }
        telemetry.addData("gyro ready", robot.getAngle());
        telemetry.addData("color sens", robot.getRed());
        telemetry.addData("ods", robot.getOds());
        telemetry.update();
        //wait for play button
        super.waitForStart();
        //one wheel turn towards beacon
        robot.shooter.autoShootSequence();
        /*
        robot.driveTrain.oneWheelTurn(38, 0.275, false);
        //drive to white line
        robot.driveTrain.driveToLine(60, 0.9);
        //drive a tiny bit forward so we are centered when we turn
        robot.driveTrain.driveToDistance(2.75, 0.4, true);
        //turn to the beacon
        robot.driveTrain.turn(42, 0.25, false);
        //set servo position depending on what color we see
        robot.beacon.updateServoPositionRed();
        //hit beacon
        robot.driveTrain.driveToDistance(8, 0.35, true);
        //drive backward so we can turn towards second beacon
        robot.driveTrain.driveToDistance(6, 0.5, false);
        //reset servo positions to neutral for both
        robot.beacon.initPos();
        //turn towards the second beacon
        robot.driveTrain.turn(69, 0.4, true);
        //drive forward to prepare to see to the line
        robot.driveTrain.driveToDistance(40, 0.8, true);
        //drive forward to the line
        robot.driveTrain.driveToLine(12, 0.6);
        //drive forward a tiny bit so we are centered when we turn
        robot.driveTrain.driveToDistance(4, 0.4, true);
        //turn to the beacon
        robot.driveTrain.turn(80, 0.4, false);
        //set servo positions depending on what we see
        robot.beacon.updateServoPositionRed();
        //hit beacon
        robot.driveTrain.driveToDistance(8,0.35,true);*/
    }
}
