package org.firstinspires.ftc.teamcode.opmodes;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;

import org.firstinspires.ftc.teamcode.base.Robot;

@Autonomous(name = "AutoRedBeacons")

public class AutoRedBeacons extends LinearOpMode{
    private Robot robot = Robot.getInstance();

    @Override
    public void runOpMode() throws InterruptedException {
        Passthroughs.telemetry = telemetry;
        Passthroughs.opMode = this;
        robot.autonomousInit(hardwareMap);
        while(!robot.gyroIsReady()){
            telemetry.addData("gyro ready", "no");
            telemetry.addData("redval", robot.getRed());
            telemetry.update();
        }
        super.telemetry.addData("gyro ready", robot.getAngle());

        super.telemetry.addData("color sens", robot.getRed());
        super.telemetry.addData("ods", robot.getOds());
        super.telemetry.update();
        //wait for play button
        super.waitForStart();
        //one wheel turn towards beacon
        //robot.shooter.autoShootSequence();
        robot.driveTrain.oneWheelTurn(35,0.44,false);
        sleep(300);
        robot.driveTrain.driveToDistance(35, 0.45, true);
        //drive to white line
        robot.driveTrain.driveToLine(30, 0.4);
        //robot.driveTrain.driveToDistance(1,-0.44,true);
        sleep(1000);
        //turn to the beacon
        robot.driveTrain.turn(39, 0.44, false);
        /*robot.driveTrain.driveToDistance(3,0.45,true);
        sleep(1000);
        //set servo position depending on what color we see
        robot.beacon.updateServoPositionRed();
        //hit beacon
        robot.driveTrain.driveToDistance(4, 0.4, true);*/
        /*
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
