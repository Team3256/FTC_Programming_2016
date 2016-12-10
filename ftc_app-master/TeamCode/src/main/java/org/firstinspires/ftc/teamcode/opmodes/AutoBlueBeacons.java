package org.firstinspires.ftc.teamcode.opmodes;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;

import org.firstinspires.ftc.teamcode.base.Robot;

@Autonomous(name = "AutoBlueBeacons")

public class AutoBlueBeacons extends LinearOpMode{
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
        robot.shooter.autoShootSequence();
        sleep(300);
        robot.driveTrain.driveToDistance(6, 0.5, true, 100000);
        robot.driveTrain.turn(37, 1, false, 1);
        robot.driveTrain.driveToDistance(35, 0.45, true, 10000);
        //drive to white line
        robot.driveTrain.driveToLine(30, 0.4, 5000);
        robot.driveTrain.turn(25, 1, false, 1);
        robot.driveTrain.driveToDistance(6, 0.6, true,3000);
        robot.beacon.updateServoPositionBlue();
        robot.driveTrain.driveToDistance(6,0.5,false,3000);
        robot.driveTrain.driveToDistance(6,0.6,true,3000);
        while (opModeIsActive()){

        }
    }
}
