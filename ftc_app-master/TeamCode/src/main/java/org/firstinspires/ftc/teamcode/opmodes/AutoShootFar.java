package org.firstinspires.ftc.teamcode.opmodes;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;

import org.firstinspires.ftc.teamcode.base.Robot;

@Autonomous(name = "AutoShootFar")
public class AutoShootFar extends LinearOpMode {
    private Robot robot = Robot.getInstance();

    @Override
    public void runOpMode() throws InterruptedException {
        TelemetryHolder.telemetry = telemetry;
        robot.autonomousInit(hardwareMap);

        /*while(!robot.gyroIsReady()){
            telemetry.addData("gyro ready", "no");
            telemetry.update();
        }
        telemetry.addData("gyro ready", "yes");
        telemetry.update();*/

        super.waitForStart();

        sleep(5 * 1000);
        robot.driveTrain.driveToDistance(16, 0.8,true);
        robot.shooter.autoShootSequence();
        sleep(1000);
        robot.driveTrain.driveToDistance(45,0.7,true);
        robot.driveTrain.driveToDistance(5,0.8,false);
        robot.driveTrain.driveToDistance(5,0.8,true);
    }
}
