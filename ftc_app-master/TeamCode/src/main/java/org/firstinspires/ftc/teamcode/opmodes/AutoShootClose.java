package org.firstinspires.ftc.teamcode.opmodes;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;

import org.firstinspires.ftc.teamcode.base.Robot;

@Autonomous(name = "AutoShootClose")
public class AutoShootClose extends LinearOpMode {
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

        //sleep(15 * 1000);
        robot.driveTrain.driveToDistance(6, 0.8,true);
        robot.shooter.autoShootSequence();
        sleep(1000);
        robot.driveTrain.driveToDistance(40,0.7,true);
        robot.driveTrain.driveToDistance(5,0.8,false);
        robot.driveTrain.driveToDistance(12,0.8,true);
        /*
        robot.driveTrain.turn(80,0.23, true);
        sleep(1000);
        robot.driveTrain.driveToRamp(70,0.8);*/
    }
}
