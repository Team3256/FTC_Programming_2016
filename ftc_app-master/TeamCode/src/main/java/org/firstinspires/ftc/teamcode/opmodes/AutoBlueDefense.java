package org.firstinspires.ftc.teamcode.opmodes;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;

import org.firstinspires.ftc.robotcore.external.Telemetry;
import org.firstinspires.ftc.teamcode.base.Robot;

/**
 * Created by Team 2891 on 11/16/2016.
 */
@Autonomous(name = "AutoBlueDefense")
public class AutoBlueDefense extends LinearOpMode {

    private Robot robot = Robot.getInstance();
    private boolean beac1blue = false, beac2blue = false;
    public static Telemetry telemetryPass;

    @Override
    public void runOpMode() throws InterruptedException {

        telemetryPass = telemetry;
        robot.autonomousInit(hardwareMap);

        while(!robot.gyroIsReady()){
            telemetry.addData("gyro not ready", "");
            telemetry.update();
        }
        telemetry.addData("gyro is ready", "");
        telemetry.update();

        super.waitForStart();

        //sleep(15 * 1000);
        robot.driveTrain.driveToDistance(60, 0.8);
        sleep(1000);
        robot.driveTrain.turn(45, 0.23, true);
        sleep(1000);
        robot.driveTrain.driveToRamp(70,0.8);
    }
}
