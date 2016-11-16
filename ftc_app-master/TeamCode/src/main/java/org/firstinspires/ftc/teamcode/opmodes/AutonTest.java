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
    public static Telemetry telemetryPass;
    @Override
    public void runOpMode() throws InterruptedException {
        telemetryPass = telemetry;
        robot.teleopInit(hardwareMap);//teleop for now
        super.waitForStart();
        robot.driveTrain.driveToDistance(12, 0.5);
        robot.driveTrain.turn(90, 0.2, true);
    }
}
