package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

/**
 * Created by Team 2891 on 9/16/2016.
 */

@TeleOp(name="Robot: Teleop", group = "Linear OpMode")
public class Robot_Teleop extends LinearOpMode {

    Robot robot = new Robot();

    public void runOpMode() throws InterruptedException {
        robot.teleop_init(hardwareMap);
        telemetry.addData("init",0);
        while(opModeIsActive()) {
            telemetry.addData("gyro", robot.getAngle());
            telemetry.update();
            robot.waitForTick(40);
            idle();
        }
    }

}
