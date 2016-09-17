package org.firstinspires.ftc.teamcode;

import android.hardware.Sensor;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.HardwareMap;

/**
 * Created by Team 2891 on 9/16/2016.
 */

@TeleOp(name="Teleop", group = "Linear OpMode")
public class Robot_Teleop extends LinearOpMode {
    //Create subsytem objects
    private DriveTrain drive;
    private SensorBase sensorBase;

    //Create robot object
    Robot robot = new Robot(super.hardwareMap,drive,sensorBase,"teleop");

    //doubles for joystick values
    double left1, right1;

    /**
     * runOpMode()
     * Runs the teleop OpMode
     * @throws InterruptedException
     */
    public void runOpMode() throws InterruptedException {
        super.waitForStart();
        while(opModeIsActive()) {
            left1 = -gamepad1.left_stick_y;
            right1 = -gamepad1.right_stick_y;
            drive.tankDrive(left1, right1);
            telemetry.addData("gyro", sensorBase.getAngle());
            telemetry.update();
            robot.waitForTick(40);
            idle();
        }
    }

}
