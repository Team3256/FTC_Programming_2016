package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

/**
 * Created by Team 2891 on 9/16/2016.
 */

@TeleOp(name="Teleop", group = "Linear OpMode")
public class Robot_Teleop extends LinearOpMode{

    //Create subsytem objects
    private DriveTrain drive = new DriveTrain();
    private SensorBase sensorBase = new SensorBase();

    //Create robot object
    private Robot robot = new Robot();

    //doubles for joystick values
    double left1 = 0, right1 = 0;

    /**
     * runOpMode()
     * Runs the teleop OpMode
     * @throws InterruptedException - ESTOP, cancel OpMode
     */
    public void runOpMode() throws InterruptedException {

        //Waits until the init button is pressed to start running the OpMOde
        super.waitForStart();

        //Initializes the robot and its subsystems
        robot.robotInit(super.hardwareMap, drive, sensorBase, "teleop");

        //Loop running while the Teleop OpMode is Active (Until the Stop Button is pressed or until the FMS stops the robot)
        while(opModeIsActive()) {
            left1 = -gamepad1.left_stick_y;
            right1 = -gamepad1.right_stick_y;
            drive.tankDrive(left1, right1);
            telemetry.addData("gyro", sensorBase.getAngle());
            telemetry.addData("Connected", sensorBase.gyroIsConnected());
            telemetry.update();
            //Wait for the next tick before looping again
            robot.waitForTick(40);
            //Stops the opMode if it is stopped in any way
            idle();
        }
    }
}
