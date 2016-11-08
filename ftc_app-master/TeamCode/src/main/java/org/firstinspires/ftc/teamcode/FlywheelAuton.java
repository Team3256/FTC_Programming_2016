package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;

/**
 * Created by Team 2891 on 9/16/2016.
 */

@Autonomous(name="Flytest")
public class FlywheelAuton extends LinearOpMode{

    //Create subsytem objects
    private DriveTrain drive = new DriveTrain();
    private Intake intake = new Intake();
    private SensorBase sensorBase = new SensorBase();
    private Beacon beacon = new Beacon();

    //Create robot object
    private Robot robot = new Robot();

    //doubles for joystick values

    /**
     * runOpMode()
     * Runs the teleop OpMode
     * @throws InterruptedException - ESTOP, cancel OpMode
     */
    public void runOpMode() throws InterruptedException {

        //Waits until the init button is pressed to start running the OpMOde
        super.waitForStart();

        //Initializes the robot and its subsystems
        robot.robotInit(super.hardwareMap, drive, intake, beacon, sensorBase, "teleop");

        //Loop running while the Teleop OpMode is Active (Until the Stop Button is pressed or until the FMS stops the robot)
        while(opModeIsActive()) {
            drive.runRight(0.6);
            drive.runLeft(0.6);
        //Wait for the next tick before looping again
            robot.waitForTick(40);
            //Stops the opMode if it is stopped in any way
            idle();
        }
    }
}
