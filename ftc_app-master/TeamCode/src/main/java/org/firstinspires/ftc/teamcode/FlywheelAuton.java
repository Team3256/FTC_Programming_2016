package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;

import org.firstinspires.ftc.teamcode.base.Robot;
import org.firstinspires.ftc.teamcode.base.SensorBase;
import org.firstinspires.ftc.teamcode.subsystems.Beacon;
import org.firstinspires.ftc.teamcode.subsystems.DriveTrain;
import org.firstinspires.ftc.teamcode.subsystems.Intake;

/**
 * Created by Team 2891 on 9/16/2016.
 */

@Autonomous(name="Flytest")
public class FlywheelAuton extends LinearOpMode{

    //Create subsytem objects
    private DriveTrain drive = DriveTrain.getInstance();
    private Intake intake = Intake.getIntake();
    private SensorBase sensorBase = SensorBase.getInstance();
    private Beacon beacon = Beacon.getBeacon();

    //Create robot object
    private Robot robot = Robot.getInstance();

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
