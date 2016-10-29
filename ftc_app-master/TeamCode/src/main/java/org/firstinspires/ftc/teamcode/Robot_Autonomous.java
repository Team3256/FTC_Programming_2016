package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;

/**
 * Created by Team 2891 on 9/16/2016.
 */

@Autonomous(name="Autonomous", group = "Linear OpMode")
public class Robot_Autonomous extends LinearOpMode{

    //Create subsytem objects
    private DriveTrain drive = new DriveTrain();
    private Intake intake = new Intake();
    private SensorBase sensorBase = new SensorBase();

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
        robot.robotInit(super.hardwareMap, drive, intake, sensorBase, "autonomous");
        sensorBase.resetSensors();
        drive.resetEncoders();
        sensorBase.disableLED();
        //BangBangDriveForward bangBangDriveForward = new BangBangDriveForward();
        //bangBangDriveForward.initialize(hardwareMap);
        PIDDriveForward pidDriveForward = new PIDDriveForward();
        pidDriveForward.initialize(hardwareMap);

        //PIDTurn pidTurn = new PIDTurn();
        //pidTurn.initialize(hardwareMap);

        //Loop running while the Teleop OpMode is Active (Until the Stop Button is pressed or until the FMS stops the robot)
        while(opModeIsActive()) {
            //bangBangDriveForward.run();
            if (!pidDriveForward.isFinished())
                pidDriveForward.run();
            /*else
                pidTurn.run();*/
            telemetry.addData("Current Ticks", drive.getEncoderValue());
            telemetry.addData("Front Pos", drive.getRightFront().getCurrentPosition());
            telemetry.addData("Back Pos", drive.getRightBack().getCurrentPosition());
            telemetry.addData("Current Inches", drive.ticksToInches(drive.getEncoderValue()));
            telemetry.update();
            //Wait for the next tick before looping again
            robot.waitForTick(40);
            //Stops the opMode if it is stopped in any way
            idle();
        }
    }
}
