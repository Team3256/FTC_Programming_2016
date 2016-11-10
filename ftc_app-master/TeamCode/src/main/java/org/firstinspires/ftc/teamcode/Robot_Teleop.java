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
    private Intake intake = new Intake();
    private SensorBase sensorBase = new SensorBase();
    private Beacon beacon = new Beacon();

    //Create robot object
    private Robot robot = new Robot();

    //doubles for joystick values
    double left1 = 0, right1 = 0;
    boolean intake_button = false, outtake_button = false;
    boolean x_button = false, y_button = false;

    /**
     * runOpMode()
     * Runs the teleop OpMode
     * @throws InterruptedException - ESTOP, cancel OpMode
     */
    public void runOpMode() throws InterruptedException {

        //Waits until the init button is pressed to start running the OpMOde

        //Initializes the robot and its subsystems
        robot.robotInit(super.hardwareMap, drive, intake, beacon, sensorBase, "teleop");

        super.waitForStart();
        //Loop running while the Teleop OpMode is Active (Until the Stop Button is pressed or until the FMS stops the robot)
        while(opModeIsActive()) {
            left1 = -gamepad1.left_stick_y;
            right1 = -gamepad1.right_stick_x;
            intake_button = gamepad1.right_bumper;
            outtake_button = gamepad1.left_bumper;
            x_button = gamepad1.x;
            y_button = gamepad1.y;

            drive.arcadeDrive(left1,right1);
            if (outtake_button){
                beacon.setLeftBangPos();
            }
            else beacon.setLeftNeutralPos();

            if (intake_button){
                beacon.setRightBangPos();
            }
            else beacon.setRightNeutralPos();

            sensorBase.disableBeaconLED();

            telemetry.addData("gyro", sensorBase.getAngle());
            telemetry.addData("ods", sensorBase.getOds());
            telemetry.addData("blue", sensorBase.isBlue());
            telemetry.addData("blue val", sensorBase.getBlue());

            telemetry.addData("red val", sensorBase.getRed());
            telemetry.addData("leftBeac", beacon.getLeftPos());
            telemetry.addData("rightBeac", beacon.getRightPos());
            telemetry.update();
            //Wait for the next tick before looping again
            robot.waitForTick(40);
            //Stops the opMode if it is stopped in any way
            //idle();
        }
    }
}
