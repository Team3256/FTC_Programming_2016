package org.firstinspires.ftc.teamcode.opmodes;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

import org.firstinspires.ftc.teamcode.base.Robot;

/**
 * Created by Team 2891 on 9/16/2016.
 */

@TeleOp(name="Teleop", group = "Linear OpMode")
public class RobotTeleop extends LinearOpMode{
    //Create subsytem objects
    //Create robot object
    private Robot robot = Robot.getInstance();

    //doubles for joystick values
    double left1 = 0, right1 = 0;
    boolean intake_button = false, outtake_button = false;
    boolean prev_intake_button = false, prev_outtake_button = false;
    boolean intake_toggle = false, outtake_toggle = false;
    boolean x_button = false, y_button = false;

    /**
     * runOpMode()
     * Runs the teleop OpMode
     * @throws InterruptedException - ESTOP, cancel OpMode
     */
    public void runOpMode() throws InterruptedException {
        //Initializes the robot and its subsystems
        robot.teleopInit(super.hardwareMap);

        super.waitForStart();
        //Loop running while the Teleop OpMode is Active (Until the Stop Button is pressed or until the FMS stops the robot)
        while(opModeIsActive()) {
            left1 = -gamepad1.left_stick_y;
            right1 = -gamepad1.right_stick_x;
            intake_button = gamepad1.right_bumper;
            outtake_button = gamepad1.left_bumper;

            if (intake_button && !prev_intake_button) {
                if (!intake_toggle) {
                    robot.beacon.setRightBangPos();
                    intake_toggle = true;
                } else {
                    robot.beacon.setRightNeutralPos();
                    intake_toggle = false;
                }
            }

            if (outtake_button && !prev_outtake_button) {
                if (!outtake_toggle) {
                    robot.beacon.setLeftBangPos();
                    outtake_toggle = true;
                } else {
                    robot.beacon.setLeftNeutralPos();
                    outtake_toggle = false;
                }
            }

            prev_intake_button = intake_button;
            prev_outtake_button = outtake_button;

            x_button = gamepad1.x;
            y_button = gamepad1.y;
            robot.driveTrain.arcadeDrive(left1,right1);

            telemetry.addData("ods", robot.getOds());
            telemetry.addData("blue", robot.isBlue());
            telemetry.addData("blue val", robot.getBlue());

            telemetry.addData("angle", robot.getAngle());
            telemetry.addData("ticks", robot.driveTrain.getLeftEncoderValue() + " " + robot.driveTrain.getRightEncoderValue());
            telemetry.addData("gyroPitch", robot.getPitch());
            telemetry.update();
            //Wait for the next tick before looping again
            robot.waitForTick(40);
            //Stops the opMode if it is stopped in any way
            //idle();
        }
    }
}
