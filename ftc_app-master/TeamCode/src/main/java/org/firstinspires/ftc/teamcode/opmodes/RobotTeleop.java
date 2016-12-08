package org.firstinspires.ftc.teamcode.opmodes;

import android.app.usage.ConfigurationStats;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

import org.firstinspires.ftc.robotcore.external.Const;
import org.firstinspires.ftc.teamcode.base.Constants;
import org.firstinspires.ftc.teamcode.base.Robot;

@TeleOp(name="Teleop", group = "Linear OpMode")
public class RobotTeleop extends LinearOpMode{
    //Create subsystem objects
    //Create robot object
    private Robot robot = Robot.getInstance();

    //doubles for joystick values
    double left1 = 0, right1 = 0;
    boolean a2 = false, y2 = false;
    boolean right_bumper1 = false, left_bumper1 = false;
    double right_trigger2 = 0;
    double left_trigger2 = 0;
    boolean x1 = false, b1 = false;
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
            a2 = gamepad2.a;
            y2 = gamepad2.y;
            left_bumper1 = gamepad1.left_bumper;
            left_trigger2 = gamepad2.left_trigger;
            right_bumper1 = gamepad1.right_bumper;
            right_trigger2 = gamepad2.right_trigger;
            x1 = gamepad2.x;
            b1 = gamepad2.b;
            /*
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
            */
            //left, right, reverse, slow

            robot.driveTrain.arcadeDrive(left1*left1*left1,right1*right1*right1,right_bumper1,left_bumper1);

            if (a2){
                robot.shooter.intakeBall();
            }
            else if (y2){
                robot.shooter.outtakeBall();
            }
            else{
                robot.shooter.stopIntake();
            }

            if (right_trigger2>0.5){
                robot.shooter.releaseBall();
                robot.shooter.runFly(Constants.CLOSE_SHOT_PID_POWER);
            }
            else if (left_trigger2>0.5){
                robot.shooter.releaseBall();
                robot.shooter.runFly(Constants.CLOSE_SHOT_PID_POWER);
            }
            else{
                robot.shooter.holdBall();
                robot.shooter.stopFly();
            }


            telemetry.addData("ods", robot.getOds());
            telemetry.addData("blue", robot.isBlue());
            telemetry.addData("blue val", robot.getBlue());

            telemetry.addData("angle", robot.getAngle());
            telemetry.addData("ticks", robot.driveTrain.getLeftEncoderValue() + " " + robot.driveTrain.getRightEncoderValue());
            telemetry.update();
            //Wait for the next tick before looping again
            robot.waitForTick(40);
            //Stops the opMode if it is stopped in any way
            //idle();
        }
    }
}
