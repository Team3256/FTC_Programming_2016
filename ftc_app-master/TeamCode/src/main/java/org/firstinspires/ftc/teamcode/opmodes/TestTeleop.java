package org.firstinspires.ftc.teamcode.opmodes;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

import org.firstinspires.ftc.teamcode.base.Robot;

@TeleOp(name="Test", group = "Linear OpMode")
public class TestTeleop extends LinearOpMode{
    //Create subsystem objects
    //Create robot object
    private Robot robot = Robot.getInstance();

    //doubles for joystick values
    double left1 = 0, right1 = 0;
    boolean a2 = false, y2 = false;
    boolean x2 = false, b2 = false;
    boolean intake_button = false, outtake_button = false;
    boolean prev_intake_button = false, prev_outtake_button = false;
    boolean intake_toggle = false, outtake_toggle = false;
    /*
    boolean prev_intake_button = false, prev_outtake_button = false;
    boolean intake_toggle = false, outtake_toggle = false;*/
    boolean right_bumper1 = false, left_bumper1 = false;
    double right_trigger1 = 0;
    double fly_speed = 0.375;
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
            a2 = gamepad1.a;
            y2 = gamepad1.y;
            x2 = gamepad1.x;
            b2 = gamepad1.b;
            left_bumper1 = gamepad1.left_bumper;
            right_bumper1 = gamepad1.right_bumper;
            intake_button = gamepad2.a;
            outtake_button = gamepad2.b;
            right_trigger1 = gamepad2.right_trigger;
            if (intake_button && !prev_intake_button) {
                if (!intake_toggle) {
                    fly_speed+=0.03;
                    intake_toggle = true;
                } else {
                    intake_button = false;
                    fly_speed+=0.03;
                }
            }

            if (outtake_button && !prev_outtake_button) {
                if (!outtake_toggle) {
                    fly_speed-=0.03;
                    outtake_toggle = true;
                } else {
                    fly_speed-=0.03;
                    outtake_toggle = false;
                }
            }
            prev_intake_button = intake_button;
            prev_outtake_button = outtake_button;


            if (a2){
                robot.shooter.intakeBall();
            }
            else if (y2){
                robot.shooter.outtakeBall();
            }
            else robot.shooter.stopIntake();

            if (right_trigger1>0.5){
                robot.shooter.runFly(fly_speed);
            }
            else robot.shooter.stopFly();

            telemetry.addData("speed", robot.shooter.getPower());

            telemetry.addData("ods", robot.getOds());
            telemetry.addData("blue", robot.isBlue());
            telemetry.addData("blue val", robot.getBlue());

            telemetry.addData("angle", robot.getAngle());
            telemetry.addData("ticks", robot.driveTrain.getLeftEncoderValue() + " " + robot.driveTrain.getRightEncoderValue());
            telemetry.addData("leftBeacon", robot.beacon.getLeftPos());
            telemetry.addData("rightBeacon", robot.beacon.getRightPos());
            telemetry.update();
            //Wait for the next tick before looping again
            robot.waitForTick(40);
            //Stops the opMode if it is stopped in any way
            //idle();
        }
    }
}
