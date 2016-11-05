package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;

import org.firstinspires.ftc.robotcore.external.Const;

/**
 * Created by Team 2891 on 9/16/2016.
 */

@Autonomous(name="Autonomous_Blue_2B", group = "Linear OpMode")
public class Robot_Autonomous_2Beac_Blue extends LinearOpMode{
    /**
     * runOpMode()
     * Runs the teleop OpMode
     * @throws InterruptedException - ESTOP, cancel OpMode
     */
    public void runOpMode() throws InterruptedException {

        //Waits until the init button is pressed to start running the OpMOde
        super.waitForStart();

        //AutoSeq autoSeq = new AutoSeq();
        //autoSeq.initialize(hardwareMap);
        AutonSequence_2Beac_Blue autonSequence = new AutonSequence_2Beac_Blue();
        autonSequence.initialize(hardwareMap);
        //Loop running while the Teleop OpMode is Active (Until the Stop Button is pressed or until the FMS stops the robot)
        while(opModeIsActive()) {
            /*if (!turn.isFinished())
                turn.run();
            else turn.end();*/
            //autoSeq.run(hardwareMap);
            autonSequence.run(hardwareMap);
            telemetry.addData("gyro", autonSequence.sensorBase.getAngle());
            telemetry.addData("internal gyro", autonSequence.turn1.sensorBase.getAngle());
            telemetry.addData("step", autonSequence.curr_step);
            telemetry.update();
            //Wait for the next tick before looping again
            autonSequence.robot.waitForTick(40);
            //Stops the opMode if it is stopped in any way
            //idle();
        }
    }
}
