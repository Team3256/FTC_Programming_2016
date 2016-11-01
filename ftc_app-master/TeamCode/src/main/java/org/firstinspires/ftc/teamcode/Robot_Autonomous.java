package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;

import org.firstinspires.ftc.robotcore.external.Const;

/**
 * Created by Team 2891 on 9/16/2016.
 */

@Autonomous(name="Autonomous", group = "Linear OpMode")
public class Robot_Autonomous extends LinearOpMode{
    /**
     * runOpMode()
     * Runs the teleop OpMode
     * @throws InterruptedException - ESTOP, cancel OpMode
     */
    public void runOpMode() throws InterruptedException {

        //Waits until the init button is pressed to start running the OpMOde
        super.waitForStart();

        AutoSeq autoSeq = new AutoSeq();
        autoSeq.initialize(hardwareMap);

        PIDTurn pidTurn = new PIDTurn();
        pidTurn.initialize(hardwareMap);

        //Loop running while the Teleop OpMode is Active (Until the Stop Button is pressed or until the FMS stops the robot)
        while(opModeIsActive()) {
            //autoSeq.run();
            pidTurn.run();
            telemetry.addData("00", pidTurn.turnController);
            telemetry.addData("res", pidTurn.pidResult);
            telemetry.update();
            //Wait for the next tick before looping again
            autoSeq.robot.waitForTick(40);
            //Stops the opMode if it is stopped in any way
            //idle();
        }
    }
}
