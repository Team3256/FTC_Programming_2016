package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.HardwareMap;

/**
 * Created by Eric on 9/16/2016.
 */
public class DriveTrain {

    // Hardware Map to store all our electrical component objects
    private HardwareMap hm;

    //Drive Motors
    private DcMotor leftFront, leftBack, rightFront, rightBack;

    /**
     * DriveTrain()
     * @param hm Instance of the HardwareMap of the Robot
     * @param state State of the robot
     */
    public DriveTrain(HardwareMap hm, Robot.State state){
        //Choose what motor mode to run in: Encoder or No Encoder
        DcMotor.RunMode mode;

        if (state == Robot.State.AUTONOMOUS) {
            mode = DcMotor.RunMode.RUN_USING_ENCODER;
        }
        else {
            mode = DcMotor.RunMode.RUN_WITHOUT_ENCODER;
        }

        //initialize the hardware map
        this.hm = hm;

        //initialize the motors
        leftFront = hm.dcMotor.get("leftFront");
        leftBack = hm.dcMotor.get("leftBack");
        rightFront = hm.dcMotor.get("rightFront");
        rightBack = hm.dcMotor.get("rightBack");

        //set motor directions
        leftFront.setDirection(DcMotor.Direction.REVERSE);
        leftBack.setDirection(DcMotor.Direction.REVERSE);
        rightFront.setDirection(DcMotor.Direction.FORWARD);
        rightBack.setDirection(DcMotor.Direction.FORWARD);

        //set motor mode
        leftFront.setMode(mode);
        leftBack.setMode(mode);
        rightFront.setMode(mode);
        rightBack.setMode(mode);

        //initialize all motors to 0 power
        leftFront.setPower(0);
        leftBack.setPower(0);
        rightFront.setPower(0);
        rightBack.setPower(0);
    }

    /**
     * runLeft()
     * @param speed speed to run the left drive motors
     */
    public void runLeft(double speed){
        leftFront.setPower(speed);
        leftBack.setPower(speed);
    }

    /**
     * runRight()
     * @param speed speed to run the right drive motors
     */
    public void runRight(double speed){
        rightFront.setPower(speed);
        rightBack.setPower(speed);
    }

    /**
     * tankDrive()
     * @param left left drivetrain output
     * @param right right drivetrain output
     */
    public void tankDrive(double left, double right){
        runLeft(left);
        runRight(right);
    }
}
