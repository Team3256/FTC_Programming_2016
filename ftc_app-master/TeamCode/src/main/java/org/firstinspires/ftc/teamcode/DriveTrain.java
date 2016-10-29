package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.util.Range;

/**
 * Created by Eric on 9/16/2016.
 */
public class DriveTrain {

    // Hardware Map to store all our electrical component objects
    private HardwareMap hm = null;

    //Drive Motors
    private DcMotor leftFront, leftBack, rightFront, rightBack;

    /**
     * DriveTrain()
     * Empty Constructor
     */
    public DriveTrain(){

    }

    /**
     * init_Drive()
     * Initializes the whole drivetrain, including all the motors
     * @param hm Instance of the HardwareMap of the Robot
     * @param state State to initialize Robot in
     */
    public void init_Drive(HardwareMap hm, Robot.State state){
        //Choose what motor mode to run in: Encoder or No Encoder
        DcMotor.RunMode mode;

        if (state.equals(Robot.State.AUTONOMOUS)) {

            mode = DcMotor.RunMode.RUN_USING_ENCODER;

        }
        else {
            mode = DcMotor.RunMode.RUN_WITHOUT_ENCODER;
        }

        //initialize the hardware map
        this.hm = hm;

        //initialize the motors
        leftFront = this.hm.dcMotor.get("leftFront");
        leftBack = this.hm.dcMotor.get("leftBack");
        rightFront = this.hm.dcMotor.get("rightFront");
        rightBack = this.hm.dcMotor.get("rightBack");

        //set motor directions
        leftFront.setDirection(DcMotor.Direction.FORWARD);
        leftBack.setDirection(DcMotor.Direction.FORWARD);
        rightFront.setDirection(DcMotor.Direction.REVERSE);
        rightBack.setDirection(DcMotor.Direction.REVERSE);
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
     * Runs the left DriveTrain motors
     * @param speed speed to run the left drive motors
     */
    public void runLeft(double speed){
        leftFront.setPower(speed);
        leftBack.setPower(speed);
    }

    /**
     * runRight()
     * Runs the right DriveTrain motors
     * @param speed speed to run the right drive motors
     */
    public void runRight(double speed){
        rightFront.setPower(speed);
        rightBack.setPower(speed);
    }

    /**
     * tankDrive()
     * TankDrive method for controlling the robot
     * @param left left drivetrain output
     * @param right right drivetrain output
     */
    public void tankDrive(double left, double right){
        runLeft(left);
        runRight(right);
    }

    public void arcadeDrive(double throttle, double turn){
        throttle = Range.clip(throttle,-1,1);
        turn = Range.clip(turn,-1,1);
        double left = throttle - turn;
        double right = throttle + turn;
        runLeft(left);
        runRight(right);
    }

    public double ticksToInches(double ticks) {
        return ticks*Constants.WHEEL_DIAMETER*Math.PI/Constants.TICKS_PER_ROTATION;
    }

    public double getEncoderValue(){
        return (rightFront.getCurrentPosition()+rightBack.getCurrentPosition())/2;
    }

    public void resetEncoders(){
        leftFront.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        leftBack.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        rightFront.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        rightBack.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);

        leftFront.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        leftBack.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        rightFront.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        rightBack.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
    }

    public DcMotor getLeftFront() {
        return leftFront;
    }

    public DcMotor getLeftBack() {
        return leftBack;
    }

    public DcMotor getRightFront() {
        return rightFront;
    }

    public DcMotor getRightBack() {
        return rightBack;
    }
}
