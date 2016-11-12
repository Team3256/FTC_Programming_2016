package org.firstinspires.ftc.teamcode.subsystems;

import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.util.Range;

import org.firstinspires.ftc.teamcode.base.Constants;
import org.firstinspires.ftc.teamcode.base.Robot;

/**
 * Created by Team 6696 on 11/11/2016.
 */
public class DriveTrain {
    private HardwareMap hardwareMap;
    //motors
    private DcMotor leftFront, leftBack, rightFront, rightBack;
    //singleton
    private static DriveTrain driveTrain = new DriveTrain();

    private DriveTrain() {

    }

    public void initDrive(HardwareMap hardwareMap, Robot.State state){
        //Choose what motor mode to run in: Encoder or No Encoder
        DcMotor.RunMode mode;

        if (state.equals(Robot.State.AUTONOMOUS))
            mode = DcMotor.RunMode.RUN_TO_POSITION;
        else
            mode = DcMotor.RunMode.RUN_WITHOUT_ENCODER;

        //initialize the hardware map
        this.hardwareMap = hardwareMap;

        //initialize the motors
        leftFront = this.hardwareMap.dcMotor.get("leftFront");
        leftBack = this.hardwareMap.dcMotor.get("leftBack");
        rightFront = this.hardwareMap.dcMotor.get("rightFront");
        rightBack = this.hardwareMap.dcMotor.get("rightBack");

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

        leftFront.setPower(0);
        leftBack.setPower(0);
        rightFront.setPower(0);
        rightBack.setPower(0);
    }

    public void setMode(DcMotor.RunMode mode){
        leftFront.setMode(mode);
        leftBack.setMode(mode);
        rightFront.setMode(mode);
        rightBack.setMode(mode);
    }

    public DcMotor.RunMode getMode() {
        return leftFront.getMode();
    }

    public void runLeft(double speed) {
        leftFront.setPower(speed);
        leftBack.setPower(speed);
    }

    public void runRight(double speed) {
        rightFront.setPower(speed);
        rightBack.setPower(speed);
    }

    public void tankDrive(double left, double right) {
        runLeft(left);
        runRight(right);
    }

    public void setTargetPos(int pos) {
        leftFront.setTargetPosition(pos);
        rightFront.setTargetPosition(pos);
        leftBack.setTargetPosition(pos);
        rightBack.setTargetPosition(pos);
        leftFront.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        rightFront.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        leftBack.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        rightBack.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
    }

    public int getTargetPos() {
        return leftFront.getTargetPosition();
    }

    public void arcadeDrive(double throttle, double turn) {
        throttle = Range.clip(throttle, -1, 1);
        turn = Range.clip(turn, -1, 1);
        double left = throttle - turn;
        double right = throttle + turn;
        runLeft(left);
        runRight(right);
    }

    public double ticksToInches(double ticks) {
        return ticks* Constants.WHEEL_DIAMETER*Math.PI/Constants.TICKS_PER_ROTATION;
    }

    public double inchesToTicks(double inches) {
        return inches*Constants.TICKS_PER_ROTATION/Constants.WHEEL_DIAMETER/Math.PI;
    }

    public double getRightEncoderValue(){
        return (rightFront.getCurrentPosition()+rightBack.getCurrentPosition())/2;
    }

    public double getLeftEncoderValue(){
        return (leftFront.getCurrentPosition()+leftBack.getCurrentPosition())/2;
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

    public static DriveTrain getInstance() {
        return driveTrain;
    }
}
