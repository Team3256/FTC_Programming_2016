package org.firstinspires.ftc.teamcode.subsystems;

import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.util.Range;

import org.firstinspires.ftc.robotcore.external.Telemetry;
import org.firstinspires.ftc.teamcode.base.Constants;
import org.firstinspires.ftc.teamcode.base.Subsystem;
import org.firstinspires.ftc.teamcode.opmodes.TelemetryHolder;

import static org.firstinspires.ftc.teamcode.opmodes.TelemetryHolder.telemetry;

public class DriveTrain extends Subsystem{
    //motors
    private DcMotor leftFront, leftBack, rightFront, rightBack;
    //singleton
    private static DriveTrain driveTrain = new DriveTrain();

    private DriveTrain() {

    }

    public void init(HardwareMap hardwareMap){
        //initialize the motors
        leftFront = hardwareMap.dcMotor.get("leftFront");
        leftBack = hardwareMap.dcMotor.get("leftBack");
        rightFront = hardwareMap.dcMotor.get("rightFront");
        rightBack = hardwareMap.dcMotor.get("rightBack");

        //set motor directions
        leftFront.setDirection(DcMotor.Direction.FORWARD);
        leftBack.setDirection(DcMotor.Direction.FORWARD);
        rightFront.setDirection(DcMotor.Direction.REVERSE);
        rightBack.setDirection(DcMotor.Direction.REVERSE);

        //set motors to brake
        leftFront.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        rightFront.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        leftBack.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        rightBack.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);

        setPower(0);

        sensorBase.initSensorBase(hardwareMap);
        sensorBase.resetSensors();
        resetEncoders();


        //default is 4000, need to determine this emperically (ticks per second)
        setMaxSpeed(2100);
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
    }

    public boolean isBusy(){
        return leftFront.isBusy() && rightFront.isBusy() && leftBack.isBusy() && rightBack.isBusy();
    }

    public void setMaxSpeed(int ticksPerSec){
        leftFront.setMaxSpeed(ticksPerSec);
        rightFront.setMaxSpeed(ticksPerSec);
        leftBack.setMaxSpeed(ticksPerSec);
        rightBack.setMaxSpeed(ticksPerSec);
    }

    public void arcadeDrive(double throttle, double turn, boolean reverse, boolean slow) {
        setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        throttle = Range.clip(throttle, -1, 1);
        turn = Range.clip(turn, -1, 1);
        if (reverse) {
            throttle = -throttle;
        }
        double left = throttle - turn;
        double right = throttle + turn;
        if (slow){
            left/=2;
            right/=2;
        }
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
        return (rightFront.getCurrentPosition()+rightBack.getCurrentPosition())/2D;
    }

    public double getLeftEncoderValue(){
        return (leftFront.getCurrentPosition()+leftBack.getCurrentPosition())/2D;
    }

    public double getAverageEncoderValue(){
        return (getLeftEncoderValue() + getRightEncoderValue()) / 2;
    }

    public void resetEncoders(){
        leftFront.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        leftBack.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        rightFront.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        rightBack.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        try {
            Thread.sleep(100);
        } catch (InterruptedException e) {
            e.printStackTrace();
        }
        leftFront.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        leftBack.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        rightFront.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        rightBack.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
    }

    public static DriveTrain getInstance() {
        return driveTrain;
    }

    public void setPower(double power){
        runLeft(power);
        runRight(power);
    }

    public void flipDirection(){
        leftFront.setDirection(DcMotor.Direction.REVERSE);
        leftBack.setDirection(DcMotor.Direction.REVERSE);
        rightFront.setDirection(DcMotor.Direction.FORWARD);
        rightBack.setDirection(DcMotor.Direction.FORWARD);
    }

    public void unFlipDirection(){
        leftFront.setDirection(DcMotor.Direction.FORWARD);
        leftBack.setDirection(DcMotor.Direction.FORWARD);
        rightFront.setDirection(DcMotor.Direction.REVERSE);
        rightBack.setDirection(DcMotor.Direction.REVERSE);
    }

    public void driveToDistance(double inches, double power, boolean forward){
        if (!forward){
            flipDirection();
        }
        setMode(DcMotor.RunMode.RUN_TO_POSITION);
        resetEncoders();
        setTargetPos((int) inchesToTicks(inches));
        setPower(power);
        while(isBusy()){
            if (inchesToTicks(inches)<=Math.abs(getAverageEncoderValue())) break;
            telemetry.addData("distance", ticksToInches(Math.abs(getAverageEncoderValue())));
            telemetry.addData("angle", getAngle());
            telemetry.update();
        }
        setPower(0);
        if (!forward){
            unFlipDirection();
        }
    }

    public void driveToRamp(double safety_inches, double power){
        setMode(DcMotor.RunMode.RUN_TO_POSITION);
        resetEncoders();
        setTargetPos((int) inchesToTicks(safety_inches));
        setPower(power);
        while(isBusy()){
            if (getPitch()>=28) break;
        }
        setPower(0);
    }

    public void driveToLine(double safety_inches, double power){
        setMode(DcMotor.RunMode.RUN_TO_POSITION);
        resetEncoders();
        setTargetPos((int) inchesToTicks(safety_inches));
        setPower(power);
        while(isBusy()){
            telemetry.addData("driving to line", "");
            if (getOds()>=0.5){
                break;
            }
        }
        telemetry.addData("drivetoline is done", "");
        setPower(0);
    }

    public void turn(double degrees, double power, boolean right){
        telemetry.addData("INSIDE TURN------------", "---");
        double direction = right ? -1 : 1;
        setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        try {
            resetGyro();
            Thread.sleep(250);
        } catch (InterruptedException e) {
            e.printStackTrace();
        }
        while(true) {
            telemetry.addData("degrees", Math.abs(sensorBase.getAngle()));
            telemetry.update();
            if (Math.abs(getAngle())>degrees) {
                runLeft(0);
                runRight(0);
                break;
            } else {
                runLeft(direction * -power);
                runRight(direction * power);
            }
        }
            setPower(0);
    }

    public void oneWheelTurn(double degrees, double power, boolean right){
        setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        resetGyro();
        while(true){
            if (Math.abs(getAngle())>degrees) {
                runLeft(0);
                runRight(0);
                break;
            }
            telemetry.addData("degrees", Math.abs(sensorBase.getAngle()));
            telemetry.update();
            if (right) runLeft(power);
            else runRight(power);
        }
        setPower(0);
    }

}
