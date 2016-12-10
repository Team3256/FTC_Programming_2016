package org.firstinspires.ftc.teamcode.subsystems;

import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.util.Range;

import org.firstinspires.ftc.teamcode.base.Constants;
import org.firstinspires.ftc.teamcode.base.PIDController;
import org.firstinspires.ftc.teamcode.base.Subsystem;
import org.firstinspires.ftc.teamcode.opmodes.Passthroughs;

import static org.firstinspires.ftc.teamcode.opmodes.Passthroughs.*;

public class DriveTrain extends Subsystem{
    //motors
    public DcMotor leftFront, leftBack, rightFront, rightBack;
    //singleton
    private static DriveTrain driveTrain = new DriveTrain();
    public PIDController pidController;
    private DriveTrain() {
    }
    double PID;

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


        //TODO: Retune for new GR
        setMaxSpeed(1600);
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
        return ticks*Constants.WHEEL_DIAMETER*Math.PI/Constants.TICKS_PER_ROTATION/Constants.GEAR_RATIO;
    }

    public double ticksToDegrees(double ticks) {
        return ticksToInches(ticks)*360/(Constants.ROBOT_TRACK*Math.PI);
    }

    public double inchesToTicks(double inches) {
        return inches*Constants.TICKS_PER_ROTATION/Constants.WHEEL_DIAMETER/Math.PI*Constants.GEAR_RATIO;
    }

    public double degreesToTicks(double degrees) {
        return inchesToTicks(Constants.ROBOT_TRACK*Math.PI*degrees/360);
    }

    public double getRightEncoderValue(){
        return (Math.abs(rightFront.getCurrentPosition())+Math.abs(rightBack.getCurrentPosition()))/2D;
    }

    public double getLeftEncoderValue(){
        return (Math.abs(leftFront.getCurrentPosition())+Math.abs(leftBack.getCurrentPosition()))/2D;
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

    public void driveToDistance(double inches, double power, boolean forward, double timeout){
        if (!forward){
            flipDirection();
        }
        setMode(DcMotor.RunMode.RUN_TO_POSITION);
        resetEncoders();
        setTargetPos((int) inchesToTicks(inches));
        setPower(power);
        double currtime = System.currentTimeMillis();
        while(isBusy()){
            if (!Passthroughs.opModeIsActive() || System.currentTimeMillis()-currtime>timeout){
                break;
            }
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

    public void driveToLine(double safety_inches, double power, double timeout){
        setMode(DcMotor.RunMode.RUN_TO_POSITION);
        resetEncoders();
        setTargetPos((int) inchesToTicks(safety_inches));
        setPower(power);
        double inittime = System.currentTimeMillis();
        while(isBusy()){
            if (!Passthroughs.opModeIsActive()|| System.currentTimeMillis()-inittime>timeout){
                break;
            }
            telemetry.addData("driving to line", "");
            if (getOds()>=0.5){
                break;
            }
        }
        telemetry.addData("drivetoline is done", "");
        setPower(0);
    }

    public void turnEncoder(double degrees, double power, boolean right) {
        resetEncoders();
        if (right) {
            leftBack.setMode(DcMotor.RunMode.RUN_TO_POSITION);
            leftFront.setMode(DcMotor.RunMode.RUN_TO_POSITION);
            leftBack.setTargetPosition((int)(degreesToTicks(degrees)/2));
            leftFront.setTargetPosition((int)(degreesToTicks(degrees)/2));
            rightBack.setDirection(DcMotor.Direction.FORWARD);
            rightFront.setDirection(DcMotor.Direction.FORWARD);
            setPower(power);
            while (isBusy()) {
                telemetry.addData("left degrees", ticksToDegrees(getLeftEncoderValue()));
                telemetry.update();
            }
            setPower(0);
        } else {
            rightBack.setMode(DcMotor.RunMode.RUN_TO_POSITION);
            rightFront.setMode(DcMotor.RunMode.RUN_TO_POSITION);
            rightBack.setTargetPosition((int) (degreesToTicks(degrees) / 2));
            rightFront.setTargetPosition((int) (degreesToTicks(degrees) / 2));
            leftBack.setDirection(DcMotor.Direction.REVERSE);
            leftFront.setDirection(DcMotor.Direction.REVERSE);
            setPower(power);
            while (isBusy()) {
                telemetry.addData("right degrees", ticksToDegrees(getRightEncoderValue()));
                telemetry.update();
            }
            setPower(0);
        }
    }

    public void turn(double degrees, double power, boolean right, double factor){
        double direction = right ? -1 : 1;
        pidController = new PIDController(0.001,0,0.0);
        setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        resetGyro();
        try {
            Thread.sleep(1000);
        } catch (InterruptedException e) {
            e.printStackTrace();
        }
        /*
        runLeft(direction * -power);
        runRight(direction * power);
        */
        while(true) {
            if (!Passthroughs.opModeIsActive()){
                break;
            }
            PID = pidController.calculatePID(Math.abs(getAngle()), degrees);
            if (PID < 0.45 && PID > 0)
                PID = 0.45;
            else if (PID > -0.45 && PID < 0)
                PID = -0.45;
            telemetry.addData("output",PID);
            telemetry.addData("degrees", Math.abs(sensorBase.getAngle()));
            telemetry.update();
            runLeft(PID * direction);
            runRight(-PID * direction);
            //if (Math.abs(getAngle())>= degrees * factor || !opModeIsActive()) {
            if (pidController.getError(Math.abs(getAngle()), degrees) <= 2){
                runLeft(0);
                runRight(0);
                break;
            }
        }
            setPower(0);
    }

    public void oneWheelTurnEncoder(double degrees, double power, boolean right) {
        resetEncoders();
        if (right) {
            leftBack.setMode(DcMotor.RunMode.RUN_TO_POSITION);
            leftFront.setMode(DcMotor.RunMode.RUN_TO_POSITION);
            leftBack.setPower(power);
            leftFront.setPower(power);
            leftBack.setTargetPosition((int) degreesToTicks(degrees));
            leftFront.setTargetPosition((int) degreesToTicks(degrees));

            while (leftBack.isBusy() && leftFront.isBusy()) {
                telemetry.addData("degrees", ticksToDegrees(getLeftEncoderValue()));
                telemetry.update();
            }
        } else {
            rightBack.setMode(DcMotor.RunMode.RUN_TO_POSITION);
            rightFront.setMode(DcMotor.RunMode.RUN_TO_POSITION);
            rightBack.setPower(power);
            rightFront.setPower(power);
            rightBack.setTargetPosition((int) degreesToTicks(degrees));
            rightFront.setTargetPosition((int) degreesToTicks(degrees));

            while (rightBack.isBusy() && rightFront.isBusy()) {
                telemetry.addData("degrees", ticksToDegrees(getRightEncoderValue()));
                telemetry.update();
            }
        }
        setPower(0);
    }

    public void oneWheelTurn(double degrees, double power, boolean right){
        setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        try {
            Thread.sleep(1000);
            resetGyro();
        } catch (InterruptedException e) {
            e.printStackTrace();
        }
        if (right) runLeft(power);
        else runRight(power);
        while(true){
            if (Math.abs(getAngle())>degrees || !opModeIsActive()) {
                runLeft(0);
                runRight(0);
                break;
            }
            telemetry.addData("degrees", Math.abs(sensorBase.getAngle()));
            telemetry.update();
        }
        setPower(0);
    }
}
