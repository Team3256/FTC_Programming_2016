package org.firstinspires.ftc.teamcode.subsystems;

import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.util.Range;

import org.firstinspires.ftc.teamcode.base.Constants;
import org.firstinspires.ftc.teamcode.base.Robot;
import org.firstinspires.ftc.teamcode.base.Subsystem;
import org.firstinspires.ftc.teamcode.opmodes.AutonTest;

/**
 * Created by Team 6696 on 11/11/2016.
 */
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
        leftFront.setDirection(DcMotor.Direction.REVERSE);
        leftBack.setDirection(DcMotor.Direction.REVERSE);
        rightFront.setDirection(DcMotor.Direction.FORWARD);
        rightBack.setDirection(DcMotor.Direction.FORWARD);

        leftFront.setPower(0);
        leftBack.setPower(0);
        rightFront.setPower(0);
        rightBack.setPower(0);

        sensorBase.initSensorBase(hardwareMap);
        sensorBase.resetSensors();
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

    public boolean isBusy(){
        return leftFront.isBusy() && rightFront.isBusy() && leftBack.isBusy()&& rightBack.isBusy();
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

    public static DriveTrain getInstance() {
        return driveTrain;
    }

    public void setPower(double power){
        leftFront.setPower(power);
        leftBack.setPower(power);
        rightFront.setPower(power);
        rightBack.setPower(power);
    }
    public void driveToDistance(double inches, double power){
        setMode(DcMotor.RunMode.RUN_TO_POSITION);
        resetEncoders();
        setTargetPos((int) inchesToTicks(inches));
        setPower(power);
        while(isBusy()){
            AutonTest.telemetryPass.addData("Driving " + inches + " inches with power: ", power);
        }
        setPower(0);
    }
    public void turn(double degrees, double power, boolean right){
        double direction = right ? 1 : -1;
        setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        resetGyro();
        while(true){
            if ((Math.abs(degrees-Math.abs(sensorBase.getAngle()))<2)){
                break;
            }
            else {
                DriveTrain drive = DriveTrain.getInstance();
                drive.runLeft(direction*-power);
                drive.runRight(direction * power);
                AutonTest.telemetryPass.addData("Angle: ", sensorBase.getAngle());
            }
        }
    }
}
