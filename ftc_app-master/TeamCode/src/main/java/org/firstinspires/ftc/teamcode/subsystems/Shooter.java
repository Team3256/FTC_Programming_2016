package org.firstinspires.ftc.teamcode.subsystems;

import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.Servo;

import org.firstinspires.ftc.teamcode.base.Subsystem;

public class Shooter extends Subsystem {
    private static Shooter shooter = new Shooter();

    private Shooter(){

    }

    public static Shooter getInstance(){
        return shooter;
    }

    private DcMotor leftFly, rightFly;
    private DcMotor intake;

    private Servo dongerLord;

    @Override
    public void init(HardwareMap hardwareMap) {
        leftFly = hardwareMap.dcMotor.get("leftFly");
        rightFly = hardwareMap.dcMotor.get("rightFly");
        leftFly.setDirection(DcMotor.Direction.FORWARD);
        rightFly.setDirection(DcMotor.Direction.REVERSE);
        leftFly.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.FLOAT);
        rightFly.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.FLOAT);
        leftFly.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        rightFly.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        //TODO: tune
        leftFly.setMaxSpeed(38000);
        rightFly.setMaxSpeed(38000);
        leftFly.setPower(0);
        rightFly.setPower(0);

        intake = hardwareMap.dcMotor.get("intake");
        intake.setDirection(DcMotor.Direction.FORWARD);
        intake.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.FLOAT);
        intake.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        intake.setPower(0);

        dongerLord = hardwareMap.servo.get("dongerLord");
        holdBall();
    }

    public void intakeBall() {
        holdBall();
        intake.setPower(1);
    }

    public void outtakeBall() {
        holdBall();
        intake.setPower(-1);
    }

    public void stopIntake(){
        intake.setPower(0);
    }
    public void holdBall() {
        dongerLord.setPosition(0.7);
    }

    public void releaseBall() {
        dongerLord.setPosition(0);
    }

    //TODO: tune
    public void runFly(double power){
        leftFly.setPower(power);
        rightFly.setPower(power);
    }

    public void stopFly(){
        leftFly.setPower(0);
        rightFly.setPower(0);
    }

    public void setFlyRunMode(DcMotor.RunMode runMode){
        leftFly.setMode(runMode);
        rightFly.setMode(runMode);
    }
    //only use in auto
    public void autoShootSequence(){
        setFlyRunMode(DcMotor.RunMode.RUN_USING_ENCODER);
        resetEncoders();
        //start flywheel
        runFly(1);
        //wait a second to ramp up
        try {
            Thread.sleep(1000);
            releaseBall();
        } catch (InterruptedException e) {
            e.printStackTrace();
        }
        //run intake for two seconds to load balls
        double curr_time = System.currentTimeMillis();
        while(curr_time<2000){
            intakeBall();
        }
        stopIntake();
        stopFly();
    }

    public int getLEnc(){
        return leftFly.getCurrentPosition();
    }

    public int getREnc(){
        return rightFly.getCurrentPosition();
    }

    public void resetEncoders(){
        leftFly.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        rightFly.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        leftFly.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        rightFly.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
    }

    public double getDongerLordPos(){
        return dongerLord.getPosition();
    }
}
