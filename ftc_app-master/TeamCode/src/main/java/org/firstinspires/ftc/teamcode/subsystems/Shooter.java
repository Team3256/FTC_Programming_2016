package org.firstinspires.ftc.teamcode.subsystems;

import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.Servo;

import org.firstinspires.ftc.teamcode.base.Constants;
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

    public void runFly(double power){
        leftFly.setPower(power);
        rightFly.setPower(power);
    }

    public void stopFly(){
        leftFly.setPower(0);
        rightFly.setPower(0);
    }

    public double getPower(){
        return (leftFly.getPower()+rightFly.getPower())/2;
    }
    public void setFlyRunMode(DcMotor.RunMode runMode){
        leftFly.setMode(runMode);
        rightFly.setMode(runMode);
    }

    //only use in auto
    public void autoShootSequence(){
        setFlyRunMode(DcMotor.RunMode.RUN_USING_ENCODER);
        //start flywheel
        runFly(Constants.CLOSE_SHOT_PID_POWER);
        //run intake for two seconds to load balls
        double prev_time = System.currentTimeMillis();
        //run elevator for 5 seconds to load balls
        while(System.currentTimeMillis()-prev_time<5000){
            intakeBall();
        }
        //stop intake and flywheel
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

    double end_time = 0, start_time = 0;
    double prev_ticks = 0, curr_ticks = 0;

    public double getRPM(){
        end_time = System.currentTimeMillis();
        curr_ticks = (getLEnc()+getREnc())/2;
        double dt = end_time-start_time;
        double dticks = curr_ticks-prev_ticks;
        double RPM = 0;
        if (dt>100){
            RPM = Constants.TICKS_PER_ROTATION*dticks/dt;
        }
        start_time = System.currentTimeMillis();
        prev_ticks = curr_ticks;
        return RPM;
    }
}
