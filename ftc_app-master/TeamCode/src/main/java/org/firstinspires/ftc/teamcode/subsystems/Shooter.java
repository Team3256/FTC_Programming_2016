package org.firstinspires.ftc.teamcode.subsystems;

import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.HardwareMap;

import org.firstinspires.ftc.teamcode.base.Subsystem;

/**
 * Created by Eric on 11/15/2016.
 */
public class Shooter extends Subsystem {
    private static Shooter shooter = new Shooter();

    private Shooter(){

    }

    public static Shooter getInstance(){
        return shooter;
    }

    private DcMotor leftFly, rightFly;
    private DcMotor intake;

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
        //determine ticks per sec emperially
        leftFly.setMaxSpeed(4000);
        rightFly.setMaxSpeed(4000);
        leftFly.setPower(0);
        rightFly.setPower(0);

        intake = hardwareMap.dcMotor.get("intake");
        intake.setDirection(DcMotor.Direction.FORWARD);
        intake.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.FLOAT);
        intake.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        intake.setPower(0);
    }

    public void intakeBall() {
        intake.setPower(0.5);
    }

    public void outputBall() {
        intake.setPower(-0.5);
    }
}
