package org.firstinspires.ftc.teamcode.subsystems;

import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.HardwareMap;

import org.firstinspires.ftc.teamcode.base.Subsystem;

/**
 * Created by Eric on 11/15/2016.
 */
public class FlyWheel extends Subsystem {
    private static FlyWheel flyWheel= new FlyWheel();

    private FlyWheel(){

    }

    public static FlyWheel getInstance(){
        return flyWheel;
    }

    private DcMotor leftFly, rightFly;

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
    }
}
