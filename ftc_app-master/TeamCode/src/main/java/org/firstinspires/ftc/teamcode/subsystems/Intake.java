package org.firstinspires.ftc.teamcode.subsystems;

import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.HardwareMap;

/**
 * Created by Team 6696 on 11/11/2016.
 */
public class Intake {
    private HardwareMap hardwareMap;

    private DcMotor intakeMotor;
    //singleton
    private static Intake intake;

    private Intake() {

    }

    public void initIntake(HardwareMap hardwareMap) {
        this.hardwareMap = hardwareMap;
        intakeMotor = this.hardwareMap.dcMotor.get("intake");
        intakeMotor.setDirection(DcMotor.Direction.FORWARD);
        intakeMotor.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.FLOAT);
        intakeMotor.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        intakeMotor.setPower(0);
    }

    public void runIntake(boolean enabled, double speed) {
        intakeMotor.setPower(enabled ? speed : 0);
    }

    public static Intake getIntake() {
        return intake;
    }
}
