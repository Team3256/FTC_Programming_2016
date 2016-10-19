package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.HardwareMap;

/**
 * Created by Eric on 9/16/2016.
 */
public class Intake {

    // Hardware Map to store all our electrical component objects
    private HardwareMap hm = null;

    //Drive Motors
    private DcMotor intake;

    /**
     * Intake()
     * Empty Constructor
     */
    public Intake() {

    }

    /**
     * init_Intake()
     * Initializes the whole intake, including all the motors
     *
     * @param hm    Instance of the HardwareMap of the Robot
     */
    public void init_Intake(HardwareMap hm) {
        //initialize the hardware map
        this.hm = hm;

        //initialize the motors
        intake = this.hm.dcMotor.get("intake");

        //set motor directions
        intake.setDirection(DcMotor.Direction.FORWARD);

        //set motor mode
        intake.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);

        //initialize all motors to 0 power
        intake.setPower(0);
    }

    /**
     * runLeft()
     * Runs the intake motor
     *
     * @param speed speed to run the motor
     */
    public void runIntake(boolean enabled, double speed) {
        if (enabled) intake.setPower(speed);
        else intake.setPower(0);
    }
}