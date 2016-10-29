package org.firstinspires.ftc.teamcode;

import java.util.Timer;
import java.util.TimerTask;

/**
 * Created by Team 2891 on 10/28/2016.
 */
public class PIDController {
    double kP, kI, kD;
    double error, sumError, changeError, prevError;
    double maxOutput, minOutput;

    public PIDController(double kP, double kI, double kD) {
        this.kP = kP;
        this.kI = kI;
        this.kD = kD;
    }

    public void reset() {
        this.error = 0;
        this.sumError = 0;
        this.changeError = 0;
        this.prevError = 0;
    }

    public void setMaxOutput(double maxOutput) {
        this.maxOutput = maxOutput;
    }

    public void setMinOutput(double minOutput) {
        this.minOutput = minOutput;
    }

    public double calculatePID(double current, double setpoint) {
        error = setpoint - current;
        sumError += error;
        changeError = error - prevError;
        double p = kP * error;
        double i = kI * sumError;
        double d = kD * changeError;
        prevError = error;
        return Math.max(minOutput, Math.min(maxOutput, p + i + d));
    }
}
