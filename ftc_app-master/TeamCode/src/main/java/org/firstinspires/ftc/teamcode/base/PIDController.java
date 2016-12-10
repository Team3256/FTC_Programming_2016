package org.firstinspires.ftc.teamcode.base;

/**
 * Created by Team 2891 on 12/9/2016.
 */
public class PIDController {
    double P, I, D, kP, kI, kD, error, sumError, changeError, prevError, PID;

    public PIDController(double kP, double kI, double kD){
        this.kP = kP;
        this.kI = kI;
        this.kD = kD;
    }

    public void resetPID(){
        error=0;
        sumError=0;
        changeError=0;
        prevError=0;
    }

    public double getError(double current, double setpoint){
        return Math.abs(setpoint - current);
    }

    public double calculatePID(double current, double setpoint) {
        System.out.println("error: " + error + " P: " + P + " kP: " + kP);
        error = setpoint - current;
        sumError = sumError + error;
        changeError = (error-prevError);
        P = error;
        I = sumError;
        D = changeError;
        PID = (kP *P) + (I * kI) + (D * kD);
        prevError = error;
        if (PID > 0.7)
            PID = 0.7;
        if (PID < -0.7)
            PID = -0.7;
        return PID;
    }

}
