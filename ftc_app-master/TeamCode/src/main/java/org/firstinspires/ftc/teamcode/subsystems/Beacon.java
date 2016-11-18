package org.firstinspires.ftc.teamcode.subsystems;

import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.Servo;

import org.firstinspires.ftc.robotcore.external.Telemetry;
import org.firstinspires.ftc.teamcode.base.SensorBase;
import org.firstinspires.ftc.teamcode.base.Subsystem;
import org.firstinspires.ftc.teamcode.opmodes.AutoBlueBeacons;

/**
 * Created by Team 6696 on 11/11/2016.
 */
public class Beacon extends Subsystem{
    private Servo leftDonger, rightDonger;
    private double leftPos = 0;
    private double rightPos = 1;
    //singleton
    private static Beacon beacon = new Beacon();
    private static Telemetry telemetry;

    private Beacon() {

    }

    public void init(HardwareMap hardwareMap) {
        telemetry = AutoBlueBeacons.telemetryPass;
        leftDonger = hardwareMap.servo.get("leftDonger");
        rightDonger = hardwareMap.servo.get("rightDonger");
    }

    public void initPos() {
        setLeftNeutralPos();
        setRightNeutralPos();
    }

    public void setLeftPos(double pos) {
        leftDonger.setPosition(pos);
    }

    public void setRightPos(double pos) {
        rightDonger.setPosition(pos);
    }

    public void incLeft(boolean left, boolean right) {
        if (left)
            leftPos += 0.03;
        else if (right)
            leftPos -= 0.03;
        else leftPos = leftDonger.getPosition();
        leftPos = Math.min(Math.max(leftPos,0),1);
        leftDonger.setPosition(leftPos);
    }

    public void incRight(boolean left, boolean right){
        if (left)
            rightPos += 0.03;
        else if (right)
            rightPos -= 0.03;
        else rightPos = rightDonger.getPosition();
        rightPos = Math.min(Math.max(rightPos,0),1);
        rightDonger.setPosition(rightPos);
    }

    public double getLeftPos() {
        return leftDonger.getPosition();
    }

    public double getRightPos() {
        return rightDonger.getPosition();
    }

    public void setLeftBangPos() {
        leftDonger.setPosition(0.72);
    }

    public void setRightBangPos() {
        rightDonger.setPosition(0.45);
    }

    public void setLeftNeutralPos() {
        leftDonger.setPosition(0);
    }

    public void setRightNeutralPos() {
        rightDonger.setPosition(0);
    }

    public static Beacon getBeacon() {
        return beacon;
    }

    public void setServoPosition() {
        telemetry.addData("blue", sensorBase.isBlue());
        telemetry.addData("blueval", sensorBase.getBlue());
        telemetry.update();
        if (sensorBase.isBlue()) {
            beacon.setRightNeutralPos();
            beacon.setLeftBangPos();
        } else {
            beacon.setRightBangPos();
            beacon.setLeftNeutralPos();
        }
    }
}
