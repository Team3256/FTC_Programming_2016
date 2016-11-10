package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.hardware.CRServo;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.Servo;

/**
 * Created by Team 2891 on 10/31/2016.
 */
public class Beacon {

    private HardwareMap hm;
    private Servo leftDonger, rightDonger;
    private double leftPos = 0;
    private double rightPos = 1;
    CRServo test;

    public void init_Beacon(HardwareMap hm){
        this.hm = hm;
        leftDonger = hm.servo.get(  "leftDonger");
        rightDonger = hm.servo.get("rightDonger");
    }

    public void initPos(){
        setLeftNeutralPos();
        setRightNeutralPos();
    }

    public void setLeftPos(double pos){
        leftDonger.setPosition(pos);
    }

    public void setRightPos(double pos){
        rightDonger.setPosition(pos);
    }

    public void incLeft(boolean left, boolean right){
        if (left){
            leftPos+=0.03;
        }
        else if (right){
            leftPos-=0.03;
        }
        else leftPos = leftDonger.getPosition();
        leftPos = Math.min(Math.max(leftPos,0),1);
        leftDonger.setPosition(leftPos);
    }

    public void incRight(boolean left, boolean right){
        if (left){
            rightPos+=0.03;
        }
        else if (right){
            rightPos-=0.03;
        }
        else rightPos = rightDonger.getPosition();
        rightPos = Math.min(Math.max(rightPos,0),1);
        rightDonger.setPosition(rightPos);

    }

    public double getLeftPos(){
        return leftDonger.getPosition();
    }

    public double getRightPos(){
        return rightDonger.getPosition();
    }

    public void setLeftBangPos(){
        leftDonger.setPosition(0.72);
    }

    public void setRightBangPos(){
        rightDonger.setPosition(.45);
    }

    public void setLeftNeutralPos(){
        leftDonger.setPosition(0);
    }

    public void setRightNeutralPos(){
        rightDonger.setPosition(0);
    }
}
