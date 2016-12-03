package org.firstinspires.ftc.teamcode.vision;

import android.app.Activity;
import android.view.SurfaceView;

import org.firstinspires.ftc.teamcode.opmodes.TelemetryHolder;
import org.opencv.android.CameraBridgeViewBase.CvCameraViewFrame;
import org.opencv.android.JavaCameraView;
import org.opencv.core.Core;
import org.opencv.core.Mat;
import org.opencv.android.CameraBridgeViewBase.CvCameraViewListener2;
import org.opencv.core.MatOfPoint;
import org.opencv.core.MatOfPoint2f;
import org.opencv.core.Point;
import org.opencv.core.RotatedRect;
import org.opencv.core.Scalar;
import org.opencv.imgproc.Imgproc;

import java.util.ArrayList;
import java.util.List;

//TODO: Find a way to calculate translational movement needed to center onto line
//TODO: See if there is a way to rotate the view of the JavaCamera
//TODO: Add Dropdown Menu for viewing different filters(HSV, Mask, etc.)

public class VisionProcessor {
    private Scalar mLowerBound = new Scalar(0, 0, 200);
    private Scalar mUpperBound = new Scalar(255, 80, 255);

    private Mat mHsvMat, mMask, mDilatedMask, mHierarchy;

    private List<MatOfPoint> contours = new ArrayList<>();
    private List<MatOfPoint> mContours = new ArrayList<>();

    RotatedRect rectCont;
    private Point[] vertices = new Point[4];

    int contourIndex = -1;
    double maxArea = 0, area;
    double lineLength, maxLength, deltaX, deltaY, longestX, longestY, lineIndex;

    public void init() {
        mHsvMat = new Mat();
        mMask = new Mat();
        mDilatedMask = new Mat();
        mHierarchy = new Mat();
    }

    public Mat process (CvCameraViewFrame frame){
        Mat rgbaFrame = frame.rgba();
        //TODO: Should size down frame for faster processing...
        //TODO: Need to figure out how to scale up contours and lines
        /*
        Imgproc.pyrDown(rgbaFrame, mPyrDownMat);
        Imgproc.pyrDown(mPyrDownMat, mPyrDownMat);
        Imgproc.cvtColor(mPyrDownMat, mHsvMat, Imgproc.COLOR_RGB2HSV_FULL);
        */

        //Convert to HSV
        Imgproc.cvtColor(rgbaFrame, mHsvMat, Imgproc.COLOR_RGB2HSV_FULL);

        //Filter by HSV Values
        Core.inRange(mHsvMat, mLowerBound, mUpperBound, mMask);
        //TODO: Figure out what this
        Imgproc.dilate(mMask, mDilatedMask, new Mat());

        //clear variables for search
        maxArea = 0;
        contours.clear();
        mContours.clear();
        contourIndex = -1;

        //find Contours
        Imgproc.findContours(mDilatedMask, contours, mHierarchy, Imgproc.RETR_EXTERNAL, Imgproc.CHAIN_APPROX_SIMPLE);

        //LOOP THROUGH ALL CONTOURS
        //filter out small contours & record largest
        for (int i = 0; i < contours.size(); i++){
            area = Imgproc.contourArea(contours.get(i));
            if (area > 400){
                if (area > maxArea) {
                    maxArea = area;
                    contourIndex = i;
                }
                mContours.add(contours.get(i));
            }
        }

        //check if an acceptable contour was found
        if (contourIndex > -1) {
            TelemetryHolder.telemetry.addData("angle", findAngle(vertices));
            rectCont = Imgproc.minAreaRect(new MatOfPoint2f(contours.get(contourIndex).toArray()));

            rectCont.points(vertices);
        }
        return rgbaFrame;
    }

    public double findAngle(Point [] corners){
        maxLength = 0;
        for (int i = 0; i < 4; i++){
            //TODO: reimplement after completing getLineLength
            deltaX = (corners[i].x - corners[(i+1) % 4].x);
            deltaY = (corners[i].y - corners[(i+1) % 4].y);
            lineLength = Math.sqrt((deltaX*deltaX) + (deltaY*deltaY));
            if (lineLength > maxLength){ //goes by longest line

                //if (getLineLength(corners[i],corners[(i+1) % 4] ) > maxLength){ //goes by longest line
                maxLength = lineLength;
                //maxLength = getLineLength(corners[i],corners[(i+1) % 4]);
                longestX = deltaX;
                longestY = deltaY;
                lineIndex = i;
            }
        }
        return  Math.atan(longestX/longestY)*180/Math.PI;
    }
    //TODO: temp method for printing out line length
    // Go back to commented in findAngle to continue calculating angle value
    public double getLineLength (Point pt1, Point pt2){
        deltaX = Math.abs(pt1.x - pt2.x);
        deltaY = Math.abs(pt1.y - pt2.y);
        lineLength = Math.sqrt((deltaX*deltaX) + (deltaY*deltaY));
        return lineLength;
    }


}