package org.firstinspires.ftc.teamcode;

import org.opencv.core.Core;
import org.opencv.core.CvType;
import org.opencv.core.Mat;
import org.opencv.core.MatOfPoint;
import org.opencv.core.Point;
import org.opencv.core.Rect;
import org.opencv.core.Scalar;
import org.opencv.core.Size;
import org.opencv.imgproc.Imgproc;
import org.openftc.easyopencv.OpenCvPipeline;

import java.util.ArrayList;

public class ShippingElementPipeline extends OpenCvPipeline {
    ArrayList<MatOfPoint> contours = new ArrayList<>();
    Point point;
    int contourIndex;

    @Override
    public Mat processFrame(Mat input) {
        Mat subMat = new Mat();
        Imgproc.cvtColor(input,subMat,Imgproc.COLOR_RGB2HSV);
        Imgproc.GaussianBlur(subMat,subMat,new Size(5,5),0);
        Scalar lower = new Scalar(7,0,0);
        Scalar upper = new Scalar(10,255,255);
        Core.inRange(subMat,lower,upper,subMat);
        Imgproc.morphologyEx(subMat,subMat,Imgproc.MORPH_OPEN,Mat.ones(new Size(3,3), CvType.CV_32F));
        contours.clear();
        MatOfPoint contour = null;
        Imgproc.findContours(subMat,contours, new Mat(), Imgproc.RETR_LIST,Imgproc.CHAIN_APPROX_TC89_KCOS);
        for (int i = 0; i < contours.size(); i++) {
            MatOfPoint newContour = contours.get(i);
            if(Imgproc.boundingRect(newContour).width >10 && Imgproc.boundingRect(newContour).height > 10){
                if(contour == null){
                    contour = newContour;
                    contourIndex=i;
                }else if(Imgproc.contourArea(newContour)> Imgproc.contourArea(contour)){
                    contour = newContour;
                    contourIndex=i;
                }
            }
        }
        Rect rect = null;
        try {
            rect = Imgproc.boundingRect(contours.get(contourIndex));
            Imgproc.drawContours(input,contours,contourIndex,new Scalar(0,255,255));
            if(rect.area()<200){
                rect = null;
            }
        }catch(Exception ignored){}
        if(rect !=null) {
            Imgproc.rectangle(input, rect, new Scalar(255, 0, 0));
            point = new Point(rect.x + rect.width / 2.0, rect.y + rect.height / 2.0);
        }else {
            point = new Point(0, 0);
        }
        subMat.release();
        return input;
    }

    public Point getPoint() {
        return point;
    }
}