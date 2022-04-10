package org.firstinspires.ftc.teamcode.drive.opmode;

import org.opencv.core.Core;
import org.opencv.core.Mat;
import org.opencv.core.MatOfPoint;
import org.opencv.core.MatOfPoint2f;
import org.opencv.core.Point;
import org.opencv.core.Rect;
import org.opencv.core.Scalar;
import org.opencv.core.Size;
import org.opencv.imgproc.Imgproc;
import org.openftc.easyopencv.OpenCvPipeline;

import java.util.ArrayList;
import java.util.List;

public class contourPipeline extends OpenCvPipeline {
    int midpoint;

    @Override
    public Mat processFrame(Mat input) {
        try {
            Mat end = input;

            Mat src = input;


            //Scalar scalarLowerYCrCb = new Scalar(0.0, 0.0, 0.0);
            //Scalar scalarUpperYCrCb = new Scalar(255.0, 100.0, 170.0);


            Scalar scalarLowerYCrCb = new Scalar(30.0, 71.4, 40.0);//HSV for now
            Scalar scalarUpperYCrCb = new Scalar(80.0, 255.0, 255.0);//HSV for now

            //Converts space to ycrcb
            Imgproc.cvtColor(src, src,Imgproc.COLOR_BGR2HSV);
            //filters out all colors not in this range
            Core.inRange(src, scalarLowerYCrCb, scalarUpperYCrCb, src);
            // Remove Noise
            Imgproc.morphologyEx(src, src, Imgproc.MORPH_OPEN, new Mat());
            Imgproc.morphologyEx(src, src, Imgproc.MORPH_CLOSE, new Mat());
            // GaussianBlur
            Imgproc.blur(src, src, new Size(10, 10));


            //Finding Contours

            ArrayList<MatOfPoint> contours = new ArrayList<>();

            Mat hierarchey = new Mat();
            //finds largest contour
            Imgproc.findContours(src, contours, hierarchey, Imgproc.RETR_TREE, Imgproc.CHAIN_APPROX_SIMPLE);

            int largestIndex = 0;
            int largest = contours.get(0).toArray().length;


            for (int i = 0; i < contours.size(); i++) {
                int currentSize = contours.get(i).toArray().length;
                if (currentSize > largest) {

                    largest = currentSize;
                    largestIndex = i;
                }

            }


            //Draw rectangle on largest contours

            MatOfPoint2f areaPoints = new MatOfPoint2f(contours.get(largestIndex).toArray());

            Rect rect = Imgproc.boundingRect(areaPoints);

            Imgproc.rectangle(end, rect, new Scalar(255, 0, 0));
            ;

            midpoint = (rect.x + (rect.x + rect.width)) / 2;


            int middleBound = 600;

            int rightBound = 1300;


            Imgproc.line(end, new Point(middleBound, 1920), new Point(middleBound, 0), new Scalar(255, 0, 0));

            Imgproc.line(end, new Point(rightBound, 1920), new Point(rightBound, 0), new Scalar(255, 0, 0));

            return end;
        }catch(IndexOutOfBoundsException e){
            int one = 1;

        }

        return input;

    }


    public int getMidpoint(){
        return midpoint;
    }


    public int getLocation(){
        //change these values when needed
        if(midpoint>1300){
            return 2;
        }else if(midpoint>600){
            return 1;
        }else{
            return 0;
        }


    }
}
