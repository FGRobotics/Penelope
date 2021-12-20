package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

import java.io.ByteArrayInputStream;
import java.util.ArrayList;
import java.util.List;
import org.opencv.core.Core;
import org.opencv.core.Mat;
import org.opencv.core.MatOfByte;
import org.opencv.core.MatOfPoint;
import org.opencv.core.MatOfPoint2f;
import org.opencv.core.Point;
import org.opencv.core.Rect;
import org.opencv.core.Scalar;

import org.opencv.core.Size;
import org.opencv.imgcodecs.Imgcodecs;
import org.opencv.imgproc.Imgproc;
import org.openftc.easyopencv.OpenCvInternalCamera;
import org.openftc.easyopencv.OpenCvPipeline;
import org.openftc.easyopencv.OpenCvCamera;
import org.openftc.easyopencv.OpenCvCameraFactory;
import org.openftc.easyopencv.OpenCvCameraRotation;

import static org.opencv.imgproc.Imgproc.boundingRect;
import static org.opencv.imgproc.Imgproc.contourArea;
import static org.opencv.imgproc.Imgproc.cvtColor;
import static org.opencv.imgproc.Imgproc.rectangle;
import static org.opencv.imgproc.Imgproc.threshold;



import java.io.File;

import java.lang.Math;
import java.io.IOException;


public class ConceptCV extends OpenCvPipeline {

    public double rectX;
    public double midpoint;


    public int length;
    Scalar HOT_PINK = new Scalar(17, 140, 0);
    public static Mat stream = new Mat();
    public static Scalar lowHSv = new Scalar(0.0, 0.0, 0.0);
    public static Scalar UpHsv = new Scalar(255.0, 120.0, 120.0);
    Rect bounding_rect = new Rect();
    public static MatOfByte mob=new MatOfByte();
    @Override
    public Mat processFrame(Mat input){



        stream = input;


        return input;
    }

    public static Mat returnInput(){
        return stream;
    }

    public static int findTSE(Mat input,int x1,int x2,int x3, int x4, int x5, int x6, int y1, int y2){
        
        
        Rect leftRect = new Rect(x1,y1,x2-x1,y1-y2);
        Imgproc.rectangle(input, leftRect, new Scalar(0, 255, 0), 2);

        Rect middleRect = new Rect(x3,y1,x4-x3,y1-y2);
        Imgproc.rectangle(input, middleRect, new Scalar(0, 255, 0), 2);

        Rect rightRect = new Rect(x5,y1,x6-x5,y1-y2);
        Imgproc.rectangle(input, rightRect, new Scalar(0, 255, 0), 2);
        
        //drew rectangle for all the color searching areas

        int avg_greenLeft = 0;
        int avg_greenMiddle = 0;
        int avg_greenRight = 0;

        for(int x = x1; x<=x2; x++) {
            for(int y = y1; y<=y2; y++) {
                double[] pixel1 = input.get(y,x);

                double g1 = pixel1[1]/255.0;
                if(g1>0.65) {
                    avg_greenLeft += 1;
                }
            }
        }

        for(int x = x3; x<=x4; x++) {
            for(int y = y1; y<=y2; y++) {
                double[] pixel1 = input.get(y,x);

                double g1 = pixel1[1]/255.0;
                if(g1>0.65) {
                    avg_greenMiddle += 1;
                }
            }
        }

        for(int x = x5; x<=x6; x++) {
            for(int y = y1; y<=y2; y++) {
                double[] pixel1 = input.get(y,x);

                double g1 = pixel1[1]/255.0;
                if(g1>0.65) {
                    avg_greenRight += 1;
                }
            }
        }



        int[] valuesArray = {0,0,0};
        valuesArray[0] = avg_greenLeft;

        valuesArray[1] = avg_greenMiddle;

        valuesArray[2] = avg_greenRight;

        int maxAt = 0;

        for (int i = 0; i < valuesArray.length; i++) {
            maxAt = valuesArray[i] > valuesArray[maxAt] ? i : maxAt;
        }

        return maxAt;


    }
}

