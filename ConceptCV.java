package org.firstinspires.ftc.teamcode.drive.opmode;

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

@Disabled
public class ConceptCV extends OpenCvPipeline {

    public double rectX;
    public double midpoint;
    public static int x1,x2,x3,x4,x5,x6,y1,y2;

    public int length;
    Scalar HOT_PINK = new Scalar(17, 140, 0);
    public static Mat stream = new Mat();
    public static Scalar lowHSv = new Scalar(0.0, 0.0, 0.0);
    public static Scalar UpHsv = new Scalar(255.0, 120.0, 120.0);
    Rect bounding_rect = new Rect();
    public static MatOfByte mob=new MatOfByte();
    public static int maxAt = 2;
    public static void configureRects(int z1,int z2,int z3, int z4, int z5, int z6, int p1, int p2){
        x1 = z1;
        x2 = z2;
        x3 = z3;
        x4 = z4;
        x5 = z5;
        x6 = z6;
        y1 = p1;
        y2 = p2;
    }
    public static void translateRects(int leftX, int middleX, int rightX, int y){
        x1 += leftX;
        x2 += leftX;

        x3 += middleX;
        x4 += middleX;

        x5 += rightX;
        x6 += rightX;

        y1 += y;
        y2 += y;


    }
    @Override
    public Mat processFrame(Mat input){

        stream = input;






        int avg_greenLeft = 0;
        int avg_greenMiddle = 0;
        int avg_greenRight = 0;

        for(int x = x1; x<=x2; x++) {
            for(int y = y1; y<=y2; y++) {
                double[] pixel1 = stream.get(y,x);
                double r1 = pixel1[2]/255.0;
                double b1 = pixel1[0]/255.0;
                double g1 = pixel1[1]/255.0;

                if(g1>0.3 && b1<0.5 /*&& r1 <0.5*/) {
                    avg_greenLeft += 1;
                }
            }
        }

        for(int x = x3; x<=x4; x++) {
            for(int y = y1; y<=y2; y++) {
                double[] pixel1 = stream.get(y,x);
                double r1 = pixel1[2]/255.0;
                double b1 = pixel1[0]/255.0;
                double g1 = pixel1[1]/255.0;

                if(g1>0.3 && b1<0.5 /*&& r1 <0.5*/) {
                    avg_greenMiddle += 1;
                }
            }
        }

        for(int x = x5; x<=x6; x++) {
            for(int y = y1; y<=y2; y++) {
                double[] pixel1 = stream.get(y,x);
                double r1 = pixel1[2]/255.0;
                double b1 = pixel1[0]/255.0;
                double g1 = pixel1[1]/255.0;

                if(g1>0.3 && b1<0.5 /*&& r1 <0.5*/) {
                    avg_greenRight += 1;
                }
            }
        }



        int[] valuesArray = {0,0,0};
        valuesArray[0] = avg_greenLeft;

        valuesArray[1] = avg_greenMiddle;

        valuesArray[2] = avg_greenRight;



        for (int i = 0; i < valuesArray.length; i++) {
            maxAt = valuesArray[i] > valuesArray[maxAt] ? i : maxAt;
        }

        if(maxAt == 0){
            Imgproc.rectangle(input,new Point(x1 , y1),new Point(x2,y2),new Scalar(0,255,0),3);

            Imgproc.rectangle(input,new Point(x3 , y1),new Point(x4,y2),new Scalar(0,0,255),3);

            Imgproc.rectangle(input,new Point(x5 , y1),new Point(x6,y2),new Scalar(0,0,255),3);
        }
        if(maxAt == 1){
            Imgproc.rectangle(input,new Point(x1 , y1),new Point(x2,y2),new Scalar(0,0,255),3);

            Imgproc.rectangle(input,new Point(x3 , y1),new Point(x4,y2),new Scalar(0,255,0),3);

            Imgproc.rectangle(input,new Point(x5 , y1),new Point(x6,y2),new Scalar(0,0,255),3);
        }
        if(maxAt == 2){
            Imgproc.rectangle(input,new Point(x1 , y1),new Point(x2,y2),new Scalar(0,0,255),3);

            Imgproc.rectangle(input,new Point(x3 , y1),new Point(x4,y2),new Scalar(0,0,255),3);

            Imgproc.rectangle(input,new Point(x5 , y1),new Point(x6,y2),new Scalar(0,255,0),3);
        }





        return input;
    }

    public static Mat returnInput(){
        return stream;
    }

    public static int findTSE(){



        return maxAt;


    }
}