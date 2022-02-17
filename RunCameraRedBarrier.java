package org.firstinspires.ftc.teamcode.drive.opmode;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;

import org.firstinspires.ftc.robotcore.external.hardware.camera.WebcamName;
import org.openftc.easyopencv.OpenCvCamera;
import org.openftc.easyopencv.OpenCvCameraFactory;
import org.openftc.easyopencv.OpenCvCameraRotation;
import org.openftc.easyopencv.OpenCvWebcam;

@Autonomous(name="CVtesterRedBarrier",group="Linear OpMode")

public class RunCameraRedBarrier extends LinearOpMode {


    @Override
    public void runOpMode() throws InterruptedException {
        //copy starting from here
        OpenCvWebcam webcam;
        int test = 1;
        int cameraMonitorViewId = hardwareMap.appContext.getResources().getIdentifier("cameraMonitorViewId", "id", hardwareMap.appContext.getPackageName());
        webcam = OpenCvCameraFactory.getInstance().createWebcam(hardwareMap.get(WebcamName.class, "Webcam"), cameraMonitorViewId);
        //OpenCV Pipeline

        ConceptCV myPipeline = new ConceptCV();
        webcam.setPipeline(myPipeline);

        OpenCvWebcam finalWebcam = webcam;
        finalWebcam.openCameraDeviceAsync(new OpenCvCamera.AsyncCameraOpenListener() {
            @Override
            public void onOpened() {
                finalWebcam.startStreaming(1920 ,1080, OpenCvCameraRotation.UPRIGHT);
                telemetry.addData("Initialization passed ", test);
                telemetry.update();
            }

            @Override
            public void onError(int errorCode) {
                telemetry.addData("Init Failed ", errorCode);
                telemetry.update();

            }
            //stop copying here
        });
        int location = 0;

        //red duck side 150,350,680,880,1380,1580,600,99
        // red barrier side 440, 640, 1080, 1280, 1680, 1880, 650, 950
        //ConceptCV.configureRects(150,350,680,880,1380,1580,600,99);
        ConceptCV.configureRects(440, 640, 1080, 1280, 1680, 1880, 650, 950);

        ConceptCV.translateRects(0,0,0,0);
        location = ConceptCV.findTSE();
        //location of 0 = left, 1 = middle, 2 is right


        waitForStart();


        while(opModeIsActive()) {

            //again start copy here


            telemetry.addData("Location: ",location);
            telemetry.update();
            //end here


            finalWebcam.stopStreaming(); //this needs to be end of file btw


        }



    }
}
