package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import org.firstinspires.ftc.robotcore.external.hardware.camera.WebcamName;
import org.openftc.easyopencv.OpenCvCamera;
import org.openftc.easyopencv.OpenCvCameraFactory;
import org.openftc.easyopencv.OpenCvCameraRotation;
import org.openftc.easyopencv.OpenCvWebcam;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;

import java.io.IOException;

@Autonomous(name="CVtester",group="Linear OpMode")
public class RunCamera extends LinearOpMode {


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

        waitForStart();


        while(opModeIsActive()) {

            //again start copy here
            int location = 0;

            location = ConceptCV.findTSE(ConceptCV.returnInput(),236,312,778,954,1416,1571,831,881);
            //location of 0 = left, 1 = middle, 2 is right



            telemetry.addData("Location: ",location);
            telemetry.update();
            //end here


            finalWebcam.stopStreaming(); //this needs to be end of file btw


        }



    }
}
