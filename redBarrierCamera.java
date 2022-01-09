package org.firstinspires.ftc.teamcode.drive.opmode;

import com.acmerobotics.roadrunner.geometry.Pose2d;
import com.acmerobotics.roadrunner.trajectory.Trajectory;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.robotcore.external.hardware.camera.WebcamName;
import org.firstinspires.ftc.teamcode.drive.SampleMecanumDrive;
import org.openftc.easyopencv.OpenCvCamera;
import org.openftc.easyopencv.OpenCvCameraFactory;
import org.openftc.easyopencv.OpenCvCameraRotation;
import org.openftc.easyopencv.OpenCvWebcam;

import java.util.List;
@Autonomous(group = "Drive")
public class redBarrierCamera extends LinearOpMode {
    private DcMotorEx LSlides, Wheel;

    private Servo Bin;
    public ElapsedTime wheelRun = new ElapsedTime(0);


    private List<DcMotorEx> motors;

    @Override
    public void runOpMode() throws InterruptedException {

        Wheel = hardwareMap.get(DcMotorEx.class, "Wheel");
        LSlides = hardwareMap.get(DcMotorEx.class, "LSlides");
        Bin = hardwareMap.get(Servo.class, "Bin");

//Bin start position - 0.4 is too low and cause problems coming back in, 0.5 cause issues intaking sometimes
        Bin.setPosition(0.5);

        SampleMecanumDrive drive = new SampleMecanumDrive(hardwareMap);
        Pose2d myPose = new Pose2d(10, -62, Math.toRadians(270));
        drive.setPoseEstimate(myPose);
        Pose2d parkT = new Pose2d(3, -45, Math.toRadians(0));


        Trajectory fondue = drive.trajectoryBuilder(myPose)
                //.back(20)
                .lineToSplineHeading(new Pose2d(3, -46, Math.toRadians(269)))
                .build();
        Trajectory park = drive.trajectoryBuilder(myPose)
                //.back(20)
                //.lineToSplineHeading(new Pose2d(-26, -51, Math.toRadians(230)))
                .lineToSplineHeading(new Pose2d(10, -46, Math.toRadians(0)))
                .build();


        waitForStart();
        Wheel.setPower(0.5);
        sleep(500);
        Wheel.setPower(0);
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
                finalWebcam.startStreaming(1920, 1080, OpenCvCameraRotation.UPRIGHT);
                telemetry.addData("Initialization passed ", test);
                telemetry.update();
            }

            @Override
            public void onError(int errorCode) {
                telemetry.addData("Init Failed ", errorCode);
                telemetry.update();

            }
        });

        sleep(4000);

        int location = 0;


        //red duck side 150,350,680,880,1380,1580,600,99
        // red barrier side 440, 640, 1080, 1280, 1680, 1880, 650, 950

        ConceptCV.configureRects(440, 640, 1080, 1280, 1680, 1880, 650, 950);

        location = ConceptCV.findTSE();
        sleep(2000);
        location = ConceptCV.findTSE();
        telemetry.addLine("location: " + location);
        telemetry.update();


        sleep(2000);


        if (isStopRequested()) return;


        finalWebcam.closeCameraDeviceAsync(new OpenCvCamera.AsyncCameraCloseListener() {


            @Override
            public void onClose() {
                //finalWebcam.stopStreaming();
                telemetry.addLine("Closed");
                telemetry.update();
            }
        });

        // Variable setup
        int cPos = LSlides.getCurrentPosition();
        double LSlidesPower = 0.0;
        double LSlidesRotation = (LSlides.getCurrentPosition() / 1680.0);
        double upRange = 0.5;
        double downRange = 1.0;
        //Telemetry
        LSlidesRotation = LSlides.getCurrentPosition();

        telemetry.addData("Slides current rotation: ", LSlidesRotation);
        telemetry.addData("Slides tick position: ", LSlides.getCurrentPosition());
        telemetry.update();


        if (location == 0) {
            drive.followTrajectory(fondue);

            LSlides.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
            LSlides.setPower(-0.8);
            LSlides.setTargetPosition(2000); //last number was 1600
            LSlides.setMode(DcMotor.RunMode.RUN_TO_POSITION);
            while (LSlides.isBusy()) {
                telemetry.addData("Curent pos: ", LSlides.getCurrentPosition());
                telemetry.update();

                idle();

            }
        } else if (location == 1) {
            drive.followTrajectory(fondue);
            LSlides.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
            LSlides.setPower(-0.8);
            LSlides.setTargetPosition(2800);
            LSlides.setMode(DcMotor.RunMode.RUN_TO_POSITION);
            while (LSlides.isBusy()) {
                telemetry.addData("Curent pos: ", LSlides.getCurrentPosition());
                telemetry.update();

                idle();

            }

        } else if (location == 2) {
            drive.followTrajectory(fondue);
            LSlides.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
            LSlides.setPower(-0.8);
            LSlides.setTargetPosition(4100);
            LSlides.setMode(DcMotor.RunMode.RUN_TO_POSITION);
            while (LSlides.isBusy()) {
                telemetry.addData("Curent pos: ", LSlides.getCurrentPosition());
                telemetry.update();

                idle();

            }
        } else {
            Wheel.setPower(0.5);
            sleep(1000);
            Wheel.setPower(0);
        }
        if(!LSlides.isBusy()){
        sleep(1000);
        Bin.setPosition(1.0);
        sleep(1000);
        Bin.setPosition(0.5);
        sleep(1000);
        }
        /*
            //park trajectory
        //drive.followTrajectory(park);
        drive.turn(Math.toRadians(91));

         */

        /*LSlides.setPower(0.8);
        LSlides.setTargetPosition(10);
        LSlides.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        while(LSlides.isBusy()) {
            telemetry.addData("Curent pos: ", LSlides.getCurrentPosition());
            telemetry.update();

            idle();
        }

        /*if(gamepad1.dpad_up){
                LSlides.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
                LSlides.setPower(0.8);
                LSlides.setTargetPosition(3800);
                LSlides.setMode(DcMotor.RunMode.RUN_TO_POSITION);
                while(LSlides.isBusy()){
                    telemetry.addData("Curent pos: ", LSlides.getCurrentPosition());
                    telemetry.update();
                    idle();
                }
                LSlides.setPower(0.0);
                if(gamepad1.triangle)
                    Bin.setPosition(1.0);
            }
            else if(gamepad1.dpad_left){
                LSlides.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
                LSlides.setPower(0.8);
                LSlides.setTargetPosition(2533);
                LSlides.setMode(DcMotor.RunMode.RUN_TO_POSITION);
                while(LSlides.isBusy()){
                    telemetry.addData("Curent pos: ", LSlides.getCurrentPosition());
                    telemetry.update();
                    idle();
                }
                LSlides.setPower(0.0);
                if(gamepad1.triangle)
                    Bin.setPosition(1.0);
            }
            else if(gamepad1.dpad_down){
                LSlides.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
                LSlides.setPower(0.8);
                LSlides.setTargetPosition(1267);
                LSlides.setMode(DcMotor.RunMode.RUN_TO_POSITION);
                while(LSlides.isBusy()){
                    telemetry.addData("Curent pos: ", LSlides.getCurrentPosition());
                    telemetry.update();
                    idle();
                }
                LSlides.setPower(0.0);
                if(gamepad1.triangle)
                    Bin.setPosition(1.0);
            }
            else if(gamepad1.cross){
                Bin.setPosition(0.5);
                LSlides.setTargetPosition(950);
                LSlides.setPower(-0.7);
                while(LSlides.isBusy()){
                    telemetry.addData("Curent pos: ", LSlides.getCurrentPosition());
                    telemetry.update();
                }
            }
            else{
                Bin.setPosition(0.5);
            }*/


    }
}
