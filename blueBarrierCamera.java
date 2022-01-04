package org.firstinspires.ftc.teamcode.drive.opmode;

import com.acmerobotics.roadrunner.geometry.Pose2d;
import com.acmerobotics.roadrunner.geometry.Vector2d;
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

@Autonomous(group = "drive")

public class blueBarrierCamera extends LinearOpMode {
    private DcMotorEx LSlides, Wheel;

    private Servo Bin;
    public ElapsedTime wheelRun = new ElapsedTime(0);


    private List<DcMotorEx> motors;

    @Override
    public void runOpMode() throws InterruptedException {

        LSlides = hardwareMap.get(DcMotorEx.class, "LSlides");
        Bin = hardwareMap.get(Servo.class, "Bin");
        Wheel = hardwareMap.get(DcMotorEx.class, "Wheel");

//Bin start position - 0.4 is too low and cause problems coming back in, 0.5 cause issues intaking sometimes
        Bin.setPosition(0.5);

        SampleMecanumDrive drive = new SampleMecanumDrive(hardwareMap);
        Pose2d myPose = new Pose2d(10, 62, Math.toRadians(90));
        drive.setPoseEstimate(myPose);
        Pose2d parkT = new Pose2d(10, 62, Math.toRadians(0));

        Trajectory fondue = drive.trajectoryBuilder(myPose)
                //.back(20)
                .lineToSplineHeading(new Pose2d(6, 50, Math.toRadians(40)))
                .build();
        Trajectory park = drive.trajectoryBuilder(parkT)
                //.back(20)
                .lineToConstantHeading(new Vector2d(100, 54))
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


        //red duck side 150,350,680,880,1380,1580,600,90 0
        // red barrier side 440, 640, 1080, 1280, 1680, 1880, 650, 950
        // blue barrier side 180, 380, 780, 980, 1430, 1630, 650, 950
        //blue duck side 430, 630, 1080, 1280, 1630, 1830, 600, 900

        ConceptCV.configureRects(180, 380, 780, 980, 1430, 1630, 650, 950);

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
                LSlides.setTargetPosition(1150);
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
                LSlides.setTargetPosition(2650);
                LSlides.setMode(DcMotor.RunMode.RUN_TO_POSITION);
                while (LSlides.isBusy()) {
                    telemetry.addData("Curent pos: ", LSlides.getCurrentPosition());
                    telemetry.update();

                    idle();

                }

            } else if (location == 2) {
                LSlides.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
                LSlides.setPower(-0.8);
                LSlides.setTargetPosition(3000);
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
            sleep(1000);
            Bin.setPosition(1.0);
            sleep(1000);
            Bin.setPosition(0.5);
            sleep(1000);
            LSlides.setPower(0.8);
            LSlides.setTargetPosition(0);
            LSlides.setMode(DcMotor.RunMode.RUN_TO_POSITION);
            while (LSlides.isBusy()) {
                telemetry.addData("Curent pos: ", LSlides.getCurrentPosition());
                telemetry.update();

                idle();
            }

            //drive.followTrajectory(fondue);
            //drive.turn(Math.toRadians(0));
        // drive.followTrajectory(park);

        }

    }