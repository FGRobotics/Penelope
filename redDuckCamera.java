package org.firstinspires.ftc.teamcode.drive.opmode;

import com.acmerobotics.roadrunner.geometry.Pose2d;
import com.acmerobotics.roadrunner.trajectory.Trajectory;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.DistanceSensor;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.robotcore.external.hardware.camera.WebcamName;
import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit;
import org.openftc.easyopencv.OpenCvCamera;
import org.openftc.easyopencv.OpenCvCameraFactory;
import org.openftc.easyopencv.OpenCvCameraRotation;
import org.openftc.easyopencv.OpenCvWebcam;

import java.util.List;
@Autonomous (group = "Drive")
public class redDuckCamera extends LinearOpMode {
    private DcMotorEx LSlides, Wheel, LEDs;
    DistanceSensor distance;
    private Servo Bin;
    public ElapsedTime wheelRun = new ElapsedTime(0);


    private List<DcMotorEx> motors;

    @Override
    public void runOpMode() throws InterruptedException {

        Wheel = hardwareMap.get(DcMotorEx.class, "Wheel");
        Wheel.setDirection(DcMotorSimple.Direction.REVERSE);
        LSlides = hardwareMap.get(DcMotorEx.class, "LSlides");
        LSlides.setDirection(DcMotorSimple.Direction.REVERSE);
        Bin = hardwareMap.get(Servo.class, "Bin");
        distance = hardwareMap.get(DistanceSensor.class, "toaster");
        distance.getDistance(DistanceUnit.INCH);
        LEDs = hardwareMap.get(DcMotorEx.class, "LEDs");

        //Bin start position - 0.4 is too low and cause problems coming back in, 0.5 cause issues intaking sometimes
        Bin.setPosition(0.5);
        LEDs.setDirection(DcMotorSimple.Direction.REVERSE);

        SampleMecanumDrive drive = new SampleMecanumDrive(hardwareMap);
        Pose2d myPose = new Pose2d(-30, -62, Math.toRadians(270));//intake facing back wall
        drive.setPoseEstimate(myPose);

        double difference = 0;
        double turn = 0;
        int location = 2;





        //Trajectories
        Trajectory fondue = drive.trajectoryBuilder(myPose)
                //.back(20)
                //.lineTo(new Vector2d(-20,-45))
                .lineToSplineHeading(new Pose2d(-7, -43, Math.toRadians(260))) //267
                //SampleMecanumDrive.getVelocityConstraint(25, DriveConstants.MAX_ANG_VEL, DriveConstants.TRACK_WIDTH),
                //SampleMecanumDrive.getAccelerationConstraint(DriveConstants.MAX_ACCEL))
                .build();


        Trajectory duck = drive.trajectoryBuilder(fondue.end())

                .lineToSplineHeading(new Pose2d(-54, -58, Math.toRadians(260))) //add more tilt
                //SampleMecanumDrive.getVelocityConstraint(20, DriveConstants.MAX_ANG_VEL, DriveConstants.TRACK_WIDTH),
                //SampleMecanumDrive.getAccelerationConstraint(22)) // x was -55 : y was -50

                .build();
        Trajectory park = drive.trajectoryBuilder(duck.end())
                .lineToSplineHeading(new Pose2d(-55, -31, Math.toRadians(269)))
                .build();
        /*Trajectory left = drive.trajectoryBuilder(myPose)
                .strafeLeft(12)
                .build();
        Trajectory up = drive.trajectoryBuilder(left.end())
                .back(14.5)
                .build();*/


        OpenCvWebcam webcam;
        int test = 1;
        int cameraMonitorViewId = hardwareMap.appContext.getResources().getIdentifier("cameraMonitorViewId", "id", hardwareMap.appContext.getPackageName());
        webcam = OpenCvCameraFactory.getInstance().createWebcam(hardwareMap.get(WebcamName.class, "Webcam"), cameraMonitorViewId);
        //OpenCV Pipeline

        contourPipeline myPipeline = new contourPipeline();
        webcam.setPipeline(myPipeline);

        OpenCvWebcam finalWebcam = webcam;

        finalWebcam.openCameraDeviceAsync(new OpenCvCamera.AsyncCameraOpenListener() {
            @Override
            public void onOpened() {
                finalWebcam.startStreaming(1920, 1080, OpenCvCameraRotation.UPSIDE_DOWN);
                telemetry.addData("Initialization passed ", test);

                telemetry.update();
            }

            @Override
            public void onError(int errorCode) {
                telemetry.addData("Init Failed ", errorCode);
                telemetry.update();

            }
        });

        while(!opModeIsActive()){

            //int testRun = 1;
            //int finalTestRun = testRun;
            //telemetry.addData("Pass number ", finalTestRun);
            telemetry.update();
            //red duck side 150,350,680,880,1380,1580,600,99
            // red barrier side 440, 640, 1080, 1280, 1680, 1880, 650, 950


            location = myPipeline.getLocation();
            telemetry.addLine("location: " + location);
            telemetry.update();
            //testRun++;

        }
        waitForStart();
        if (isStopRequested()) return;
        location = myPipeline.getLocation();
        telemetry.addLine("location: " + location);
        telemetry.update();

        finalWebcam.closeCameraDeviceAsync(new OpenCvCamera.AsyncCameraCloseListener() {


            @Override
            public void onClose() {
                //finalWebcam.stopStreaming();
                telemetry.addLine("Closed");
                telemetry.update();
            }
        });


        // Variable setup
        int targetPos = 0;
        int cPos = LSlides.getCurrentPosition();
        double LSlidesPower = 0.0;
        double LSlidesRotation = (LSlides.getCurrentPosition() / 1680.0);
        double upRange = 0.5;
        double downRange = 1.0;
        //Telemetry
        LSlidesRotation = LSlides.getCurrentPosition();

        telemetry.addLine("location: " + location);
        telemetry.update();

        telemetry.addData("Slides current rotation: ", LSlidesRotation);
        telemetry.addData("Slides tick position: ", LSlides.getCurrentPosition());
        telemetry.update();


        if (location == 0) {
            LEDs.setPower(.5);
            sleep(200);
            LEDs.setPower(0);


            drive.followTrajectory(fondue);
            if (distance.getDistance(DistanceUnit.INCH) < 30) {
                targetPos = 190 * (int) distance.getDistance(DistanceUnit.INCH) - 4000; //6650
                //targetPos = 1700;
            } else {
                targetPos = 1700;
            }

        } else if (location == 1) {
            LEDs.setPower(.5);
            LEDs.setPower(0);
            sleep(200);
            LEDs.setPower(0.5);
            sleep(200);
            LEDs.setPower(0);

            drive.followTrajectory(fondue);
            if (distance.getDistance(DistanceUnit.INCH) < 30) {
                targetPos = 190 * (int) distance.getDistance(DistanceUnit.INCH) -4460; //7700
            } else {
                targetPos = 2500;
            }

        } else if (location == 2) {
            LEDs.setPower(.5);
            LEDs.setPower(0);
            sleep(200);
            LEDs.setPower(0.5);
            LEDs.setPower(0);
            sleep(200);
            LEDs.setPower(0.5);
            sleep(200);
            LEDs.setPower(0);

            drive.followTrajectory(fondue);
            if (distance.getDistance(DistanceUnit.INCH) < 30) {
                targetPos = 190 * (int) distance.getDistance(DistanceUnit.INCH) - 5000;//9100
            } else {
                targetPos = 3600;
            }

        } else {
            Wheel.setPower(0.5);
            sleep(1000);
            Wheel.setPower(0);
        }

        difference = 18 - distance.getDistance(DistanceUnit.INCH);
        telemetry.addData("dISTANCE: ", distance.getDistance(DistanceUnit.INCH));
        telemetry.addData("Differnec", difference);
        telemetry.update();

        ElapsedTime extend = new ElapsedTime();
        extend.reset();
        while (extend.time() <= 3.0) {
            if (!(LSlides.getCurrentPosition() >= targetPos - 100 && LSlides.getCurrentPosition() <= targetPos + 100)) {
                LSlides.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
                LSlides.setPower(-0.8);
                LSlides.setTargetPosition(targetPos); //last number was 1600

                LSlides.setMode(DcMotor.RunMode.RUN_TO_POSITION);
                while (LSlides.isBusy()) {
                    telemetry.addData("Curent pos: ", LSlides.getCurrentPosition());
                    telemetry.update();

                    idle();

                }
            }
        }
        sleep(150);

        if (LSlides.getCurrentPosition() >= targetPos - 100 && LSlides.getCurrentPosition() <= targetPos + 100) {
            sleep(100);
            Bin.setPosition(1.0);
            sleep(2000);
            Bin.setPosition(0.5);
            sleep(100);
        } else {
            if (!(LSlides.getCurrentPosition() >= targetPos - 100 && LSlides.getCurrentPosition() <= targetPos + 100)) {
                LSlides.setPower(-0.8);
                LSlides.setTargetPosition(targetPos); //last number was 1600

                LSlides.setMode(DcMotor.RunMode.RUN_TO_POSITION);
                while (LSlides.isBusy()) {
                    telemetry.addData("Curent pos: ", LSlides.getCurrentPosition());
                    telemetry.update();

                    idle();

                }
            }
            if (LSlides.getCurrentPosition() >= targetPos - 100 && LSlides.getCurrentPosition() <= targetPos + 100) {
                sleep(100);
                Bin.setPosition(1.0);
                sleep(2000);
                Bin.setPosition(0.5);
                sleep(100);
            }
        }

        LSlides.setTargetPosition(0);
        LSlides.setPower(-0.8);
        while (LSlides.isBusy()) {
            idle();
        }
        LSlides.setPower(0);


        if (LSlides.getCurrentPosition() >= 0 && LSlides.getCurrentPosition() <= 100) {
            drive.followTrajectory(duck);
        } else {
            LSlides.setTargetPosition(0);
            LSlides.setPower(-0.8);
            while (LSlides.isBusy()) {
                idle();
            }
            LSlides.setPower(0);
            sleep(800);
            drive.followTrajectory(duck);
        }

        /*
        telemetry.addData("Wheel: ", wheelSensor.getDistance(DistanceUnit.INCH));
        turn = wheelSensor.getDistance(DistanceUnit.INCH) - 4;
        telemetry.addData("Turn Value", turn);
        telemetry.update();
        sleep(2000);
        telemetry.update();


        if(turn >= 0.5){
            turn = (turn + 5)*-1;
        }else if(turn<0){
            turn = 0; //joey was here
        }

         */
        //drive.turn(Math.toRadians(turn) - 1e-6);
        //sleep(400);
        extend.reset();
        while (extend.time() <= 3.00) {
            Wheel.setPower(-1*(extend.time()/3));
        }
        //sleep(3000);
        Wheel.setPower(0);
        sleep(800);
        drive.followTrajectory(park);



    }
}
