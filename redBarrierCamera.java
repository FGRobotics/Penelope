package org.firstinspires.ftc.teamcode.drive.opmode;

import com.acmerobotics.roadrunner.geometry.Pose2d;
import com.acmerobotics.roadrunner.geometry.Vector2d;
import com.acmerobotics.roadrunner.trajectory.Trajectory;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.ColorRangeSensor;
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
@Autonomous(group = "Drive", name = "redWarehouseMulti")
public class redBarrierCamera extends LinearOpMode {
    private DcMotorEx LSlides, Wheel, LEDs, Intake;
    DistanceSensor distance, BinSensor;
    private Servo Bin;

    public ElapsedTime wheelRun = new ElapsedTime(0);


    private List<DcMotorEx> motors;

    @Override
    public void runOpMode() throws InterruptedException {
        Intake = hardwareMap.get(DcMotorEx.class, "Intake");
        Wheel = hardwareMap.get(DcMotorEx.class, "Wheel");
        LSlides = hardwareMap.get(DcMotorEx.class, "LSlides");

        LSlides.setDirection(DcMotorSimple.Direction.REVERSE);
        Bin = hardwareMap.get(Servo.class, "Bin");
        distance = hardwareMap.get(DistanceSensor.class, "toaster");
        distance.getDistance(DistanceUnit.INCH);
        LEDs = hardwareMap.get(DcMotorEx.class, "LEDs");
        LEDs.setDirection(DcMotorSimple.Direction.REVERSE);


        BinSensor = hardwareMap.get(DistanceSensor.class, "BinSensor");
        BinSensor.getDistance(DistanceUnit.INCH);
        //Bin start position - 0.4 is too low and cause problems coming back in, 0.5 cause issues intaking sometimes
        Bin.setPosition(0.5);


        //initTurret.setPosition(.8);

        SampleMecanumDrive drive = new SampleMecanumDrive(hardwareMap);
        Pose2d myPose = new Pose2d(10, -62, Math.toRadians(270));
        drive.setPoseEstimate(myPose);
        double difference = 0;

        Pose2d parkT = new Pose2d(-2, -44, Math.toRadians(5));


        Trajectory fondue = drive.trajectoryBuilder(myPose)

                .lineToConstantHeading(new Vector2d(-12, -51)) //-51
                .build();

        Trajectory park = drive.trajectoryBuilder(fondue.end())
                .lineToSplineHeading(new Pose2d(8, -69, Math.toRadians(10)))

                .build();
        Trajectory apark = drive.trajectoryBuilder(fondue.end())
                .lineToSplineHeading(new Pose2d(8, -70, Math.toRadians(10)))

                .build();






        Trajectory FPark = drive.trajectoryBuilder(apark.end())
                .lineTo(new Vector2d(40, -71),
                        SampleMecanumDrive.getVelocityConstraint(25, DriveConstants.MAX_ANG_VEL, DriveConstants.TRACK_WIDTH),
                        SampleMecanumDrive.getAccelerationConstraint(20))
                .build();





        Trajectory FFPark = drive.trajectoryBuilder(FPark.end())
                .lineToSplineHeading(new Pose2d(55, -70, Math.toRadians(0)),
                        SampleMecanumDrive.getVelocityConstraint(25, DriveConstants.MAX_ANG_VEL, DriveConstants.TRACK_WIDTH),
                        SampleMecanumDrive.getAccelerationConstraint(24))

                .build();

        Trajectory release = drive.trajectoryBuilder(FFPark.end())
                .lineTo(new Vector2d(40,-70))
                .build();


        Trajectory leave = drive.trajectoryBuilder(release.end())
                .lineTo(new Vector2d(30,-50))
                .build();

        Trajectory fontwo = drive.trajectoryBuilder(leave.end())
                .lineTo(new Vector2d(-10,-50))
                .build();
        Trajectory fontthree = drive.trajectoryBuilder(fontwo.end())
                .lineToSplineHeading(new Pose2d(-11,-32,Math.toRadians(-52)))
                .build();


        Trajectory reverse = drive.trajectoryBuilder(fontthree.end())
                .lineToSplineHeading(new Pose2d(20,-40,Math.toRadians(-200)))
                .build();

        Trajectory over = drive.trajectoryBuilder(reverse.end())
                .lineTo(new Vector2d(72,-40))
                .build();












        Trajectory toBlocks = drive.trajectoryBuilder(FFPark.end())
                .lineToSplineHeading(new Pose2d(65, -61, Math.toRadians(-95)))
                .build();
        Trajectory back = drive.trajectoryBuilder(toBlocks.end())
                .lineTo(new Vector2d(65, -50))
                .build();
        Trajectory lineBarrier = drive.trajectoryBuilder(back.end())
                .lineToSplineHeading(new Pose2d(40, -55, Math.toRadians(20)))
                .build();
        Trajectory overBarrier = drive.trajectoryBuilder(lineBarrier.end())
                .lineToSplineHeading(new Pose2d(-3, -55, Math.toRadians(20)),
                        SampleMecanumDrive.getVelocityConstraint(50, DriveConstants.MAX_ANG_VEL, DriveConstants.TRACK_WIDTH),
                        SampleMecanumDrive.getAccelerationConstraint(40))
                .build();

        Trajectory lineFountain = drive.trajectoryBuilder(overBarrier.end())
                .lineToSplineHeading(new Pose2d(-6, -55, Math.toRadians(340)))
                .build();


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
                telemetry.addData("BinSensor: ", BinSensor.getDistance(DistanceUnit.INCH));
                telemetry.update();
            }

            @Override
            public void onError(int errorCode) {
                telemetry.addData("Init Failed ", errorCode);
                telemetry.update();

            }
        });


        int location = 2;
        int targetPos = 0;
        while (!opModeIsActive()) {
            //int testRun = 1;
            //int finalTestRun = testRun;
            //telemetry.addData("Pass number ", finalTestRun);
            telemetry.update();

            //red duck side 150,350,680,880,1380,1580,600,99
            // red barrier side 440, 640, 1080, 1280, 1680, 1880, 650, 950



            location = myPipeline.getLocation();
            telemetry.addData("location: ", location);
            telemetry.update();
            //testRun++;
        }
        waitForStart();
        location = myPipeline.getLocation();
        telemetry.addLine("location: " + location);
        telemetry.update();


        if (isStopRequested()) return;
        LSlides.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);


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
            if (distance.getDistance(DistanceUnit.INCH) < 24) {
                targetPos = 190 * (int) distance.getDistance(DistanceUnit.INCH) -4100;
                //targetPos = 1700;
            } else {
                targetPos = 1700;
            }
        } else if (location == 1) {

            drive.followTrajectory(fondue);
            if (distance.getDistance(DistanceUnit.INCH) < 24) {
                targetPos = 190 * (int) distance.getDistance(DistanceUnit.INCH) - 4530;
            } else {
                targetPos = 2500;
            }
            //targetPos = 2500;

        } else if (location == 2) {

            drive.followTrajectory(fondue);
            if (distance.getDistance(DistanceUnit.INCH) < 24) {
                targetPos = 190 * (int) distance.getDistance(DistanceUnit.INCH) - 5400;
            } else {
                targetPos = 3600;
            }
        } else {
            Wheel.setPower(0.5);
            sleep(1000);
            Wheel.setPower(0);
        }

        difference = 18 - distance.getDistance(DistanceUnit.INCH);
        telemetry.addData("DISTANCE: ", distance.getDistance(DistanceUnit.INCH));
        telemetry.addData("Difference", difference);
        telemetry.update();

        ElapsedTime extend = new ElapsedTime();
        extend.reset();
        while (extend.time() <= 2.0) {
            if (!(LSlides.getCurrentPosition() >= targetPos - 100 && LSlides.getCurrentPosition() <= targetPos + 100)) {
                LSlides.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
                LSlides.setPower(-1);
                LSlides.setTargetPosition(targetPos); //last number was 1600

                LSlides.setMode(DcMotor.RunMode.RUN_TO_POSITION);
                while (LSlides.isBusy()) {
                    //telemetry.addData("Curent pos: ", LSlides.getCurrentPosition());
                    //telemetry.update();

                    idle();

                }
            }
        }


        if (LSlides.getCurrentPosition() >= targetPos - 100 && LSlides.getCurrentPosition() <= targetPos + 100) {
            //sleep(100);
            Bin.setPosition(1.0);
            sleep(2000);
            Bin.setPosition(0.5);
            sleep(100);
        } else {
            if (!(LSlides.getCurrentPosition() >= targetPos - 100 && LSlides.getCurrentPosition() <= targetPos + 100)) {
                LSlides.setPower(-1);
                LSlides.setTargetPosition(targetPos); //last number was 1600

                LSlides.setMode(DcMotor.RunMode.RUN_TO_POSITION);

                while (LSlides.isBusy()) {
                    //telemetry.addData("Curent pos: ", LSlides.getCurrentPosition());
                    //telemetry.update();

                    idle();

                }
            }
            if (LSlides.getCurrentPosition() >= targetPos - 100 && LSlides.getCurrentPosition() <= targetPos + 100) {
                //sleep(100);
                Bin.setPosition(1.0);
                sleep(2000);
                Bin.setPosition(0.5);
                sleep(100);
            }
        }

        LSlides.setTargetPosition(0);
        LSlides.setPower(-1);
        while (LSlides.isBusy()) {

            idle();
        }
        LSlides.setPower(0);
        if (LSlides.getCurrentPosition() >= 0 && LSlides.getCurrentPosition() <= 100) {
            drive.followTrajectory(park);
        } else {
            LSlides.setTargetPosition(0);
            LSlides.setPower(-1);
            while (LSlides.isBusy()) {
                idle();
            }
            LSlides.setPower(0);

            drive.followTrajectory(park);

            drive.followTrajectory(apark);

        }

        drive.followTrajectory(FPark);

        Intake.setPower(1);
        drive.followTrajectory(FFPark);

        sleep(400);

        Intake.setPower(0.14);

        drive.followTrajectory(release);

        ElapsedTime endTimer = new ElapsedTime();
        endTimer.reset();




            while(BinSensor.getDistance(DistanceUnit.INCH ) > 3 ){

                Intake.setPower(1);




            }



        if(endTimer.time() < 2.0) {
            Intake.setPower(-1);



            drive.followTrajectory(leave);


            drive.followTrajectory(fontwo);

            Intake.setPower(0);



            drive.followTrajectory(fontthree);


            extend.reset();

            targetPos = - 2000;



                while (!(LSlides.getCurrentPosition() >= targetPos - 100 && LSlides.getCurrentPosition() <= targetPos + 100)) {
                    LSlides.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
                    LSlides.setPower(-1);
                    LSlides.setTargetPosition(targetPos); //last number was 1600

                    LSlides.setMode(DcMotor.RunMode.RUN_TO_POSITION);
                    while (LSlides.isBusy()) {
                        //telemetry.addData("Curent pos: ", LSlides.getCurrentPosition());
                        //telemetry.update();

                        idle();

                    }
                }



            sleep(100);
            Bin.setPosition(1.0);
            sleep(2000);
            Bin.setPosition(0.5);
            sleep(100);

            targetPos = 0;

            /*
                while (!(LSlides.getCurrentPosition() >= targetPos - 100 && LSlides.getCurrentPosition() <= targetPos + 100)) {
                    LSlides.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
                    LSlides.setPower(-1);
                    LSlides.setTargetPosition(targetPos); //last number was 1600

                    LSlides.setMode(DcMotor.RunMode.RUN_TO_POSITION);
                    while (LSlides.isBusy()) {
                        //telemetry.addData("Curent pos: ", LSlides.getCurrentPosition());
                        //telemetry.update();

                        idle();

                    }
                }
*/

                drive.followTrajectory(reverse);
                sleep(100);
                drive.followTrajectory(over);












        }else if(endTimer.time() > 2){
            Intake.setPower(-1);
            sleep(1000);
            Intake.setPower(0);

        }













        while (opModeIsActive()) {



        }



    }

    public void doubleBlock(){
        Intake.setPower(-1);
        sleep(2500);
        Intake.setPower(0);



    }
}
/*
 Pose2d parkT = new Pose2d(-2, -42, Math.toRadians(5));
        Trajectory fondue = drive.trajectoryBuilder(myPose)
                //.back(20)
                .lineToSplineHeading(new Pose2d(-9, -51, Math.toRadians(270)),
                        SampleMecanumDrive.getVelocityConstraint(40, DriveConstants.MAX_ANG_VEL, DriveConstants.TRACK_WIDTH),
                        SampleMecanumDrive.getAccelerationConstraint(30))
                .build();
        Trajectory park = drive.trajectoryBuilder(myPose)
                //.back(20)
                //.lineToSplineHeading(new Pose2d(-26, -51, Math.toRadians(230)))
                //.lineTo(new Vector2d(6,-42))
                .lineToSplineHeading(new Pose2d(-2, -42, Math.toRadians(5)))
                .build();
        Trajectory FPark = drive.trajectoryBuilder(parkT)
                //.back(20)
                .lineTo(new Vector2d(60, -42),
                        SampleMecanumDrive.getVelocityConstraint(50, DriveConstants.MAX_ANG_VEL, DriveConstants.TRACK_WIDTH),
                        SampleMecanumDrive.getAccelerationConstraint(40))
                .build();
        Trajectory FFPark = drive.trajectoryBuilder(FPark.end())
                .lineToSplineHeading(new Pose2d(60, -50, Math.toRadians(5)))
                .build();
*/