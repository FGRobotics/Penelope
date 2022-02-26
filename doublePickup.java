package org.firstinspires.ftc.teamcode.drive.opmode;

import com.acmerobotics.roadrunner.geometry.Pose2d;
import com.acmerobotics.roadrunner.geometry.Vector2d;
import com.acmerobotics.roadrunner.trajectory.Trajectory;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.DistanceSensor;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.robotcore.external.hardware.camera.WebcamName;
import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit;
import org.firstinspires.ftc.teamcode.drive.SampleMecanumDrive;
import org.openftc.easyopencv.OpenCvCamera;
import org.openftc.easyopencv.OpenCvCameraFactory;
import org.openftc.easyopencv.OpenCvCameraRotation;
import org.openftc.easyopencv.OpenCvWebcam;

import java.util.List;
@Autonomous(group = "Drive")
public class doublePickup extends LinearOpMode {
    private DcMotorEx LSlides, Wheel;
    DistanceSensor distance;
    private Servo Bin;
    public ElapsedTime wheelRun = new ElapsedTime(0);


    private List<DcMotorEx> motors;

    @Override
    public void runOpMode() throws InterruptedException {

        Wheel = hardwareMap.get(DcMotorEx.class, "Wheel");
        LSlides = hardwareMap.get(DcMotorEx.class, "LSlides");
        Bin = hardwareMap.get(Servo.class, "Bin");
        distance = hardwareMap.get(DistanceSensor.class, "toaster");
        distance.getDistance(DistanceUnit.INCH);
        
        rightDistance = hardwareMap.get(DistanceSensor.class, "wheelSensor");
        rightDistance.getDistance(DistanceUnit.INCH);

//Bin start position - 0.4 is too low and cause problems coming back in, 0.5 cause issues intaking sometimes
        Bin.setPosition(0.5);

        SampleMecanumDrive drive = new SampleMecanumDrive(hardwareMap);
        Pose2d myPose = new Pose2d(10, -62, Math.toRadians(270));
        drive.setPoseEstimate(myPose);
        double difference = 0;

        Pose2d parkT = new Pose2d(-2, -36, Math.toRadians(0));


        Trajectory fondue = drive.trajectoryBuilder(myPose)
                
                .lineToSplineHeading(new Pose2d(-9, -48, Math.toRadians(270)))
                .build();
        
        Trajectory park = drive.trajectoryBuilder(myPose)
                .lineToSplineHeading(new Pose2d(-2, -36, Math.toRadians(0)))
                .build();
        
        Trajectory FPark = drive.trajectoryBuilder(parkT)
                .lineTo(new Vector2d(40, -36))
                .build();
        


        waitForStart();
        Wheel.setPower(0.5);
        sleep(800);
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

        int location = 2;
        int targetPos = 0;

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
            if(distance.getDistance(DistanceUnit.INCH) < 20) {
                targetPos = -300 * (int) distance.getDistance(DistanceUnit.INCH) + 6750;
                //targetPos = 1700;
            }else{
                targetPos = 1700;
            }
        } else if (location == 1) {
            drive.followTrajectory(fondue);
            if(distance.getDistance(DistanceUnit.INCH) < 20) {
                targetPos = -300 * (int) distance.getDistance(DistanceUnit.INCH) + 7600;
            }else{
                targetPos = 2500;
            }
            //targetPos = 2500;


        } else if (location == 2) {
            drive.followTrajectory(fondue);
            if(distance.getDistance(DistanceUnit.INCH) < 20) {
                targetPos = -300 * (int) distance.getDistance(DistanceUnit.INCH) + 9000;
            }else {
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
        while(extend.time() <= 2.0 ) {
            if(!(LSlides.getCurrentPosition() >= targetPos - 100 && LSlides.getCurrentPosition() <= targetPos + 100)) {
                LSlides.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
                LSlides.setPower(-0.8);
                LSlides.setTargetPosition(targetPos); //last number was 1600

                LSlides.setMode(DcMotor.RunMode.RUN_TO_POSITION);
                while (LSlides.isBusy()) {
                    //telemetry.addData("Curent pos: ", LSlides.getCurrentPosition());
                    //telemetry.update();

                    idle();

                }
            }
        }





        if(LSlides.getCurrentPosition() >= targetPos - 100 && LSlides.getCurrentPosition() <= targetPos + 100){
        sleep(1000);
        Bin.setPosition(1.0);
        sleep(1000);
        Bin.setPosition(0.5);
        sleep(1000);
        }else{
            if(!(LSlides.getCurrentPosition() >= targetPos - 100 && LSlides.getCurrentPosition() <= targetPos + 100)) {
                LSlides.setPower(-0.8);
                LSlides.setTargetPosition(targetPos); //last number was 1600

                LSlides.setMode(DcMotor.RunMode.RUN_TO_POSITION);
                while (LSlides.isBusy()) {
                    //telemetry.addData("Curent pos: ", LSlides.getCurrentPosition());
                    //telemetry.update();

                    idle();

                }
            }
            if(LSlides.getCurrentPosition() >= targetPos - 100 && LSlides.getCurrentPosition() <= targetPos + 100){
                sleep(1000);
                Bin.setPosition(1.0);
                sleep(1000);
                Bin.setPosition(0.5);
                sleep(1000);
            }
        }

        LSlides.setTargetPosition(0);
        LSlides.setPower(-0.8);
        while(LSlides.isBusy()){
            idle();
        }
        LSlides.setPower(0);
        if(LSlides.getCurrentPosition() >= 0 && LSlides.getCurrentPosition() <= 100) {
            drive.followTrajectory(park);
        }else{
            LSlides.setTargetPosition(0);
            LSlides.setPower(-0.8);
            while(LSlides.isBusy()){
                idle();
            }
            LSlides.setPower(0);
            sleep(800);
            drive.followTrajectory(park);

        }
        sleep(800);
       
        drive.followTrajectory(FPark);
        
        int rightSide = -72 + rightDistance.getDistance(DistanceUnit.INCH);
        int front = 72 - distance.getDistance(DistanceUnit.INCH);
      
      
      
      
        Pose2d inWarehouse = new Pose2d(rightSide, front, Math.toRadians(0));

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
