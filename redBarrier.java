package org.firstinspires.ftc.teamcode.drive.opmode;

import com.acmerobotics.roadrunner.geometry.Pose2d;
import com.acmerobotics.roadrunner.geometry.Vector2d;
import com.acmerobotics.roadrunner.trajectory.Trajectory;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.teamcode.drive.SampleMecanumDrive;

import static org.firstinspires.ftc.robotcore.external.BlocksOpModeCompanion.hardwareMap;
import static org.firstinspires.ftc.robotcore.external.BlocksOpModeCompanion.telemetry;
@Autonomous(group = "drive")
public class redBarrier extends LinearOpMode {

        private DcMotorEx SlidesAngle, LSlides, Intake, Wheel;
        private Servo Bin;
        private ElapsedTime runtime = new ElapsedTime(0);

        public void runOpMode() throws InterruptedException {
            SlidesAngle = hardwareMap.get(DcMotorEx.class, "SlidesAngle");
            LSlides = hardwareMap.get(DcMotorEx.class, "LSlides");
            Bin = hardwareMap.get(Servo.class, "Bin");
            Wheel = hardwareMap.get(DcMotorEx.class, "Wheel");
            Intake = hardwareMap.get(DcMotorEx.class, "Intake");
            Wheel.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);

            SampleMecanumDrive drive = new SampleMecanumDrive(hardwareMap);
            Bin.setPosition(0.5);

            Pose2d myPose = new Pose2d(10, -62, Math.toRadians(270));
            drive.setPoseEstimate(myPose);

            telemetry.addData("Status", "Initialized");
            telemetry.update();
            waitForStart();

            if (isStopRequested()) return;

            Trajectory fondue = drive.trajectoryBuilder(myPose)
                    //.back(20)
                    .lineToSplineHeading(new Pose2d(-10, 1, Math.toRadians(310)))

                    .addTemporalMarker(6,()->{


                        LSlides.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
                        LSlides.setTargetPosition(3800);
                        LSlides.setMode(DcMotor.RunMode.RUN_TO_POSITION);
                        LSlides.setPower(0.8);
                        SlidesAngle.setPower(0.5);


                        while(LSlides.isBusy()){
                            telemetry.addData("Curent pos: ", LSlides.getCurrentPosition());
                            telemetry.update();

                            idle();

                        }


                        LSlides.setPower(0.0);
                        Bin.setPosition(1.0);
                        sleep(2000);
                        Bin.setPosition(0.5);
                        LSlides.setTargetPosition(800);
                        //LSlides.setMode(DcMotor.RunMode.RUN_TO_POSITION);
                        LSlides.setPower(-0.7);

                        while(LSlides.isBusy()){
                            telemetry.addData("Curent pos: ", LSlides.getCurrentPosition());
                            telemetry.update();
                            //idle
                            SlidesAngle.setPower(-0.1);
                        }
                        LSlides.setPower(0);
                        SlidesAngle.setPower(0.0);



                    })


                    .build();
            Trajectory park = drive.trajectoryBuilder(fondue.end())
                    .lineToSplineHeading(new Pose2d(60, -26, Math.toRadians(0)))
                    .build();
            Trajectory tPark = drive.trajectoryBuilder(park.end())
                    .lineToConstantHeading(new Vector2d(160, -36))
                    .build();
            drive.followTrajectory(fondue);
            drive.followTrajectory(park);
            drive.followTrajectory(tPark);
        }

    }

