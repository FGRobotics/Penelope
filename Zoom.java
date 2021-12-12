
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

import java.util.Arrays;
import java.util.Vector;

/*
 * This is an example of a more complex path to really test the tuning.
 */
@Autonomous(group = "drive")
public class Zoom extends LinearOpMode {
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
        LSlides.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        LSlides.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        double LSlidesPower;
        SampleMecanumDrive drive = new SampleMecanumDrive(hardwareMap);

        Bin.setPosition(0.5);
        Pose2d duckPose = new Pose2d(-85, -80, Math.toRadians(180));
        Pose2d myPose = new Pose2d(-30, -62, Math.toRadians(270));//intake facing back wall
        drive.setPoseEstimate(myPose);

        telemetry.addData("Status", "Initialized");
        telemetry.update();
        waitForStart();

        if (isStopRequested()) return;
        /*
        Trajectory traj = drive.trajectoryBuilder(myPose)
                .splineTo(new Vector2d(30, 30), 0)
                .build();
        drive.followTrajectory(traj);
        sleep(2000);
        drive.followTrajectory(
                drive.trajectoryBuilder(traj.end(), true)
                        .splineTo(new Vector2d(0, 0), Math.toRadians(180))
                        .build()
        );
        */
        //START-------------------------------------------------------------------------------------

        //trajectories
        Trajectory fondue = drive.trajectoryBuilder(myPose)
                //.back(20)
                .lineToSplineHeading(new Pose2d(0, -15, Math.toRadians(240)))
                /*.addDisplacementMarker(()->{

                    LSlides.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
                    LSlides.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
                    //LSlides.setTargetPosition(2400);
                    //LSlides.setMode(DcMotor.RunMode.RUN_TO_POSITION);
                    LSlides.setPower(0.4);
                    sleep(2000);
                    /*
                    while(LSlides.isBusy()){
                        idle();

                    }


                    LSlides.setPower(0.0);
                    Bin.setPosition(1.0);
                    sleep(2000);
                    Bin.setPosition(0.5);
                    LSlides.setTargetPosition(0);
                    //LSlides.setMode(DcMotor.RunMode.RUN_TO_POSITION);
                    LSlides.setPower(-0.2);
                    sleep(750);
                    while(LSlides.isBusy()){
                        idle();
                        SlidesAngle.setPower(-0.1);
                    }
                    LSlides.setPower(0);



                })*/
                .build();
        //avoid barcode
        Trajectory back = drive.trajectoryBuilder(fondue.end())
               .lineTo(new Vector2d(-18, -40))
                .build();


        //duck
        Trajectory duck = drive.trajectoryBuilder(fondue.end())
                .lineToSplineHeading(new Pose2d(-85, -80, Math.toRadians(240)))
                .build();

        //park in tape
        Trajectory tPark = drive.trajectoryBuilder(duckPose)
                .lineToSplineHeading(new Pose2d(-98,-73, Math.toRadians(180)))
                //.splineTo(new Vector2d(-85, -55), Math.toRadians(180))
                //.lineToConstantHeading(new Vector2d(-85, -65))
                .build();

        /*
        //park in warehouse
        Trajectory Wpark = drive.trajectoryBuilder(duck.end())
                .splineTo(new Vector2d(9,-54), Math.toRadians(0))
                .strafeTo(new Vector2d(9, -40))
                .lineTo(new Vector2d(54, -40))
                .build();
           */

        drive.followTrajectory(fondue);
        LSlidesPower = 0.4;

        /*LSlides.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        LSlides.setTargetPosition(2400);
        LSlides.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        LSlides.setVelocity(100);
        //sleep(2000);





                    Bin.setPosition(1.0);
                    sleep(2000);
                    Bin.setPosition(0.5);
                    LSlides.setTargetPosition(0);
                    LSlides.setMode(DcMotor.RunMode.RUN_TO_POSITION);
                    LSlides.setVelocity(-100);
                    //sleep(750);*/


        //drive.followTrajectory(back);
        drive.followTrajectory(duck);
        drive.turn(Math.toRadians(-60));
        Wheel.setPower(-0.2);
        sleep(3000);
        Wheel.setPower(0);
        drive.followTrajectory(tPark);





        /*
        drive.followTrajectory(duck);
        telemetry.addData("Trajectory: ", "t2 done");
        drive.followTrajectory(tPark);
        telemetry.addData("Trajectory: ", "park done");
        */

    }

}