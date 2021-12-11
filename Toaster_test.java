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

/*
 * This is an example of a more complex path to really test the tuning.
 */
@Autonomous(group = "drive")
public class Toaster_test extends LinearOpMode {
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

        Pose2d myPose = new Pose2d(54, -20, Math.toRadians(0));//intake facing back wall
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
                .lineTo(new Vector2d(-10, -50))
                .build();

        //duck
        Trajectory duck = drive.trajectoryBuilder(fondue.end())
                .lineTo(new Vector2d(34,4))
                .build();

        //park in tape
        Trajectory tPark = drive.trajectoryBuilder(duck.end())
                .lineTo(new Vector2d(-20,2))
                //.splineTo(new Vector2d(-54, -36), Math.toRadians(0))
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
        drive.followTrajectory(duck);
        drive.turn(Math.toRadians(-90));
        //drive.wait(3000);
        drive.followTrajectory(tPark);


        /*
        drive.followTrajectory(duck);
        telemetry.addData("Trajectory: ", "t2 done");

        drive.followTrajectory(tPark);
        telemetry.addData("Trajectory: ", "park done");
        */

    }

}
