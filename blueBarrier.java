package org.firstinspires.ftc.teamcode.drive.opmode;

import com.acmerobotics.roadrunner.geometry.Pose2d;
import com.acmerobotics.roadrunner.trajectory.Trajectory;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.util.ElapsedTime;
import com.acmerobotics.roadrunner.geometry.Vector2d;


import org.firstinspires.ftc.teamcode.drive.SampleMecanumDrive;

import static org.firstinspires.ftc.robotcore.external.BlocksOpModeCompanion.hardwareMap;
@Autonomous(group = "drive")
public class blueBarrier extends LinearOpMode {
    private DcMotorEx LSlides, Intake, Wheel;
    private Servo Bin;
    private ElapsedTime runtime = new ElapsedTime(0);

    public void runOpMode() throws InterruptedException {

        LSlides = hardwareMap.get(DcMotorEx.class, "LSlides");
        Bin = hardwareMap.get(Servo.class, "Bin");
        Wheel = hardwareMap.get(DcMotorEx.class, "Wheel");
        Intake = hardwareMap.get(DcMotorEx.class, "Intake");
        Wheel.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);

        SampleMecanumDrive drive = new SampleMecanumDrive(hardwareMap);
        Pose2d myPose = new Pose2d(10, 62, Math.toRadians(90));
        drive.setPoseEstimate(myPose);
        Pose2d turn = new Pose2d(10, 26, Math.toRadians(180));
        telemetry.addData("Status", "Initialized");
        telemetry.update();
        waitForStart();
        if (isStopRequested()) return;
        Trajectory forward = drive.trajectoryBuilder(myPose)
                .lineToConstantHeading(new Vector2d(10, 26))
                .build();


        Trajectory park = drive.trajectoryBuilder(turn)
                .lineToConstantHeading(new Vector2d(150, 26))
                .
                        build();

        drive.followTrajectory(forward);
        drive.turn(Math.toRadians(90));
        drive.followTrajectory(park);






    }
}