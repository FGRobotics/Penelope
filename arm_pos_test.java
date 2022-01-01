package org.firstinspires.ftc.teamcode.drive.opmode;

import com.acmerobotics.roadrunner.geometry.Pose2d;
import com.acmerobotics.roadrunner.trajectory.Trajectory;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.hardware.configuration.typecontainers.MotorConfigurationType;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.teamcode.drive.SampleMecanumDrive;

import java.util.Arrays;
import java.util.List;
@TeleOp(name="armTest")
public class arm_pos_test extends LinearOpMode {
    private DcMotorEx LSlides;

    private Servo Bin;
    public ElapsedTime wheelRun = new ElapsedTime(0);


    private List<DcMotorEx> motors;

    @Override
    public void runOpMode() throws InterruptedException{

        LSlides = hardwareMap.get(DcMotorEx.class, "LSlides");
        Bin = hardwareMap.get(Servo.class, "Bin");

//Bin start position - 0.4 is too low and cause problems coming back in, 0.5 cause issues intaking sometimes
        Bin.setPosition(0.5);

        SampleMecanumDrive drive = new SampleMecanumDrive(hardwareMap);
        Pose2d myPose = new Pose2d(-30, -62, Math.toRadians(270));//intake facing back wall
        drive.setPoseEstimate(myPose);

        Trajectory fondue = drive.trajectoryBuilder(myPose)
                //.back(20)
                .lineToSplineHeading(new Pose2d(-1, -14, Math.toRadians(240)))
                .build();

        waitForStart();

        drive.followTrajectory(fondue);

        while(opModeIsActive()) {
            // Variable setup
            int cPos = LSlides.getCurrentPosition();
            double LSlidesPower = 0.0;
            double LSlidesRotation = (LSlides.getCurrentPosition()/1680.0);
            double upRange = 0.5;
            double downRange = 1.0;
            //Telemetry
            LSlidesRotation = LSlides.getCurrentPosition();

            telemetry.addData("Slides current rotation: ", LSlidesRotation);
            telemetry.addData("Slides tick position: ", LSlides.getCurrentPosition());
            telemetry.update();

            if(gamepad1.dpad_up){
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
            }
        }


    }
}
