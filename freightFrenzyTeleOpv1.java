package org.firstinspires.ftc.teamcode.drive.opmode;
import androidx.annotation.NonNull;

import com.acmerobotics.dashboard.config.Config;
import com.acmerobotics.roadrunner.control.PIDCoefficients;
import com.acmerobotics.roadrunner.drive.DriveSignal;
import com.acmerobotics.roadrunner.drive.MecanumDrive;
import com.acmerobotics.roadrunner.followers.HolonomicPIDVAFollower;
import com.acmerobotics.roadrunner.followers.TrajectoryFollower;
import com.acmerobotics.roadrunner.geometry.Pose2d;
import com.acmerobotics.roadrunner.trajectory.Trajectory;
import com.acmerobotics.roadrunner.trajectory.TrajectoryBuilder;
import com.acmerobotics.roadrunner.trajectory.constraints.AngularVelocityConstraint;
import com.acmerobotics.roadrunner.trajectory.constraints.MecanumVelocityConstraint;
import com.acmerobotics.roadrunner.trajectory.constraints.MinVelocityConstraint;
import com.acmerobotics.roadrunner.trajectory.constraints.ProfileAccelerationConstraint;
import com.acmerobotics.roadrunner.trajectory.constraints.TrajectoryAccelerationConstraint;
import com.acmerobotics.roadrunner.trajectory.constraints.TrajectoryVelocityConstraint;
import com.qualcomm.hardware.bosch.BNO055IMU;
import com.qualcomm.hardware.lynx.LynxModule;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.CRServo;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.LED;
import com.qualcomm.robotcore.hardware.PIDFCoefficients;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.hardware.VoltageSensor;
import com.qualcomm.robotcore.hardware.configuration.typecontainers.MotorConfigurationType;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.teamcode.trajectorysequence.TrajectorySequence;
import org.firstinspires.ftc.teamcode.trajectorysequence.TrajectorySequenceBuilder;
import org.firstinspires.ftc.teamcode.trajectorysequence.TrajectorySequenceRunner;
import org.firstinspires.ftc.teamcode.util.LynxModuleUtil;

import java.util.ArrayList;
import java.util.Arrays;
import java.util.List;

import static org.firstinspires.ftc.teamcode.drive.DriveConstants.MAX_ACCEL;
import static org.firstinspires.ftc.teamcode.drive.DriveConstants.MAX_ANG_ACCEL;
import static org.firstinspires.ftc.teamcode.drive.DriveConstants.MAX_ANG_VEL;
import static org.firstinspires.ftc.teamcode.drive.DriveConstants.MAX_VEL;
import static org.firstinspires.ftc.teamcode.drive.DriveConstants.MOTOR_VELO_PID;
import static org.firstinspires.ftc.teamcode.drive.DriveConstants.RUN_USING_ENCODER;
import static org.firstinspires.ftc.teamcode.drive.DriveConstants.TRACK_WIDTH;
import static org.firstinspires.ftc.teamcode.drive.DriveConstants.encoderTicksToInches;
import static org.firstinspires.ftc.teamcode.drive.DriveConstants.kA;
import static org.firstinspires.ftc.teamcode.drive.DriveConstants.kStatic;
import static org.firstinspires.ftc.teamcode.drive.DriveConstants.kV;
@TeleOp(name="FrenzyTelev1")
public class freightFrenzyTeleOpv1 extends LinearOpMode {
    private DcMotorEx leftFront, leftRear, rightRear, rightFront, LSlides, Wheel, Intake, LEDs;
    private Servo Bin, Claw, UpTurret, HorizontallyTurret, initTurret;
    private CRServo extendTurret;


    private List<DcMotorEx> motors;

    @Override
    public void runOpMode() throws InterruptedException {
        leftFront = hardwareMap.get(DcMotorEx.class, "leftFront");
        leftRear = hardwareMap.get(DcMotorEx.class, "leftRear");
        rightRear = hardwareMap.get(DcMotorEx.class, "rightRear");
        rightFront = hardwareMap.get(DcMotorEx.class, "rightFront");
        LEDs = hardwareMap.get(DcMotorEx.class, "LEDs");
        //SlidesAngle = hardwareMap.get(DcMotorEx.class, "SlidesAngle");
        LSlides = hardwareMap.get(DcMotorEx.class, "LSlides");
        Bin = hardwareMap.get(Servo.class, "Bin");
        //Claw = hardwareMap.get(Servo.class, "Claw");
        Wheel = hardwareMap.get(DcMotorEx.class, "Wheel");
        Intake = hardwareMap.get(DcMotorEx.class, "Intake");
        Wheel.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        LSlides.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        //UpTurret = hardwareMap.get(Servo.class, "UpTurret");
        //HorizontallyTurret = hardwareMap.get(Servo.class, "horizontalTurret");
        //initTurret = hardwareMap.get(Servo.class, "susControl");
        //extendTurret = hardwareMap.get(CRServo.class, "extendTurret");



        motors = Arrays.asList(leftFront, leftRear, rightRear, rightFront, LSlides, Wheel, Intake, LEDs);

        LEDs.setDirection(DcMotorSimple.Direction.REVERSE);
        leftFront.setDirection(DcMotorSimple.Direction.REVERSE);
        rightRear.setDirection(DcMotorSimple.Direction.REVERSE);
        rightFront.setDirection(DcMotorSimple.Direction.REVERSE);
        //LSlides.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        // LSlides.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        //LSlides.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);

        for (DcMotorEx motor : motors) {
            MotorConfigurationType motorConfigurationType = motor.getMotorType().clone();
            motorConfigurationType.setAchieveableMaxRPMFraction(1.0);
            motor.setMotorType(motorConfigurationType);
        }
//Bin start position - 0.4 is too low and cause problems coming back in, 0.5 cause issues intaking sometimes
        Bin.setPosition(0.5);
        //Claw.setPosition(0);

        waitForStart();
        //moves turret up
        //initTurret.setPosition(1);
        while (opModeIsActive()) {
            // Variable setup
            double fortuneIII;
            double leftPower;
            double rightPower;
            //double slidesAngPower;
            double LSlidesPower = 0.0;
            int LSlidesRotation;
            double wheelPower;

            double upRange = 0.49;
            double downRange = 1.0;


            //Telemetry
            LSlidesRotation = LSlides.getCurrentPosition();

            telemetry.addData("Slides Position: ", LSlidesRotation);
            telemetry.update();

            //Foundation motion
            leftPower = gamepad1.left_stick_y * -1;
            rightPower = gamepad1.right_stick_y * -1;
            leftPower *= (int) 10 * -1;
            leftPower /= 10;

            rightPower *= (int) 10 * -1;
            rightPower /= 10;

            //Linear Slides angular motion
            //slidesAngPower = gamepad2.right_stick_y * 0.6;

            //Linear slides encoder tracker
            //if (LSlidesRotation < 5040){
            LSlidesPower = gamepad2.left_stick_y;
            //}else{
            //LSlides.setPower(-0.1);
            //}

            //wheelPower = gamepad1.right_trigger * -1;
            //Wheel motion at different speeds
            while(gamepad1.left_trigger > 0.0) {
                Wheel.setPower(-0.8);
                //LEDs.setPower(Math.abs(Wheel.getPower()));
            }
            while(gamepad1.right_trigger > 0.0) {
                Wheel.setPower(gamepad1.right_trigger * -1);
                //LEDs.setPower(Math.abs(Wheel.getPower()));
            }
            Wheel.setPower(0);

            while (gamepad1.dpad_left) {
                Wheel.setPower(0.8);
                LEDs.setPower(Math.abs(Wheel.getPower()));
            }
            while (gamepad1.dpad_right) {
                Wheel.setPower(0.8);
                LEDs.setPower(Math.abs(Wheel.getPower()));
            }
            Wheel.setPower(0);

            //Bin up and down
            if (gamepad2.right_bumper) {

                Bin.setPosition(upRange);
                LEDs.setPower(upRange);

            } else if (gamepad2.left_bumper) {
                Bin.setPosition(downRange);
                LEDs.setPower(downRange);
            } else if (gamepad2.dpad_left) {
                Bin.setPosition(0.4);

            }
            //claw movement
            /*
            if (gamepad2.triangle) {
                Claw.setPosition(0);
            } else if (gamepad2.cross) {
                Claw.setPosition(1);
            }
            */
            // Straight line motion with Dpad
            if (gamepad1.dpad_down) {
                leftFront.setPower(1);
                leftRear.setPower(1);
                rightFront.setPower(1);
                rightRear.setPower(1);
                LEDs.setPower(1);
            } else if (gamepad1.dpad_up) {
                leftFront.setPower(-1);
                leftRear.setPower(-1);
                rightFront.setPower(-1);
                rightRear.setPower(-1);
                LEDs.setPower(1);
            } else {
                leftFront.setPower(0);
                leftRear.setPower(0);
                rightFront.setPower(0);
                rightRear.setPower(0);
                LEDs.setPower(0);
            }
            //Intake
            if (gamepad2.right_trigger > 0.0) {
                Intake.setPower(gamepad2.right_trigger);
                LEDs.setPower(1);
                Bin.setPosition(0.48);
            } else if (gamepad2.left_trigger > 0.0) {
                Intake.setPower(gamepad2.left_trigger * -1);
                LEDs.setPower(1);
            } else {
                Intake.setPower(0);
                LEDs.setPower(0);
                //Bin.setPosition(0.49);
            }
            //Motor power setup
            leftFront.setPower(leftPower);
            leftRear.setPower(leftPower);
            rightFront.setPower(rightPower);
            rightRear.setPower(rightPower);

            //SlidesAngle.setPower(slidesAngPower);
            LSlides.setPower(LSlidesPower);
            //LEDs.setPower(Math.abs(LSlidesPower));
            //Wheel.setPower(wheelPower);

            //Turret


            //Wheel exponential


            if (gamepad1.square || gamepad1.circle) {
                ElapsedTime wheelRun = new ElapsedTime(0);
                wheelRun.reset();
                while (wheelRun.time() < 2.25) {
                    fortuneIII = wheelRun.time() / 6;

                    fortuneIII *= -1;
                    Wheel.setPower(fortuneIII);
                }


            } else {
                Wheel.setPower(0);
            }

            if (gamepad1.cross) {
                ElapsedTime wheelRun = new ElapsedTime(0);
                wheelRun.reset();
                while (wheelRun.time() < 3) {
                    fortuneIII = wheelRun.time() / 6;


                    Wheel.setPower(fortuneIII);
                }


            } else {
                Wheel.setPower(0);
            }

            //Strafe while loop
            while (gamepad1.left_bumper) {

                // if(LSlidesPower > 0.0){
                //   Bin.setPosition(-1);
                //   }

                //Linear slides encoder tracker
                if (LSlidesRotation < 5040) {
                    LSlidesPower = gamepad2.left_stick_y;
                } else {
                    LSlides.setPower(-0.1);
                }

                //Intake
                if (gamepad2.right_trigger > 0.0) {
                    Intake.setPower(gamepad2.right_trigger);
                } else if (gamepad2.left_trigger > 0.0) {
                    Intake.setPower(gamepad2.left_trigger * -1);
                } else {
                    Intake.setPower(0);
                }


                //wheelPower = gamepad1.right_trigger * -1;

                //Strafe speed
                leftPower = gamepad1.right_stick_x;

                //Linear Slides angular motion
                //slidesAngPower = gamepad2.right_stick_y;
                //Linear slides outward motion
                LSlidesPower = gamepad2.left_stick_y;

                //Strafe power setup
                leftFront.setPower(leftPower * -1);
                leftRear.setPower(leftPower);
                rightFront.setPower(leftPower);
                rightRear.setPower(leftPower * -1);

                //SlidesAngle.setPower(slidesAngPower);
                LSlides.setPower(LSlidesPower);
                //Wheel.setPower(wheelPower);


            }
            //strafed curve?
            while (gamepad1.right_bumper) {
                cStrafe(gamepad1.right_stick_x);
            }

        }
        //All motors at 0 power at start
        leftFront.setPower(0);
        leftRear.setPower(0);
        rightFront.setPower(0);
        rightRear.setPower(0);
        Wheel.setPower(0);
        LSlides.setPower(0);
        Intake.setPower(0);
        LEDs.setPower(0);
    }




    public void cStrafe(double p){
        if(p > 0){
            rightRear.setPower(p-p*0.8);
            leftFront.setPower(p);
            leftRear.setPower(p*-1);
            rightFront.setPower(p*-1);
            LEDs.setPower(1);
        }
        else if(p < 0){
            leftFront.setPower(p);
            leftRear.setPower(p*-1);
            rightFront.setPower(p*-1);
            rightRear.setPower(p - p*0.8);
            LEDs.setPower(1);
        }


    }



}