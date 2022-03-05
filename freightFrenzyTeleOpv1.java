package org.firstinspires.ftc.teamcode.drive.opmode;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.CRServo;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.hardware.configuration.typecontainers.MotorConfigurationType;
import com.qualcomm.robotcore.util.ElapsedTime;

import java.util.Arrays;
import java.util.List;

@TeleOp(name="FrenzyTelev1")
public class freightFrenzyTeleOpv1 extends LinearOpMode {
    private DcMotorEx leftFront, leftRear, rightRear, rightFront, LSlides, Wheel, Intake, LEDs;
    private Servo Bin, UpTurret;
    private CRServo extendTurret, HorizontallyTurret;


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
        UpTurret = hardwareMap.get(Servo.class, "tilt");
        HorizontallyTurret = hardwareMap.get(CRServo.class, "pan");

        extendTurret = hardwareMap.get(CRServo.class, "extend");



        motors = Arrays.asList(leftFront, leftRear, rightRear, rightFront, LSlides, Wheel, Intake, LEDs);
        //LSlides.setDirection(DcMotorSimple.Direction.REVERSE);
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
                HorizontallyTurret.setPower(gamepad2.right_stick_x * -0.5);

                extendTurret.setPower(gamepad2.left_stick_x);

                if(gamepad2.cross) {
                    UpTurret.setPosition(UpTurret.getPosition() - 0.005);
                }else if(gamepad2.triangle){
                    UpTurret.setPosition(UpTurret.getPosition() + 0.005);
                }
            }
            while(gamepad1.right_trigger > 0.0) {
                Wheel.setPower(gamepad1.right_trigger * -1);
                //LEDs.setPower(Math.abs(Wheel.getPower()));
                HorizontallyTurret.setPower(gamepad2.right_stick_x * -0.5);

                extendTurret.setPower(gamepad2.left_stick_x);

                if(gamepad2.cross) {
                    UpTurret.setPosition(UpTurret.getPosition() - 0.005);
                }else if(gamepad2.triangle){
                    UpTurret.setPosition(UpTurret.getPosition() + 0.005);
                }
            }
            Wheel.setPower(0);

            while (gamepad1.dpad_left) {
                Wheel.setPower(0.8);
                LEDs.setPower(Math.abs(Wheel.getPower()));
                HorizontallyTurret.setPower(gamepad2.right_stick_x * -0.5);

                extendTurret.setPower(gamepad2.left_stick_x);

                if(gamepad2.cross) {
                    UpTurret.setPosition(UpTurret.getPosition() - 0.005);
                }else if(gamepad2.triangle){
                    UpTurret.setPosition(UpTurret.getPosition() + 0.005);
                }
            }
            while (gamepad1.dpad_right) {
                Wheel.setPower(0.8);
                LEDs.setPower(Math.abs(Wheel.getPower()));
                HorizontallyTurret.setPower(gamepad2.right_stick_x * -0.5);

                extendTurret.setPower(gamepad2.left_stick_x);

                if(gamepad2.cross) {
                    UpTurret.setPosition(UpTurret.getPosition() - 0.005);
                }else if(gamepad2.triangle){
                    UpTurret.setPosition(UpTurret.getPosition() + 0.005);
                }
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

            while(gamepad2.right_stick_y > 0.5) {
                UpTurret.setPosition(UpTurret.getPosition() + -0.1*0.005);
            }
            while(gamepad2.right_stick_y < -0.5){
                UpTurret.setPosition(UpTurret.getPosition() + 0.1*0.005);
            }



            //turret

            HorizontallyTurret.setPower(gamepad2.right_stick_x * -0.18);



            extendTurret.setPower(gamepad2.left_stick_x);

            if(gamepad2.cross) {
                UpTurret.setPosition(UpTurret.getPosition() - 0.0005);
            }else if(gamepad2.triangle){
                UpTurret.setPosition(UpTurret.getPosition() + 0.0005);
            }



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