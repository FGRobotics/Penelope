package org.firstinspires.ftc.teamcode.drive.opmode;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.DistanceSensor;

import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit;
@Autonomous(group="Drive")
public class DistTest extends LinearOpMode {
    DistanceSensor distance, wheelSensor;
    private DcMotorEx LSlides;

    public void runOpMode() throws InterruptedException {
        distance = hardwareMap.get(DistanceSensor.class, "toaster");
        distance.getDistance(DistanceUnit.INCH);
        wheelSensor = hardwareMap.get(DistanceSensor.class, "wheelSensor");
        wheelSensor.getDistance(DistanceUnit.INCH);
        LSlides = hardwareMap.get(DcMotorEx.class, "LSlides");
        LSlides.setDirection(DcMotorSimple.Direction.REVERSE);
        int targetPos = -1000;
        waitForStart();
        LSlides.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        while(opModeIsActive()) {

            telemetry.addData("Turn Value Wheel", wheelSensor.getDistance(DistanceUnit.INCH));
            telemetry.addData("Slides distance", distance.getDistance(DistanceUnit.INCH));
            telemetry.addData("targetPosition", targetPos);
            telemetry.update();
            if (gamepad1.dpad_up){
                targetPos = targetPos - 50;
            }
            if (gamepad1.dpad_left){
                targetPos = targetPos + 10;
            }
            while (gamepad1.square){
                LSlides.setPower(-0.8);
                LSlides.setTargetPosition(0);
                if(LSlides.isBusy()){
                    idle();
                }
            }
            while (gamepad1.cross) {
                LSlides.setPower(-0.8);
                LSlides.setTargetPosition(targetPos);
                if(LSlides.isBusy()){
                    idle();
                }
            }
            telemetry.update();

        }
    }



}
