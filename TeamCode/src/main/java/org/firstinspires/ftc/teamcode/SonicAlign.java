package org.firstinspires.ftc.teamcode;

import android.transition.*;

import com.qualcomm.robotcore.hardware.DcMotor;

import org.firstinspires.ftc.robotcore.external.Telemetry;
import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit;
import org.firstinspires.ftc.robotcore.external.navigation.RelicRecoveryVuMark;


public class SonicAlign {
    RobotHardware robot;
    Telemetry telemetry;
    Slide slide;
    final static int ULTRASONICTHRESHHOLD = 3;


    public SonicAlign(RobotHardware robot, Telemetry telemetry, Slide slide) {
        this.robot = robot;
        this.telemetry = telemetry;
        this.slide = slide;
    }
    public double run(RelicRecoveryVuMark columnPosition) throws InterruptedException{
        double baseline;
        baseline = findColumn();
        if (columnPosition == RelicRecoveryVuMark.CENTER){
            slide.run(0.07, 3, robot.RIGHT);
            baseline = findColumn();
        }
        if (columnPosition == RelicRecoveryVuMark.RIGHT){
            slide.run(0.07, 10, robot.RIGHT);
            baseline = findColumn();
        }
        Thread.sleep(500);
        return baseline - 6;
    }

    double findColumn  () throws InterruptedException {
        double baseline = robot.sonicOne.cmUltrasonic();
        double baselineInches = robot.sonicOne.getDistance(DistanceUnit.INCH);
        robot.centerWheel.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        robot.centerWheel.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        robot.centerWheel.setPower(-0.07);
        while (robot.sonicOne.cmUltrasonic() > (baseline - ULTRASONICTHRESHHOLD) && robot.centerWheel.getCurrentPosition() < 5000) {
            telemetry.addData("ultrasonic reading :", robot.sonicOne.cmUltrasonic());
            telemetry.addData("center wheel :", robot.centerWheel.getCurrentPosition());
            telemetry.update();

        }
        robot.centerWheel.setPower(0);
        Thread.sleep(500);
        return baselineInches;
    }
}