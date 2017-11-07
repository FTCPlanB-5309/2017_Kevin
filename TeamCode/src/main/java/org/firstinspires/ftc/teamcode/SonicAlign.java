package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.hardware.DcMotor;

import org.firstinspires.ftc.robotcore.external.Telemetry;
import org.firstinspires.ftc.robotcore.external.navigation.RelicRecoveryVuMark;

/**
 * Created by user on 10/17/2017.
 */

public class SonicAlign {
    RobotHardware robot;
    Telemetry telemetry;
    final static int ULTRASONICTHRESHHOLD = 3;


    public SonicAlign(RobotHardware robot, Telemetry telemetry) {
        this.robot = robot;
        this.telemetry = telemetry;
    }
    public void run(RelicRecoveryVuMark columnPosition){
        double baseline = robot.sonicOne.cmUltrasonic();
        if (columnPosition == RelicRecoveryVuMark.LEFT){
            robot.centerWheel.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
            robot.centerWheel.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
            robot.centerWheel.setPower(-0.07);
            while (robot.sonicOne.cmUltrasonic() > (baseline - ULTRASONICTHRESHHOLD) && robot.centerWheel.getCurrentPosition() < 5000) {
                telemetry.addData("ultrasonic reading :", robot.sonicOne.cmUltrasonic());
                telemetry.update();
            }
        }
    }
}