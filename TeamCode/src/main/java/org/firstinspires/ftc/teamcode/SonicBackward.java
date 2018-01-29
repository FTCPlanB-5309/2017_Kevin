package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.util.Range;

import org.firstinspires.ftc.robotcore.external.Telemetry;
import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit;

/**
 * Created by user on 12/28/2017.
 */

public class SonicBackward {
    RobotHardware robot;
    Telemetry telemetry;

    public SonicBackward(RobotHardware robot, Telemetry telemetry) {
        this.robot = robot;
        this.telemetry = telemetry;
    }

    public void sonic (double cm) {
        robot.leftWheel.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        robot.rightWheel.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        robot.leftWheel.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        robot.rightWheel.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        robot.leftWheel.setPower(robot.PLATFORM_SPEED);
        robot.rightWheel.setPower(robot.PLATFORM_SPEED);
        final int maxDistance = (int)(-36*robot.COUNTS_PER_INCH);

        while(robot.sonicOne.cmUltrasonic() < cm && robot.leftWheel.getCurrentPosition() > maxDistance){
            telemetry.addData("Sonic cm", robot.sonicOne.cmUltrasonic());
            telemetry.addData("Left Encoder", robot.leftWheel.getCurrentPosition());
            telemetry.update();
        }

        robot.leftWheel.setPower(0);
        robot.rightWheel.setPower(0);
        robot.leftWheel.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        robot.rightWheel.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
    }
}
