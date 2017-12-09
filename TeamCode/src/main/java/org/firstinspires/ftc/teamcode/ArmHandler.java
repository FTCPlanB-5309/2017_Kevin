package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.hardware.DcMotor;

import org.firstinspires.ftc.robotcore.external.Telemetry;

/**
 * Created by user on 10/17/2017.
 */

public class ArmHandler {
    RobotHardware robot;
    Telemetry telemetry;


    public ArmHandler(RobotHardware robot, Telemetry telemetry) {
        this.robot = robot;
        this.telemetry = telemetry;
    }

public void armToPosition (int pos) throws InterruptedException {
        robot.armMotor.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        robot.armMotor.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        robot.armMotor.setTargetPosition(pos);
        robot.armMotor.setPower(0.5);
        while (robot.armMotor.isBusy()) {
                Thread.yield();
        }
        robot.armMotor.setPower(0.0);
        }
}