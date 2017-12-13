package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.hardware.DcMotor;

import org.firstinspires.ftc.robotcore.external.Telemetry;

/**
 * Created by user on 10/17/2017.
 */

public class Jewel {
    RobotHardware robot;
    Telemetry telemetry;


    public Jewel (RobotHardware robot, Telemetry telemetry) {
        this.robot = robot;
        this.telemetry = telemetry;
//        this.robot.jewelArmMotor.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
    }

    public void moveArm (int destination) {
        robot.jewelArmMotor.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        robot.jewelArmMotor.setTargetPosition(destination);
        robot.jewelArmMotor.setPower(0.25);
        while (robot.jewelArmMotor.isBusy()) {
            Thread.yield();
        }
    }

    public void JewelSwatter (int allianceColor) throws InterruptedException {
        int blueValue = 0;
        int redValue = 0;
        robot.jewelServo.setPosition(robot.JEWEL_SERVO_MIDDLE);
        Thread.sleep(500);
        moveArm(300);
        Thread.sleep(250);
        blueValue = robot.JewelColorSensor.blue();
        redValue = robot.JewelColorSensor.red();
        moveArm(325);
        Thread.sleep(250);
        if (robot.JewelColorSensor.blue() > blueValue)
            blueValue = robot.JewelColorSensor.blue();
        if (robot.JewelColorSensor.red() > redValue)
            redValue = robot.JewelColorSensor.red();
        Thread.sleep(500);
        if (robot.JewelColorSensor.blue() > blueValue)
            blueValue = robot.JewelColorSensor.blue();
        if (robot.JewelColorSensor.red() > redValue)
            redValue = robot.JewelColorSensor.red();
        if (allianceColor == robot.BLUE) {
            if (blueValue > redValue)
                robot.jewelServo.setPosition(robot.JEWEL_SERVO_RIGHT);
            if (blueValue < redValue)
                robot.jewelServo.setPosition(robot.JEWEL_SERVO_LEFT);
        }
        else {
            if (blueValue > redValue)
                robot.jewelServo.setPosition(robot.JEWEL_SERVO_LEFT);
            if (blueValue < redValue)
                robot.jewelServo.setPosition(robot.JEWEL_SERVO_RIGHT);
        }
        telemetry.addData("Blue Value", blueValue);
        telemetry.addData("Red Value", redValue);
        Thread.sleep(500);
        moveArm(0);
        robot.jewelServo.setPosition(robot.JEWEL_SERVO_LEFT);

        }
}