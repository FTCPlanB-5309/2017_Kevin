package org.firstinspires.ftc.teamcode;

import org.firstinspires.ftc.robotcore.external.Telemetry;

/**
 * Created by user on 10/17/2017.
 */

public class Jewel {
    RobotHardware robot;
    Telemetry telemetry;

    final double ARM_SERVO_UP = 0.42;
    final double ARM_SERVO_DOWN = 0.05;
    final double JEWEL_SERVO_MIDDLE = 0.22;
    final double JEWEL_SERVO_LEFT = 0;
    final double JEWEL_SERVO_RIGHT = 1;

    public Jewel (RobotHardware robot, Telemetry telemetry) {
        this.robot = robot;
        this.telemetry = telemetry;
    }

public void JewelSwatter (int allianceColor) throws InterruptedException {
        int blueValue = 0;
        int redValue = 0;
        robot.jewelServo.setPosition(JEWEL_SERVO_MIDDLE);
        Thread.sleep(3000);
        robot.armServo.setPosition(ARM_SERVO_DOWN);
        Thread.sleep(3000);
        blueValue = robot.ColorSensor.blue();
        redValue = robot.ColorSensor.red();
        robot.armServo.setPosition(robot.armServo.getPosition() - 0.01);
        Thread.sleep(500);
        if (robot.ColorSensor.blue() > blueValue)
        blueValue = robot.ColorSensor.blue();
        if (robot.ColorSensor.red() > redValue)
        redValue = robot.ColorSensor.red();
        robot.armServo.setPosition(robot.armServo.getPosition() - 0.01);
        Thread.sleep(500);
        if (robot.ColorSensor.blue() > blueValue)
        blueValue = robot.ColorSensor.blue();
        if (robot.ColorSensor.red() > redValue)
        redValue = robot.ColorSensor.red();
        if (allianceColor == robot.BLUE)
        {
        if (blueValue > redValue)
        robot.jewelServo.setPosition(JEWEL_SERVO_RIGHT);
        if (blueValue < redValue)
            robot.jewelServo.setPosition(JEWEL_SERVO_LEFT);
        }
        else {
        if (blueValue > redValue)
            robot.jewelServo.setPosition(JEWEL_SERVO_LEFT);
        if (blueValue < redValue)
        robot.jewelServo.setPosition(JEWEL_SERVO_RIGHT);
        }
        telemetry.addData("Blue Value", blueValue);
        telemetry.addData("Red Value", redValue);
        Thread.sleep(500);
        robot.armServo.setPosition(ARM_SERVO_UP);
        Thread.sleep(1000);

        }
}