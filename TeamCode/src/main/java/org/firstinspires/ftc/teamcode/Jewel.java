package org.firstinspires.ftc.teamcode;

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
    }

public void JewelSwatter (int allianceColor) throws InterruptedException {
        int blueValue = 0;
        int redValue = 0;
        robot.jewelServo.setPosition(robot.JEWEL_SERVO_MIDDLE);
        Thread.sleep(1000);
        robot.armServo.setPosition(robot.ARM_SERVO_DOWN);
        Thread.sleep(1000);
        blueValue = robot.JewelColorSensor.blue();
        redValue = robot.JewelColorSensor.red();
        robot.armServo.setPosition(robot.armServo.getPosition() - 0.01);
        Thread.sleep(500);
        if (robot.JewelColorSensor.blue() > blueValue)
        blueValue = robot.JewelColorSensor.blue();
        if (robot.JewelColorSensor.red() > redValue)
        redValue = robot.JewelColorSensor.red();
        robot.armServo.setPosition(robot.armServo.getPosition() - 0.01);
        Thread.sleep(500);
        if (robot.JewelColorSensor.blue() > blueValue)
        blueValue = robot.JewelColorSensor.blue();
        if (robot.JewelColorSensor.red() > redValue)
        redValue = robot.JewelColorSensor.red();
        if (allianceColor == robot.BLUE)
        {
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
        robot.armServo.setPosition(robot.ARM_SERVO_UP);
        Thread.sleep(1000);
        robot.jewelServo.setPosition(robot.JEWEL_SERVO_LEFT);

        }
}