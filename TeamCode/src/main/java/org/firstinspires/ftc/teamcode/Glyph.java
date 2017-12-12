package org.firstinspires.ftc.teamcode;

import org.firstinspires.ftc.robotcore.external.Telemetry;

/**
 * Created by user on 10/17/2017.
 */

public class Glyph {
    RobotHardware robot;
    Telemetry telemetry;


    public Glyph(RobotHardware robot, Telemetry telemetry) {
        this.robot = robot;
        this.telemetry = telemetry;
    }

    public void grabber(double pos){
        if (pos == robot.OPEN){
            robot.leftClaw.setPosition(robot.LEFT_CLAW_OPEN);
            robot.rightClaw.setPosition(robot.RIGHT_CLAW_OPEN);
            robot.upperLeftClaw.setPosition(robot.UPPER_LEFT_CLAW_OPEN);
            robot.upperRightClaw.setPosition(robot.UPPER_RIGHT_CLAW_OPEN);
        }
        if (pos == robot.CLOSE){
            robot.leftClaw.setPosition(robot.LEFT_CLAW_CLOSED);
            robot.rightClaw.setPosition(robot.RIGHT_CLAW_CLOSED);
            robot.upperLeftClaw.setPosition(robot.UPPER_LEFT_CLAW_CLOSED);
            robot.upperRightClaw.setPosition(robot.UPPER_RIGHT_CLAW_CLOSED);
        }
        if (pos == robot.SOFT){
            robot.leftClaw.setPosition(robot.LEFT_CLAW_SOFT);
            robot.rightClaw.setPosition(robot.RIGHT_CLAW_SOFT);
            robot.upperLeftClaw.setPosition(robot.UPPER_LEFT_CLAW_SOFT);
            robot.upperRightClaw.setPosition(robot.UPPER_RIGHT_CLAW_SOFT);
        }
    }
}