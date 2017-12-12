package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.hardware.DcMotor;

import org.firstinspires.ftc.robotcore.external.Telemetry;

/**
 * Created by user on 10/17/2017.
 */

public class Relic {
    RobotHardware robot;
    Telemetry telemetry;


    public Relic(RobotHardware robot, Telemetry telemetry) {
        this.robot = robot;
        this.telemetry = telemetry;
    }

    public void wrist(double speed){
        robot.wristRelic.setPosition(robot.wristRelic.getPosition() + speed);
        if(robot.wristRelic.getPosition() < 0)
            robot.wristRelic.setPosition(0);
        if(robot.wristRelic.getPosition() > 1)
            robot.wristRelic.setPosition(1);
    }
    public void extend(double dir){
        robot.extensionRelic.setPosition(dir);
        double rDir = 0.5;
        if(dir == 0)
            rDir = 1;
        if(dir == 1)
            rDir = 0;
        robot.ExtensionReversed.setPosition(rDir);
    }
    public void claw(double pos){
        if(pos==robot.OPEN){
            robot.leftRelic.setPosition(robot.LEFT_RELIC_OPEN);
            robot.rightRelic.setPosition(robot.RIGHT_RELIC_OPEN);
        }
        if(pos==robot.CLOSE){
            robot.leftRelic.setPosition(robot.LEFT_RELIC_CLOSED);
            robot.rightRelic.setPosition(robot.RIGHT_RELIC_CLOSED);
        }
    }

}