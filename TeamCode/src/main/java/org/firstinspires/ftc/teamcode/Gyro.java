package org.firstinspires.ftc.teamcode;
import org.firstinspires.ftc.robotcore.external.Telemetry;


public class Gyro {
    RobotHardware robot;
    Telemetry telemetry;

    enum quadrantCode {
        Q1toQ4,
        Q4toQ1,
        QOther
    }

    final String Q4TOQ1 = "Q1 to Q4";
    //final int Q4TOQ1 = 123;
    final int Q1TOQ4 = 456;
    final int OTHERQ = 789;
    final int LEFT = 987;
    final int RIGHT = 654;

    public Gyro (RobotHardware robot, Telemetry telemetry) {
        this.robot = robot;
        this.telemetry = telemetry;
    }

    public void turn(int target) {
        quadrantCode qCode;
        int currentHeading;
        int direction;
        int speed;
        currentHeading = robot.gyroSensor.getHeading();
        while (currentHeading != target) {
            qCode = getQuadrant(target, currentHeading);
            direction = getDirection(target, qCode, currentHeading);
            speed = getSpeed(target, currentHeading, qCode);
            if (direction == LEFT) {
                robot.rightWheel.setPower(speed);
                robot.leftWheel.setPower(-speed);
            } else {
                robot.rightWheel.setPower(-speed);
                robot.leftWheel.setPower(speed);
            }
            telemetry.addData("Quadrant", qCode);
            telemetry.addData("target",target);
            telemetry.addData("speed",speed);
            telemetry.addData("direction",direction);
            telemetry.addData("currentHeading",currentHeading);
        }
        robot.rightWheel.setPower(0);
        robot.leftWheel.setPower(0);
    }

     quadrantCode getQuadrant (int target, int currentHeading) {
        quadrantCode qCode;
        if (currentHeading >= 270 && target < 90) {
            qCode = quadrantCode.Q4toQ1;
        }
        else if (currentHeading < 90 && target >= 270) {
            qCode = quadrantCode.Q1toQ4;
        } else qCode = quadrantCode.QOther;
        return qCode;
    }

    int getDirection (int target, quadrantCode qCode, int currentHeading) {
        int direction;
        if (qCode == quadrantCode.Q1toQ4) {
            direction = LEFT;
        }
        else if (qCode == quadrantCode.Q4toQ1) {
            direction = RIGHT;
        }
        else {
            if (currentHeading < target) {
                if (currentHeading - target < 180) {
                    direction = LEFT;
                }
                else direction = RIGHT;
            }
            else {
                if (target - currentHeading < 180) {
                    direction = RIGHT;
                }
                else direction = LEFT;
            }
        }
        return  direction;
    }


    int getSpeed (int target, int currentHeading, quadrantCode qCode) {
        int distance;
        int speed;
        if (qCode == quadrantCode.Q1toQ4) {
            distance = currentHeading + (359 - target);
        }
        else if (qCode == quadrantCode.Q4toQ1) {
            distance = target + (359 - currentHeading);
        }
        else {
            distance = Math.abs(currentHeading - target);
            if (distance > 180) {
                distance = 360 - distance;
            }
        }

        if (distance >= 100) {
            speed = 1;
        }
        else if (distance > 10 && distance < 100) {
            speed = distance/100;
        }
        else  {
            speed = distance;
        }

        return  speed;
        }
    }



