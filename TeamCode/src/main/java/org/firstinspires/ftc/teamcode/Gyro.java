package org.firstinspires.ftc.teamcode;
import com.qualcomm.robotcore.hardware.DcMotor;

import org.firstinspires.ftc.robotcore.external.Telemetry;


public class Gyro {
    RobotHardware robot;
    Telemetry telemetry;

    double speed;

    enum quadrantCode {
        Q1toQ4,
        Q4toQ1,
        QOther
    }

    enum directionCode {
        Left,
        Right
    }

    public Gyro (RobotHardware robot, Telemetry telemetry) {
        this.robot = robot;
        this.telemetry = telemetry;
    }

    public void turn(int target) {
        robot.leftWheel.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        robot.rightWheel.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        quadrantCode qCode;
        int currentHeading;
        directionCode direction;
        currentHeading = robot.gyroSensor.getHeading();
        robot.leftWheel.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        robot.rightWheel.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        while (currentHeading != target) {
            currentHeading = robot.gyroSensor.getHeading();
            qCode = getQuadrant(target, currentHeading);
            direction = getDirection(target, qCode, currentHeading);
            speed = getSpeed(target, currentHeading, qCode);
            if (direction == direction.Left) {
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
            telemetry.update();
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

    directionCode getDirection (int target, quadrantCode qCode, int currentHeading) {
        directionCode direction;
        if (qCode == quadrantCode.Q1toQ4) {
            direction = directionCode.Right;
        }
        else if (qCode == quadrantCode.Q4toQ1) {
            direction = directionCode.Left;
        }
        else {
            if (currentHeading < target) {
                if (target - currentHeading < 180) {
                    direction = directionCode.Left;
                }
                else direction = directionCode.Right;
            }
            else {
                if (currentHeading - target < 180) {
                    direction = directionCode.Right;
                }
                else direction = directionCode.Left;
            }
        }
        return  direction;
    }


    double getSpeed (int target, int currentHeading, quadrantCode qCode) {
        int distance;
        if (qCode == quadrantCode.Q4toQ1) {
            distance = (target + (360 - currentHeading));
            if (distance > 45)
                return 0.5;
            else
                return 0.07;
        }

        if (qCode == quadrantCode.Q1toQ4){
            distance = (currentHeading + (360 - target));
            if (distance > 45)
                return 0.5;
            else
                return 0.07;
        }

        if (qCode == quadrantCode.QOther) {
            if (Math.abs(currentHeading - target) > 45)
                return 0.5;
            else
                return 0.07;
        }
        return  0.07;
        }
    }



