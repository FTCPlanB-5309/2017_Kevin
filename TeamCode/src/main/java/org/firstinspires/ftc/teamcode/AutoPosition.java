package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.util.Range;
import org.firstinspires.ftc.robotcore.external.Telemetry;

/**
 * Created by user on 10/17/2017.
 */

public class AutoPosition {
    RobotHardware robot;
    Telemetry telemetry;
    Forward forward = new Forward(robot, telemetry);
    Slide slide = new Slide(robot, telemetry);
    Gyro gyro = new Gyro(robot, telemetry);
    SonicAlign sonicAlign = new SonicAlign(robot, telemetry, slide);
    ArmHandler armHandler = new ArmHandler(robot,telemetry);

    public AutoPosition(RobotHardware robot, Telemetry telemetry) {
        this.robot = robot;
        this.telemetry = telemetry;
    }

    public boolean moveToWall(double in){
        robot.leftWheel.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        robot.rightWheel.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        robot.leftWheel.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        robot.rightWheel.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        robot.leftWheel.setPower(robot.PLATFORM_SPEED);
        robot.rightWheel.setPower(robot.PLATFORM_SPEED);
        int heading = robot.gyroSensor.getHeading();
        double offset = 0;
        while(robot.sonicOne.cmUltrasonic() > in*2.54) {
            heading = robot.gyroSensor.getHeading();
            if (heading < 90) {
                offset = robot.MOVEMENT_INCREMENT * heading;
                robot.leftWheel.setPower(Range.clip(robot.PLATFORM_SPEED + offset, 0, 0.3));
                robot.rightWheel.setPower(Range.clip(robot.PLATFORM_SPEED - offset, 0, 0.3));
            } else if (heading > 270) {
                offset = robot.MOVEMENT_INCREMENT * (360 - heading);
                robot.leftWheel.setPower(Range.clip(robot.PLATFORM_SPEED - offset, 0, 0.3));
                robot.rightWheel.setPower(Range.clip(robot.PLATFORM_SPEED + offset, 0, 0.3));
            } else {
                return false;
            }
            telemetry.addData("offset", offset);
            telemetry.addData("heading", heading);
            telemetry.addData("left", robot.leftWheel.getPower());
            telemetry.addData("right", robot.rightWheel.getPower());
            telemetry.update();
        }
        robot.leftWheel.setPower(0);
        robot.rightWheel.setPower(0);
        return true;
    }


    public void center(int color, int pic){
        if(color==robot.BLUE) {
            robot.centerWheel.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
            robot.centerWheel.setMode(DcMotor.RunMode.RUN_TO_POSITION);
            int target = (int) (-36 * robot.COUNTS_PER_INCH);
            robot.centerWheel.setTargetPosition(target);
            robot.centerWheel.setPower(-0.1);
            while (robot.centerWheel.isBusy() && robot.FloorColorSensor.blue() <= 1) {
                telemetry.addData("Blue value: ", "%d", robot.FloorColorSensor.blue());
                telemetry.addData("Red value: ", "%d", robot.FloorColorSensor.red());
                telemetry.addData("Center wheel value: ", "%d", robot.centerWheel.getCurrentPosition());
                telemetry.update();
            }
            while (robot.centerWheel.isBusy() && robot.FloorColorSensor.blue() > 1) {
                telemetry.addData("Blue value: ", "%d", robot.FloorColorSensor.blue());
                telemetry.addData("Red value: ", "%d", robot.FloorColorSensor.red());
                telemetry.addData("Center wheel value: ", "%d", robot.centerWheel.getCurrentPosition());
                telemetry.update();
            }
            int blueline1 = robot.centerWheel.getCurrentPosition();
            while (robot.centerWheel.isBusy() && robot.FloorColorSensor.blue() <= 1) {
                telemetry.addData("Blue value: ", "%d", robot.FloorColorSensor.blue());
                telemetry.addData("Red value: ", "%d", robot.FloorColorSensor.red());
                telemetry.addData("Center wheel value: ", "%d", robot.centerWheel.getCurrentPosition());
                telemetry.update();
            }
            int blueline2 = robot.centerWheel.getCurrentPosition();

            //Going to the center of the safe zone
            int safezonecenter = (blueline1 + blueline2) / 2;
            if (pic == robot.CENTER){
                target = (int) (safezonecenter+7* robot.COUNTS_PER_INCH);
                robot.centerWheel.setTargetPosition(target);
            }
            else if(pic == robot.RIGHT){
                target = (int) (safezonecenter-1* robot.COUNTS_PER_INCH);
                robot.centerWheel.setTargetPosition(target);
            }
            else{
                target = (int) (safezonecenter+15* robot.COUNTS_PER_INCH);
                robot.centerWheel.setTargetPosition(target);
            }
            robot.centerWheel.setTargetPosition(target);
            while (robot.centerWheel.isBusy()) {
                telemetry.addData("Blue value: ", "%d", robot.FloorColorSensor.blue());
                telemetry.addData("Red value: ", "%d", robot.FloorColorSensor.red());
                telemetry.addData("Center wheel value: ", "%d", robot.centerWheel.getCurrentPosition());
                telemetry.update();
            }
        }
        if(color==robot.RED) {
            robot.centerWheel.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
            robot.centerWheel.setMode(DcMotor.RunMode.RUN_TO_POSITION);
            int target = (int) (36 * robot.COUNTS_PER_INCH);
            robot.centerWheel.setTargetPosition(target);
            robot.centerWheel.setPower(0.1);
            while (robot.centerWheel.isBusy() && robot.FloorColorSensor.red() <= 1) {
                telemetry.addData("Blue value: ", "%d", robot.FloorColorSensor.blue());
                telemetry.addData("Red value: ", "%d", robot.FloorColorSensor.red());
                telemetry.addData("Center wheel value: ", "%d", robot.centerWheel.getCurrentPosition());
                telemetry.update();
            }
            while (robot.centerWheel.isBusy() && robot.FloorColorSensor.red() > 1) {
                telemetry.addData("Blue value: ", "%d", robot.FloorColorSensor.blue());
                telemetry.addData("Red value: ", "%d", robot.FloorColorSensor.red());
                telemetry.addData("Center wheel value: ", "%d", robot.centerWheel.getCurrentPosition());
                telemetry.update();
            }
            int redline1 = robot.centerWheel.getCurrentPosition();
            while (robot.centerWheel.isBusy() && robot.FloorColorSensor.red() <= 1) {
                telemetry.addData("Blue value: ", "%d", robot.FloorColorSensor.blue());
                telemetry.addData("Red value: ", "%d", robot.FloorColorSensor.red());
                telemetry.addData("Center wheel value: ", "%d", robot.centerWheel.getCurrentPosition());
                telemetry.update();
            }
            int redline2 = robot.centerWheel.getCurrentPosition();

            //Going to the center of the safe zone
            int safezonecenter = (redline1 + redline2) / 2;
            target = safezonecenter;
            robot.centerWheel.setTargetPosition(target);
            while (robot.centerWheel.isBusy()) {
                telemetry.addData("Blue value: ", "%d", robot.FloorColorSensor.blue());
                telemetry.addData("Red value: ", "%d", robot.FloorColorSensor.red());
                telemetry.addData("Center wheel value: ", "%d", robot.centerWheel.getCurrentPosition());
                telemetry.update();
            }
            if (pic == robot.CENTER){
                target = (int) (robot.centerWheel.getCurrentPosition()+7* robot.COUNTS_PER_INCH);
                robot.centerWheel.setTargetPosition(target);
            }
            else if(pic == robot.RIGHT){
                target = (int) (robot.centerWheel.getCurrentPosition()-1* robot.COUNTS_PER_INCH);
                robot.centerWheel.setTargetPosition(target);
            }
            else{
                target = (int) (robot.centerWheel.getCurrentPosition()+15* robot.COUNTS_PER_INCH);
                robot.centerWheel.setTargetPosition(target);
            }
        }
        robot.centerWheel.setPower(0);
    }

    public void prepareToMove() throws InterruptedException{
        robot.leftClaw.setPosition(robot.LEFT_CLAW_CLOSED);
        robot.rightClaw.setPosition(robot.RIGHT_CLAW_CLOSED);
        Thread.sleep(250);
        armHandler.armToPosition(robot.GLYPH_ARM_UP);
    }
}