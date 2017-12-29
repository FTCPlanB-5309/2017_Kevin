package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.hardware.DcMotor;

import org.firstinspires.ftc.robotcore.external.Telemetry;
import org.firstinspires.ftc.robotcore.external.navigation.RelicRecoveryVuMark;

/**
 * Created by user on 12/28/2017.
 */

public class ColorSensorSlide {
    RobotHardware robot;
    Telemetry telemetry;

    public ColorSensorSlide (RobotHardware robot, Telemetry telemetry) {
        this.robot = robot;
        this.telemetry = telemetry;
    }

    public void findColumn (int allianceColor, RelicRecoveryVuMark column, int slideDirection) {
        int direction;
        if (slideDirection == robot.RIGHT)
            direction = -1;
        else
            direction = 1;

        if(allianceColor==robot.BLUE) {
            robot.centerWheel.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
            robot.centerWheel.setMode(DcMotor.RunMode.RUN_TO_POSITION);
            int target = (int) (36 * robot.COUNTS_PER_INCH) * direction;
            robot.centerWheel.setTargetPosition(target);
            robot.centerWheel.setPower(-0.2);

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
            if (column == RelicRecoveryVuMark.CENTER){
                target = (int) (safezonecenter + 7 * robot.COUNTS_PER_INCH);
                robot.centerWheel.setTargetPosition(target);
            }
            else if(column == RelicRecoveryVuMark.RIGHT){
                target = (int) (safezonecenter - 2 * robot.COUNTS_PER_INCH);
                robot.centerWheel.setTargetPosition(target);
            }
            else {
                target = (int) (safezonecenter + 15 * robot.COUNTS_PER_INCH);
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
        if (allianceColor == robot.RED) {
            robot.centerWheel.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
            robot.centerWheel.setMode(DcMotor.RunMode.RUN_TO_POSITION);
            int target = (int) (36 * robot.COUNTS_PER_INCH) * direction;
            robot.centerWheel.setTargetPosition(target);
            robot.centerWheel.setPower(0.2);
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
            if (column == RelicRecoveryVuMark.CENTER){
                target = (int) (robot.centerWheel.getCurrentPosition() + 7 * robot.COUNTS_PER_INCH);
                robot.centerWheel.setTargetPosition(target);
            }
            else if(column == RelicRecoveryVuMark.RIGHT){
                target = (int) (robot.centerWheel.getCurrentPosition() - 2 * robot.COUNTS_PER_INCH);
                robot.centerWheel.setTargetPosition(target);
            }
            else {
                target = (int) (robot.centerWheel.getCurrentPosition() + 15 * robot.COUNTS_PER_INCH);
                robot.centerWheel.setTargetPosition(target);
            }
        }
        robot.centerWheel.setPower(0);
    }
}
