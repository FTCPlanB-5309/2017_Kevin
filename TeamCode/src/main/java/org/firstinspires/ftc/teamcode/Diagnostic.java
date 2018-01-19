package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;

import org.firstinspires.ftc.robotcore.external.navigation.RelicRecoveryVuMark;

/**
 * Created by user on 11/1/2017.
 */
@Autonomous(name="Diagnostic", group="Sensor")

public class Diagnostic extends LinearOpMode {

    RobotHardware         robot   = new RobotHardware(telemetry);

    public void runOpMode() throws InterruptedException {
        robot.init(hardwareMap);
        ConceptVuMarkId conceptVuMarkId = new ConceptVuMarkId(hardwareMap, telemetry);
        RelicRecoveryVuMark columnPosition;
        while(robot.gyroSensor.isCalibrating()){
            telemetry.addData("0 CSC: ", robot.gyroSensor.isCalibrating());
            telemetry.update();
        }
        waitForStart();
        while(opModeIsActive()){
            telemetry.addData("7 Heading: ", robot.gyroSensor.getHeading());
            telemetry.addData("4 X: ", robot.gyroSensor.rawX());
            telemetry.addData("5 Y: ", robot.gyroSensor.rawY());
            telemetry.addData("6 Z: ", robot.gyroSensor.rawZ());
            telemetry.addData("1 R: ", robot.JewelColorSensor.red());
            telemetry.addData("2 G: ", robot.JewelColorSensor.green());
            telemetry.addData("3 B: ", robot.JewelColorSensor.blue());
            telemetry.addData("8 Alpha: ", robot.JewelColorSensor.alpha());
            telemetry.addData("11 R: ", robot.FloorColorSensor.red());
            telemetry.addData("20 G: ", robot.FloorColorSensor.green());
            telemetry.addData("30 B: ", robot.FloorColorSensor.blue());
            telemetry.addData("80 Alpha: ", robot.FloorColorSensor.alpha());
            telemetry.addData("9 Distance", robot.sonicOne.cmUltrasonic());
            telemetry.addData("10 Two Distance", (robot.sonicTwo.cmUltrasonic()/2.54));
            telemetry.update();

        }
    }

}


