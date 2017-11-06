package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;

import org.firstinspires.ftc.robotcore.external.navigation.RelicRecoveryVuMark;

/**
 * Created by user on 11/1/2017.
 */
@Autonomous(name="Diagnostic", group="Sensor")

public class Diagnostic extends LinearOpMode {

    //
    // Creating the robot class from the Plan B Hardware class and passing in telemetry
    // object as a parameter so that the hardware class can spit out telemetry data
    //
    RobotHardware         robot   = new RobotHardware(telemetry);
    Jewel                 jewel   = new Jewel(robot, telemetry);
    Forward forward = new Forward(robot,telemetry);
    Slide slide = new Slide(robot, telemetry);
    Gyro gyro = new Gyro(robot, telemetry);

    public void runOpMode() throws InterruptedException {
        robot.init(hardwareMap);
        ConceptVuMarkId conceptVuMarkId = new ConceptVuMarkId(hardwareMap, telemetry);
        RelicRecoveryVuMark columnPosition;
        while(robot.gyroSensor.isCalibrating()){
            telemetry.addData("0 CSC: ", robot.gyroSensor.isCalibrating());
            telemetry.update();
        }
        waitForStart();
        while(!isStopRequested()){
            telemetry.addData("7 Heading: ", robot.gyroSensor.getHeading());
            telemetry.addData("4 X: ", robot.gyroSensor.rawX());
            telemetry.addData("5 Y: ", robot.gyroSensor.rawY());
            telemetry.addData("6 Z: ", robot.gyroSensor.rawZ());
            telemetry.addData("1 R: ", robot.ColorSensor.red());
            telemetry.addData("2 G: ", robot.ColorSensor.green());
            telemetry.addData("3 B: ", robot.ColorSensor.blue());
            telemetry.addData("8 Alpha: ", robot.ColorSensor.alpha());
            telemetry.update();
        }
    }

}


