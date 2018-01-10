package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import org.firstinspires.ftc.robotcore.external.navigation.RelicRecoveryVuMark;


@Autonomous(name="Right Red Autonomous", group="Red")

public class RedRightAutonomous extends LinearOpMode {
    //
    // Creating the robot class from the Plan B Hardware class and passing in telemetry
    // object as a parameter so that the hardware class can spit out telemetry data
    //
    RobotHardware         robot   = new RobotHardware(telemetry);
    Jewel                 jewel   = new Jewel(robot, telemetry);
    Forward forward = new Forward(robot, telemetry);
    Backward backward = new Backward(robot, telemetry);
    Gyro gyro = new Gyro(robot, telemetry);
    ArmHandler armHandler = new ArmHandler(robot, telemetry);
    ColorSensorSlide colorSensorSlide = new ColorSensorSlide(robot, telemetry);
    Glyph glyph = new Glyph(robot, telemetry);
    GyroForward gyroForward = new GyroForward(robot, telemetry);
    Slide slide = new Slide(robot, telemetry);
    ConceptVuMarkId conceptVuMarkId = null;

    public void runOpMode() throws InterruptedException {
        double distanceForward;
        robot.init(hardwareMap);
        conceptVuMarkId = new ConceptVuMarkId(hardwareMap, telemetry);
        RelicRecoveryVuMark column = RelicRecoveryVuMark.CENTER;

        waitForStart();
        robot.gyroSensor.resetZAxisIntegrator();
        jewel.JewelSwatter(robot.RED);
        column = conceptVuMarkId.findColumn(3000);
        glyph.close();
        armHandler.armToPosition(2800);
        forward.run(0.35, -22);
        slide.run(0.35, 3, robot.LEFT);
        gyro.turn(180);
        armHandler.armToPosition(400);
//        forward.run(0.25, 10);
//        gyro.turn(180);
//        slide.run(0.25, 2, robot.RIGHT);
        colorSensorSlide.findColumn(robot.RED, column, robot.LEFT);
        gyro.turn(180);
        forward.run(0.25, 8);
        glyph.soft();
        forward.run(0.25, -8);

        armHandler.armToPosition(0);
        glyph.open();

    }
}
