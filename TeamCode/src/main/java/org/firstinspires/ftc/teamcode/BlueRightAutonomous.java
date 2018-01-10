package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;

import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit;
import org.firstinspires.ftc.robotcore.external.navigation.RelicRecoveryVuMark;






@Autonomous(name="Right Blue Autonomous", group="Blue")

public class BlueRightAutonomous extends LinearOpMode {

    //
    // Creating the robot class from the Plan B Hardware class and passing in telemetry
    // object as a parameter so that the hardware class can spit out telemetry data
    //
    RobotHardware         robot   = new RobotHardware(telemetry);
    Jewel                 jewel   = new Jewel(robot, telemetry);
    Forward forward = new Forward(robot,telemetry);
    Gyro gyro = new Gyro(robot, telemetry);
    ArmHandler armHandler = new ArmHandler(robot, telemetry);
    ColorSensorSlide colorSensorSlide = new ColorSensorSlide(robot, telemetry);
    Glyph glyph = new Glyph(robot, telemetry);
    GyroForward gyroForward = new GyroForward(robot, telemetry);
    ConceptVuMarkId conceptVuMarkId = null;
    Slide slide = new Slide(robot,telemetry);

    public void runOpMode() throws InterruptedException {
        double distanceForward;
        robot.init(hardwareMap);
        conceptVuMarkId = new ConceptVuMarkId(hardwareMap, telemetry);
        RelicRecoveryVuMark column = null;
        waitForStart();
        robot.gyroSensor.resetZAxisIntegrator();
        jewel.JewelSwatter(robot.BLUE);
        column = conceptVuMarkId.findColumn(5000);
        glyph.close();
        armHandler.armToPosition(400);
        gyroForward.distance(30, 0);
        slide.run(0.5, 3, robot.RIGHT);
        gyro.turn(90);
        gyroForward.sonic(14, 90);
        colorSensorSlide.findColumn(robot.BLUE, column, robot.RIGHT);
        forward.run(0.25, 8);
        glyph.soft();
        forward.run(0.25, -8);
        armHandler.armToPosition(0);
        glyph.open();
    }
}
