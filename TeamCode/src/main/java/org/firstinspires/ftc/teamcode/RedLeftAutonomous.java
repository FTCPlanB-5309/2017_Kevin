package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import org.firstinspires.ftc.robotcore.external.navigation.RelicRecoveryVuMark;
import com.vuforia.CameraDevice;



@Autonomous(name = "Left Red Autonomous", group = "Red")
public class RedLeftAutonomous extends LinearOpMode {

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
        RelicRecoveryVuMark column = null;
        waitForStart();

        CameraDevice.getInstance().setFlashTorchMode(true);

        robot.gyroSensor.resetZAxisIntegrator();
        jewel.JewelSwatter(robot.RED);
        column = conceptVuMarkId.findColumn(4000);
        glyph.close();
        armHandler.armToPosition(500);

        if (column == RelicRecoveryVuMark.RIGHT) {
            backward.run(0.25, 40);
            gyro.turn(45);
        }
        else if (column == RelicRecoveryVuMark.CENTER) {
            backward.run(0.25, 49);
            gyro.turn(50);
        }
        else if (column == RelicRecoveryVuMark.LEFT) {
            backward.run(0.25, 29);
            gyro.turn(210);
            gyro.turn(125);
        }
        forward.run(0.25, 11);
        glyph.soft();
        backward.run(0.25, 6);
        armHandler.armToPosition(0);
    }
}
