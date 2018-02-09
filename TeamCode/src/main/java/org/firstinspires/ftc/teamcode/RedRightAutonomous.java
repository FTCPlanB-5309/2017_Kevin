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
    Glyph glyph = new Glyph(robot, telemetry);
    ConceptVuMarkId conceptVuMarkId = null;
    ColorSensorBackward colorSensorBackward = new ColorSensorBackward(robot, telemetry);
    GyroBackward gyroBackward = new GyroBackward(robot,telemetry);

    public void runOpMode() throws InterruptedException {
        robot.init(hardwareMap);
        conceptVuMarkId = new ConceptVuMarkId(hardwareMap, telemetry);
        waitForStart();
        robot.gyroSensor.resetZAxisIntegrator();
        jewel.JewelSwatter(robot.RED);
        RelicRecoveryVuMark column = conceptVuMarkId.findColumn(3000);
        glyph.close();
        sleep(300);
        armHandler.armToPosition(800);
        gyroBackward.distance(26);
        gyro.turn(90);
        telemetry.update();
        robot.gyroSensor.resetZAxisIntegrator();
        armHandler.armToPosition(400);
        colorSensorBackward.findColumn((int)(-8 * robot.COUNTS_PER_INCH), (int)(0 * robot.COUNTS_PER_INCH),
                (int)(7* robot.COUNTS_PER_INCH), column);
        if (column == RelicRecoveryVuMark.LEFT) {
            gyro.turn(50);
            forward.run(0.25, 5);
        }
        else if (column == RelicRecoveryVuMark.CENTER) {
            gyro.turn(45);
            forward.run(0.25, 4);
        }
        else {
            gyro.turn(45);
            forward.run(0.25, 4);
        }
        glyph.soft();
        robot.gyroSensor.resetZAxisIntegrator();
        gyroBackward.distance(4);
        armHandler.armToPosition(0);
        glyph.open();
    }
}
