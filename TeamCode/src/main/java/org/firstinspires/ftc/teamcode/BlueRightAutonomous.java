package org.firstinspires.ftc.teamcode;

        import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
        import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
        import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
        import com.qualcomm.robotcore.hardware.DcMotor;
        import com.vuforia.CameraDevice;

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
    Backward backward = new Backward(robot,telemetry);
    ConceptVuMarkId conceptVuMarkId = null;
    Slide slide = new Slide(robot,telemetry);
    ColorSensorForward colorSensorForward = new ColorSensorForward(robot,telemetry);
    GyroBackward gyroBackward = new GyroBackward(robot,telemetry);
    public void runOpMode() throws InterruptedException {
        robot.init(hardwareMap);
        conceptVuMarkId = new ConceptVuMarkId(hardwareMap, telemetry);

        RelicRecoveryVuMark column = null;
        waitForStart();

        robot.gyroSensor.resetZAxisIntegrator();
        jewel.JewelSwatter(robot.BLUE);
        column = conceptVuMarkId.findColumn(4000);
        glyph.close();
        sleep(300);
        armHandler.armToPosition(600);
        forward.run(0.15, 24);
        telemetry.update();
        gyro.turn(0);
        robot.armMotor.setPower(-1);
        sleep(1000);
        robot.armMotor.setPower(0);
        colorSensorForward.findColumn((int)(-7 * robot.COUNTS_PER_INCH), (int)(-7 * robot.COUNTS_PER_INCH),
                (int)(2* robot.COUNTS_PER_INCH), column);
        if (column == RelicRecoveryVuMark.LEFT) {
            gyro.turn(65);
            forward.run(0.25, 6);
        }
        else if (column == RelicRecoveryVuMark.CENTER) {
            gyro.turn(50);
            forward.run(0.25, 7);
        }
        else {
            gyro.turn(50);
            forward.run(0.25, 6);
        }

        glyph.soft();
        robot.gyroSensor.resetZAxisIntegrator();
        gyroBackward.distance(4);
        armHandler.armToPosition(0);
        glyph.open();
    }
}

//        robot.init(hardwareMap);
//                conceptVuMarkId = new ConceptVuMarkId(hardwareMap, telemetry);
//                RelicRecoveryVuMark column = null;
//                waitForStart();
//                robot.gyroSensor.resetZAxisIntegrator();
//                jewel.JewelSwatter(robot.BLUE);
//                column = conceptVuMarkId.findColumn(5000);
//                glyph.close();
//                sleep(300);
//                armHandler.armToPosition(400);
//                forward.run(0.35, 30);
//                slide.run(0.5, 3, robot.RIGHT);
//                gyro.turn(90);
//                robot.gyroSensor.resetZAxisIntegrator();
//                sleep(100);
//                slide.run(0.5,3,robot.LEFT);
//                gyroForward.sonic(15, 0);
//                colorSensorSlide.findColumn( (int)(16.5 * robot.COUNTS_PER_INCH),
//                (int) (7* robot.COUNTS_PER_INCH),
//                (int) (1*robot.COUNTS_PER_INCH),
//                column, robot.RIGHT);
//                gyro.turn(0);
//                forward.run(0.25, 8);
//                glyph.open();
//                forward.run(0.25, -8);
//                armHandler.armToPosition(0);
//                glyph.open();