package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;

import org.firstinspires.ftc.robotcore.external.navigation.RelicRecoveryVuMark;


@TeleOp(name="Autonomous Test", group="Blue")

public class AutoTest extends LinearOpMode{

    //
    // Creating the robot class from the Plan B Hardware class and passing in telemetry
    // object as a parameter so that the hardware class can spit out telemetry data
    //
    RobotHardware         robot   = new RobotHardware(telemetry);
//    Jewel                 jewel   = new Jewel(robot, telemetry);
    Forward forward = new Forward(robot,telemetry);
    Slide slide = new Slide(robot, telemetry);
    Gyro gyro = new Gyro(robot, telemetry);
    SonicAlign sonicAlign = new SonicAlign(robot, telemetry, slide);

    public void runOpMode() throws InterruptedException {
        double distanceForward;


        robot.init(hardwareMap);
//        ConceptVuMarkId conceptVuMarkId = new ConceptVuMarkId(hardwareMap, telemetry);
        RelicRecoveryVuMark columnPosition = RelicRecoveryVuMark.CENTER;
        waitForStart();

        //Grabbing the block and lifting it up
        robot.leftClaw.setPosition(robot.LEFT_CLAW_CLOSED);
        robot.rightClaw.setPosition(robot.RIGHT_CLAW_CLOSED);
        sleep(250);
        robot.armMotor.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        robot.armMotor.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        robot.armMotor.setTargetPosition(robot.GLYPH_ARM_UP);
        robot.armMotor.setPower(0.5);
        while (robot.armMotor.isBusy() && opModeIsActive()) {
            Thread.yield();
        }
        robot.armMotor.setPower(0.0);

        //Traveling off the platform
        slide.run(1, 18, robot.RIGHT);
        forward.run(0.5, 24);
        gyro.turn(0);
        slide.run(0.25, 6, robot.RIGHT);
        forward.run(0.25, 5);

        //Finding the edges of the triangles
        robot.centerWheel.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        robot.centerWheel.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        int target = (int) (36 * robot.COUNTS_PER_INCH);
        robot.centerWheel.setTargetPosition(target);
        robot.centerWheel.setPower(0.1);
        while (robot.centerWheel.isBusy() && opModeIsActive() && robot.ColorSensor.blue() <= 3) {
            telemetry.addData("Blue value: ", "%d", robot.ColorSensor.blue());
            telemetry.addData("Red value: ", "%d", robot.ColorSensor.red());
            telemetry.addData("Center wheel value: ", "%d", robot.centerWheel.getCurrentPosition());
            telemetry.update();
        }
        while (robot.centerWheel.isBusy() && opModeIsActive() && robot.ColorSensor.blue() > 3) {
            telemetry.addData("Blue value: ", "%d", robot.ColorSensor.blue());
            telemetry.addData("Red value: ", "%d", robot.ColorSensor.red());
            telemetry.addData("Center wheel value: ", "%d", robot.centerWheel.getCurrentPosition());
            telemetry.update();
        }
        int blueline1 = robot.centerWheel.getCurrentPosition();
        while (robot.centerWheel.isBusy() && opModeIsActive() && robot.ColorSensor.blue() <= 3) {
            telemetry.addData("Blue value: ", "%d", robot.ColorSensor.blue());
            telemetry.addData("Red value: ", "%d", robot.ColorSensor.red());
            telemetry.addData("Center wheel value: ", "%d", robot.centerWheel.getCurrentPosition());
            telemetry.update();
        }
        int blueline2 = robot.centerWheel.getCurrentPosition();

        //Going to the center of the safe zone
        int safezonecenter = (blueline1 + blueline2)/2;
        target = safezonecenter;
        robot.centerWheel.setTargetPosition(target);
        while (robot.centerWheel.isBusy() &&  opModeIsActive()) {
            telemetry.addData("Blue value: ", "%d", robot.ColorSensor.blue());
            telemetry.addData("Red value: ", "%d", robot.ColorSensor.red());
            telemetry.addData("Center wheel value: ", "%d", robot.centerWheel.getCurrentPosition());
            telemetry.update();
        }

        forward.run(0.25, -3);
        robot.armMotor.setTargetPosition(0);
        robot.armMotor.setPower(0.25);
        while (robot.armMotor.isBusy() && opModeIsActive()) {
            Thread.yield();
        }
        robot.armMotor.setPower(0.0);

        forward.run(0.25, 6);
        robot.leftClaw.setPosition(robot.LEFT_CLAW_OPEN);
        robot.rightClaw.setPosition(robot.RIGHT_CLAW_OPEN);
        forward.run(0.25, -6);

        robot.centerWheel.setPower(0.0);

        while (opModeIsActive()) {
            telemetry.addData("Blue value: ", robot.ColorSensor.blue());
            telemetry.addData("Center wheel value: ", "%d", robot.centerWheel.getCurrentPosition());
            telemetry.update();
        }

    }
}
