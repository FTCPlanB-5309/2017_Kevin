package org.firstinspires.ftc.teamcode;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.hardware.DcMotor;

import org.firstinspires.ftc.teamcode.RobotHardware;
import org.firstinspires.ftc.robotcontroller.external.samples.ConceptVuforiaNavigation;
import org.firstinspires.ftc.robotcore.external.ClassFactory;
import org.firstinspires.ftc.robotcore.external.matrices.OpenGLMatrix;
import org.firstinspires.ftc.robotcore.external.matrices.VectorF;
import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.robotcore.external.navigation.AxesOrder;
import org.firstinspires.ftc.robotcore.external.navigation.AxesReference;
import org.firstinspires.ftc.robotcore.external.navigation.Orientation;
import org.firstinspires.ftc.robotcore.external.navigation.RelicRecoveryVuMark;
import org.firstinspires.ftc.robotcore.external.navigation.VuMarkInstanceId;
import org.firstinspires.ftc.robotcore.external.navigation.VuforiaLocalizer;
import org.firstinspires.ftc.robotcore.external.navigation.VuforiaTrackable;
import org.firstinspires.ftc.robotcore.external.navigation.VuforiaTrackableDefaultListener;
import org.firstinspires.ftc.robotcore.external.navigation.VuforiaTrackables;


@Autonomous(name="Left Blue Autonomous", group="Blue")

public class BlueLeftAuto extends LinearOpMode {

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
        waitForStart();
        jewel.JewelSwatter(robot.BLUE);
        sleep(500);
        columnPosition = conceptVuMarkId.findColumn(5000);
        telemetry.addData("Column Position", columnPosition);
        telemetry.update();
        sleep(500);
        robot.leftClaw.setPosition(robot.LEFT_CLAW_CLOSED);
        robot.rightClaw.setPosition(robot.RIGHT_CLAW_CLOSED);
        sleep(500);
        robot.armMotor.setPower(.25);
        sleep(400);
        robot.armMotor.setPower(0);
        sleep(500);
        forward.run(0.25, 24);
        sleep(500);
        slide.run(0.5, 12, robot.LEFT);
        gyro.turn(0);
        robot.centerWheel.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        robot.centerWheel.setPower(0.07);
        while(true){
            telemetry.addData("Slide Encoder", robot.centerWheel.getCurrentPosition());
            telemetry.addData("Ultrasonic Reading", robot.sonicOne.cmUltrasonic());
            telemetry.update();
        }
//        sleep(500);
//        forward.run(0.5, 6);
//        sleep(500);
    }
}
