package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;

import org.firstinspires.ftc.robotcore.external.navigation.RelicRecoveryVuMark;

/**
 * Created by user on 11/1/2017.
 */
@Autonomous(name="Right Blue Autonomous", group="Blue")

public class BlueRightAuto extends LinearOpMode {

    //
    // Creating the robot class from the Plan B Hardware class and passing in telemetry
    // object as a parameter so that the hardware class can spit out telemetry data
    //
    RobotHardware         robot   = new RobotHardware(telemetry);
    Jewel                 jewel   = new Jewel(robot, telemetry);
    Forward forward = new Forward(robot,telemetry);
    Slide slide = new Slide(robot, telemetry);
    Gyro gyro = new Gyro(robot, telemetry);
    SonicAlign sonicAlign = new SonicAlign(robot, telemetry, slide);

    public void runOpMode() throws InterruptedException {
        double distanceForward;

        robot.init(hardwareMap);
        ConceptVuMarkId conceptVuMarkId = new ConceptVuMarkId(hardwareMap, telemetry);
        RelicRecoveryVuMark columnPosition;
        waitForStart();
        robot.gyroSensor.resetZAxisIntegrator();
        jewel.JewelSwatter(robot.BLUE);
        sleep(1000);
        columnPosition = conceptVuMarkId.findColumn(5000);
        robot.leftClaw.setPosition(robot.LEFT_CLAW_CLOSED);
        robot.rightClaw.setPosition(robot.RIGHT_CLAW_CLOSED);
        sleep(1000);
        robot.armMotor.setPower(0.25);
        sleep(400);
        robot.armMotor.setPower(0);
        sleep(1000);
        forward.run(0.25, 25);
        sleep(1000);
        slide.run(0.25, 1, robot.RIGHT);
        gyro.turn(90);
        sleep(1000);
        //forward.run(0.5, 6);
        sleep(1000);
        distanceForward = sonicAlign.run(columnPosition);
        forward.run(0.5, 9);
    }

}


