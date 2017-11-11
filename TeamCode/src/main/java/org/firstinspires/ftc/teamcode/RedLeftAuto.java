package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;

import org.firstinspires.ftc.robotcore.external.navigation.RelicRecoveryVuMark;

@Autonomous(name="Left Red Autonomous", group="Red")

public class RedLeftAuto extends LinearOpMode {

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
        jewel.JewelSwatter(robot.RED);
        columnPosition = conceptVuMarkId.findColumn(5000);
        robot.leftClaw.setPosition(robot.LEFT_CLAW_CLOSED);
        robot.rightClaw.setPosition(robot.RIGHT_CLAW_CLOSED);
        sleep(1000);
        robot.armMotor.setPower(0.25);
        sleep(600);
        robot.armMotor.setPower(0);
        sleep(1000);
        forward.run(0.25, -46);
        slide.run(0.2, 1, robot.RIGHT);
        gyro.turn(90);
        sleep(1000);
        distanceForward = sonicAlign.run(columnPosition);
        telemetry.addData("distance forward: ", distanceForward);
        forward.run(0.25, (int)distanceForward);

    }

}


