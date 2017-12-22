package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;

import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit;
import org.firstinspires.ftc.robotcore.external.navigation.RelicRecoveryVuMark;


@TeleOp(name="Autonomous Test", group="Blue")

public class BlueLeftAutonomous {

    @Autonomous(name="Left Blue Autonomous", group="Blue")

    public class AutoTest extends LinearOpMode {

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
        ArmHandler armHandler = new ArmHandler(robot, telemetry);
        AutoPosition autoPosition = new AutoPosition(robot,telemetry);

        public void runOpMode() throws InterruptedException {
            double distanceForward;


            robot.init(hardwareMap);
            //        ConceptVuMarkId conceptVuMarkId = new ConceptVuMarkId(hardwareMap, telemetry);
            RelicRecoveryVuMark columnPosition = RelicRecoveryVuMark.LEFT;
            waitForStart();

            jewel.JewelSwatter(robot.BLUE);

            robot.leftClaw.setPosition(robot.LEFT_CLAW_CLOSED);
            robot.rightClaw.setPosition(robot.RIGHT_CLAW_CLOSED);
            Thread.sleep(250);
            armHandler.armToPosition(400);

            if(!autoPosition.moveToWall(14)){
                while(opModeIsActive()){}
            }

            autoPosition.center(robot.BLUE, robot.RIGHT);

            gyro.turn(0);

            while(robot.sonicOne.cmUltrasonic() > 8){
                robot.leftWheel.setPower(0.15);
                robot.rightWheel.setPower(0.15);
            }
            robot.leftWheel.setPower(0);
            robot.rightWheel.setPower(0);
        }
    }
}
