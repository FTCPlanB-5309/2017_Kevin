package org.firstinspires.ftc.teamcode;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import org.firstinspires.ftc.teamcode.RobotHardware;

@Autonomous(name="Left Blue Autonomous", group="Blue")

public class BlueLeftAuto extends LinearOpMode {

    RobotHardware robot=new RobotHardware(telemetry);
    Jewel                 jewel   = new Jewel(robot, telemetry);
    Forward    forward=new Forward(robot, telemetry);
    Slide                 slide   = new Slide(robot, telemetry);

    @Override
    public void runOpMode() throws InterruptedException {
    //
        robot.init(hardwareMap);
        waitForStart();
        //jewel.JewelSwatter(robot.BLUE);
        robot.leftClaw.setPosition(robot.LEFT_CLAW_CLOSED);
        robot.rightClaw.setPosition(robot.RIGHT_CLAW_CLOSED);
        Thread.sleep(300);
        robot.armMotor.setPower(0.5);
        Thread.sleep(250);
        robot.armMotor.setPower(0);
        forward.run(.5, 26);
        slide.run(.5, 6, robot.RIGHT);
        forward.run(.5, 4);
    }

}
