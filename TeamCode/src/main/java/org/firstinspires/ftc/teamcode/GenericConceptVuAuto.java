package org.firstinspires.ftc.teamcode;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.Disabled;
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


@Autonomous(name="Generic Concept View Auto", group="Test")

public class GenericConceptVuAuto extends LinearOpMode {

    //
    // Creating the robot class from the Plan B Hardware class and passing in telemetry
    // object as a parameter so that the hardware class can spit out telemetry data
    //
    RobotHardware         robot     = new RobotHardware(telemetry);

    public void runOpMode() throws InterruptedException {
        robot.init(hardwareMap);
        Jewel           jewel           = new Jewel(robot, telemetry);
        Forward         forward         = new Forward(robot,telemetry);
        Slide           slide           = new Slide(robot, telemetry);
        Gyro            gyro            = new Gyro(robot, telemetry);
        ConceptVuMarkId conceptVuMarkId = new ConceptVuMarkId(hardwareMap, telemetry);

        RelicRecoveryVuMark columnPosition;

        waitForStart();
        /*
         * Swat the jewel
         */
        jewel.JewelSwatter(robot.BLUE);
        /*
         * Look for VuMark for no more than 15 seconds
         */
        columnPosition = conceptVuMarkId.findColumn(15000);

        /*
         * Grab the glyph
         */
        robot.leftClaw.setPosition(robot.LEFT_CLAW_CLOSED);
        robot.rightClaw.setPosition(robot.RIGHT_CLAW_CLOSED);
        sleep(1000);

        /*
         * Raise the Glyph Arm
         */
        robot.armMotor.setPower(.25);
        sleep(400);
        robot.armMotor.setPower(0);
        sleep(500);

        /*
         * Drive off the balancing stone
         */
        forward.run(0.25, 24);
        sleep(1000);

        /*
         * Realign to zero
         */
        gyro.turn(0);

        /*
         * slide over in front of glyph box
         */
        slide.run(0.5, 6, robot.RIGHT);
        sleep(1000);

        /*
         * stuff the glyph in the box
         */
        forward.run(0.3, 6);
        sleep(1000);
    }
}
