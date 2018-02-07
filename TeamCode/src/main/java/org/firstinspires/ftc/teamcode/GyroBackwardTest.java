package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;

import org.firstinspires.ftc.robotcore.external.navigation.RelicRecoveryVuMark;

/**
 * Created by user on 2/6/2018.
 */
@Autonomous(name = "Gyro Backward Test", group = "Testing")
public class GyroBackwardTest  extends LinearOpMode {
    RobotHardware         robot   = new RobotHardware(telemetry);
    GyroBackward gyroBackward = new GyroBackward(robot, telemetry);

    public void runOpMode() throws InterruptedException {
        robot.init(hardwareMap);

        waitForStart();
        robot.gyroSensor.resetZAxisIntegrator();
        gyroBackward.distance(50);
    }
}
