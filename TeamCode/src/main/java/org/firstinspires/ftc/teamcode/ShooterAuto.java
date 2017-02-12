package org.firstinspires.ftc.teamcode;
import android.app.Activity;
import android.graphics.Color;
import android.view.View;

import com.qualcomm.ftcrobotcontroller.R;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.ColorSensor;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.UltrasonicSensor;

/**
 * Created by Tony on 11/20/2016.
 */
@Disabled
@Autonomous(name = "Shooter", group = "Sensor")
public class ShooterAuto extends LinearOpMode {
    public DcMotor leftShooter = null;
    public DcMotor rightShooter = null;
    public void runOpMode() throws InterruptedException {
        leftShooter = hardwareMap.dcMotor.get("LSh");
        rightShooter = hardwareMap.dcMotor.get("RSh");
        waitForStart();


        // Note we use opModeIsActive() as our loop condition because it is an interruptible method. // while the op mode is active, loop and read the RGB data.
        while (opModeIsActive()) {
            leftShooter.setPower(-1);
            rightShooter.setPower(1);
        }
    }
}
