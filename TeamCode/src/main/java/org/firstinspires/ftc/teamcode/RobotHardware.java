package org.firstinspires.ftc.teamcode;

import android.graphics.Color;
import android.test.suitebuilder.annotation.Suppress;

import com.qualcomm.robotcore.hardware.ColorSensor;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.util.ElapsedTime;
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

import org.firstinspires.ftc.robotcore.external.Telemetry;

@SuppressWarnings("unused")

public class RobotHardware {
    public ColorSensor ColorSensor = null;
    public Servo armServo = null;
    public Servo jewelServo = null;
    public Servo leftClaw = null;
    public Servo rightClaw = null;
    public DcMotor leftWheel = null;
    public DcMotor rightWheel = null;
    public DcMotor centerWheel = null;
    public DcMotor armMotor = null;
    public int currentRed;
    public int currentBlue;
    HardwareMap hwMap = null;
    private ElapsedTime runtime = new ElapsedTime();
    final int RIGHT = 314152693;
    final int LEFT = 396251413;
    final int ARM_POSITION_ZERO = 0;
    final int ARM_POSITION_ONE = 1;
    final int ARM_POSITION_TWO = 2;
    final int ARM_POSITION_THREE = 3;
    final int ARM_POSITION_FOUR = 4;
    final double ARM_SERVO_UP = 0.89;
    final double ARM_SERVO_DOWN = 0.05;
    final double JEWEL_SERVO_MIDDLE = 0.47;
    final double JEWEL_SERVO_LEFT = 0;
    final double JEWEL_SERVO_RIGHT = 1;
    final double LEFT_CLAW_CLOSED = 0.5;
    final double LEFT_CLAW_OPEN = 0.2;
    final double RIGHT_CLAW_CLOSED = 0.5;
    final double RIGHT_CLAW_OPEN = 0.7;
    public static int armPosition = 0;
    static final double COUNTS_PER_MOTOR_REV = 1180;    // eg: TETRIX Motor Encoder
    static final double DRIVE_GEAR_REDUCTION = 1;     // This is < 1.0 if geared UP
    static final double WHEEL_DIAMETER_INCHES = 4.0;     // For figuring circumference
    static final double COUNTS_PER_INCH = (COUNTS_PER_MOTOR_REV * DRIVE_GEAR_REDUCTION) /
            (WHEEL_DIAMETER_INCHES * 3.1415);
    public final int BLUE = 0;
    public final int RED = 1;
    Telemetry telemetry;

    public RobotHardware(Telemetry telemetry){
        this.telemetry = telemetry;
    }

    public void init(HardwareMap ahwMap) {
        hwMap = ahwMap;

        leftWheel = hwMap.dcMotor.get("LD");
        rightWheel = hwMap.dcMotor.get("RD");
        centerWheel = hwMap.dcMotor.get("CD");
        armMotor = hwMap.dcMotor.get("AD");
        leftWheel.setDirection(DcMotor.Direction.FORWARD);
        rightWheel.setDirection(DcMotor.Direction.REVERSE);
        leftWheel.setPower(0);
        leftWheel.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        rightWheel.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        centerWheel.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        armServo = hwMap.servo.get("armServo");
        jewelServo = hwMap.servo.get("jewelServo");
        leftClaw = hwMap.servo.get("LC");
        rightClaw = hwMap.servo.get("RC");
        jewelServo.setPosition(JEWEL_SERVO_LEFT);
        armServo.setPosition(ARM_SERVO_UP);
        leftClaw.setPosition(LEFT_CLAW_OPEN);
        rightClaw.setPosition(RIGHT_CLAW_OPEN);
        ColorSensor = hwMap.colorSensor.get("colorSensor");
        ColorSensor.enableLed(true);
    }

    public void Jewel (int allianceColor) throws InterruptedException {
        int blueValue = 0;
        int redValue = 0;
        jewelServo.setPosition(JEWEL_SERVO_MIDDLE);
        Thread.sleep(3000);
        armServo.setPosition(ARM_SERVO_DOWN);
        Thread.sleep(3000);
        blueValue = ColorSensor.blue();
        redValue = ColorSensor.red();
        armServo.setPosition(armServo.getPosition() - 0.01);
        Thread.sleep(500);
        if (ColorSensor.blue() > blueValue)
            blueValue = ColorSensor.blue();
        if (ColorSensor.red() > redValue)
            redValue = ColorSensor.red();
        armServo.setPosition(armServo.getPosition() - 0.01);
        Thread.sleep(500);
        if (ColorSensor.blue() > blueValue)
            blueValue = ColorSensor.blue();
        if (ColorSensor.red() > redValue)
            redValue = ColorSensor.red();
        if (allianceColor == BLUE)
        {
            if (blueValue > redValue)
                jewelServo.setPosition(JEWEL_SERVO_RIGHT);
            if (blueValue < redValue)
                jewelServo.setPosition(JEWEL_SERVO_LEFT);
        }
        else {
            if (blueValue > redValue)
                jewelServo.setPosition(JEWEL_SERVO_LEFT);
            if (blueValue < redValue)
                jewelServo.setPosition(JEWEL_SERVO_RIGHT);
        }
        telemetry.addData("Blue Value", blueValue);
        telemetry.addData("Red Value", redValue);
        Thread.sleep(500);
        armServo.setPosition(ARM_SERVO_UP);
        Thread.sleep(1000);


    }

    public void rightDrive(double power) {
        rightWheel.setPower(power);
    }

    public void leftDrive(double power) {
        leftWheel.setPower(power);
    }

    public void stopDrive() {
        leftWheel.setPower(0);
        rightWheel.setPower(0);
    }

    public void backward(double speed, int distance, int timeInMS) {
        leftWheel.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        rightWheel.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        int target = (int) (distance * COUNTS_PER_INCH);
        leftWheel.setTargetPosition(-target);
        rightWheel.setTargetPosition(-target);
        // Turn On RUN_TO_POSITION
        leftWheel.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        rightWheel.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        leftWheel.setPower(speed);
        rightWheel.setPower(speed);
        runtime.reset();
        // keep looping while we are still active, and there is time left, and both motors are running.
        while (leftWheel.isBusy() && rightWheel.isBusy() && runtime.milliseconds() < timeInMS) {
            // Allow time for other processes to run.
            Thread.yield();
        }

        // Stop all motion;
        leftWheel.setPower(0);
        rightWheel.setPower(0);
    }

    public void forward(double speed, int distance, int timeInMS) {
        leftWheel.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        rightWheel.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);

        int target = (int) (distance * COUNTS_PER_INCH);
        leftWheel.setTargetPosition(target);
        rightWheel.setTargetPosition(target);
        // Turn On RUN_TO_POSITION
        leftWheel.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        rightWheel.setMode(DcMotor.RunMode.RUN_TO_POSITION);

        // reset the timeout time and start motion.
        leftWheel.setPower(speed);
        rightWheel.setPower(speed);
        runtime.reset();
        // keep looping while we are still active, and there is time left, and both motors are running.
        while (leftWheel.isBusy() && rightWheel.isBusy() && runtime.milliseconds() < timeInMS) {
            // Allow time for other processes to run.
            Thread.yield();
        }
        // Stop all motion;
        leftWheel.setPower(0);
        rightWheel.setPower(0);
    }

    public void leftWheelTurn(double speed, int degrees, int timeInMS) {
        int clicks = degrees * 6;

        leftWheel.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        leftWheel.setTargetPosition(clicks);
        leftWheel.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        leftWheel.setPower(speed);
        rightWheel.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        rightWheel.setTargetPosition(2);
        rightWheel.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        rightWheel.setPower(0.5);
        runtime.reset();
       //till active, and there is time left, and both motors are running.
        while (leftWheel.isBusy() && runtime.milliseconds() < timeInMS) {
            // Allow time for other processes to run.
            Thread.yield();
        }

        // Stop all motion;
        leftWheel.setPower(0);
        rightWheel.setPower(0);
    }

    public void rightWheelTurn(double speed, int degrees, int timeInMS) {
        int clicks = degrees * 6;

        rightWheel.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        rightWheel.setTargetPosition(clicks);
        rightWheel.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        rightWheel.setPower(speed);
        leftWheel.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        leftWheel.setTargetPosition(2);
        leftWheel.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        leftWheel.setPower(0.5);
        runtime.reset();
        // keep looping while we are still active, and there is time left, and both motors are running.
        while (rightWheel.isBusy() && runtime.milliseconds() < timeInMS) {
            // Allow time for other processes to run.
            Thread.yield();
        }
        rightWheel.setPower(0);
        leftWheel.setPower(0);
    }







    /*public double rangeRead() {
        range1Cache = RANGE1Reader.read(0x04, 2);  //Read 2 bytes starting at 0x04
        return range1Cache[0] & 0xFF;
    }

    public double rangeRead2() {
        range2Cache = RANGE2Reader.read(0x04, 2);
        return range2Cache[0] & 0xFF;
    }

    public void adjustTurn(double speed) throws InterruptedException {
        double distance = 255;
        double distance2 = 255;
        leftWheel.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        leftWheel.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        rightWheel.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        rightWheel.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        for (int i = 1; i < 10 && (distance == 255 || distance2 == 255); i++) {
            if (distance == 255)
                distance = rangeRead();
            if (distance2 == 255)
                distance2 = rangeRead2();
            Thread.yield();
        }
        double difference = distance2 - distance;
        if (distance == 255 || distance2 == 255) {
            difference = 0;
        }
        double setSpeed;
        if (difference == 0)
            setSpeed = 0;
        else
            setSpeed = speed + ((Math.abs(difference) - 1) * 200);
        setSpeed=setSpeed/3000;
        while (difference != 0) {
            if (distance == 255 || distance2 == 255) {
                leftWheel.setPower(0);
                rightWheel.setPower(0);
            } else if (distance > distance2) {
                leftWheel.setPower(-setSpeed);
                rightWheel.setPower(setSpeed);
            } else {
                leftWheel.setPower(setSpeed);
                rightWheel.setPower(-setSpeed);
            }
            distance = rangeRead();
            distance2 = rangeRead2();
            difference = (int) distance2 - (int) distance;
            setSpeed = speed + ((Math.abs(difference) - 1) * 200);
            Thread.yield();
        }
        rightWheel.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        leftWheel.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
    }
    public int matrixAdjustment(double speed){
        double distance = 255;
        double distance2 = 255;
        leftWheel.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        leftWheel.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        rightWheel.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        rightWheel.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        for (int i = 1; i < 10 && (distance == 255 || distance2 == 255); i++) {
            if (distance == 255)
                distance = rangeRead();
            if (distance2 == 255)
                distance2 = rangeRead2();
            Thread.yield();
        }
        double difference =  distance2 - distance;
        if (distance == 255 || distance2 == 255) {
            difference = 0;
        }
        double setSpeed;
        if (difference == 0)
            setSpeed = 0;
        else
            setSpeed = speed + ((Math.abs(difference) - 1) * 0.07);
        int arrayDiff = (int) difference + 3;
        arrayDiff = Range.clip(arrayDiff, 0, 9);
        if(difference !=0 && (distance != 255 || distance2 != 255)){
            if (distance < distance2) {
                leftWheel.setTargetPosition(correctionArray[arrayDiff]);
                leftWheel.setPower(setSpeed);
            }
            else {
                leftWheel.setTargetPosition(correctionArray[arrayDiff]);
                leftWheel.setPower(-setSpeed);
            }
        }
        while(leftWheel.isBusy() && correctionArray[arrayDiff] != 0) {
            Thread.yield();
        }
        leftWheel.setPower(0);
        rightWheel.setPower(0);
        rightWheel.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        leftWheel.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        return(correctionArray[arrayDiff]);
    }
    /*public void correctWhileDriving(double power, int distance){
        double offset;
        if (power > 0)
            offset = -0.1;
        else
            offset = 0.1;


        leftWheel.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        rightWheel.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);

        int target =(int)(distance * COUNTS_PER_INCH);
        leftWheel.setTargetPosition(target);
        rightWheel.setTargetPosition(target);

        // Turn On RUN_TO_POSITION
        leftWheel.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        rightWheel.setMode(DcMotor.RunMode.RUN_TO_POSITION);

        // reset the timeout time and start motion.
        leftWheel.setPower(power);
        rightWheel.setPower(power);
        double cmDistance = rangeRead2();
        double baseline = cmDistance;
        // keep looping while we are still active, and there is time left, and both motors are running.
        while (leftWheel.isBusy() && rightWheel.isBusy()) {

            if(cmDistance > baseline){
                leftWheel.setPower(Range.clip(power - offset, -1, 1));
                rightWheel.setPower(Range.clip(power + offset, -1, 1));
            }
            else if(cmDistance < baseline){
                leftWheel.setPower(Range.clip(power + offset, -1, 1));
                rightWheel.setPower(Range.clip(power - offset, -1, 1));
            }
            else{
                leftWheel.setPower(power);
                rightWheel.setPower(power);
            }
            Thread.yield();
        }

        // Stop all motion;
        leftWheel.setPower(0);
        rightWheel.setPower(0);

    }*/
}