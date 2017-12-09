package org.firstinspires.ftc.teamcode;

import android.graphics.Color;
import android.test.suitebuilder.annotation.Suppress;

import com.qualcomm.hardware.modernrobotics.ModernRoboticsI2cGyro;
import com.qualcomm.hardware.modernrobotics.ModernRoboticsI2cRangeSensor;
import com.qualcomm.robotcore.hardware.ColorSensor;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.GyroSensor;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.IntegratingGyroscope;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.hardware.UltrasonicSensor;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.robotcore.external.Telemetry;

@SuppressWarnings("unused")

public class RobotHardware {
    /*
     * Define DC motors
     */
    public DcMotor relicArm = null;
    public DcMotor leftWheel = null;
    public DcMotor rightWheel = null;
    public DcMotor centerWheel = null;
    public DcMotor armMotor = null;
    /*
     * Define Servos
     */
    public Servo armServo = null;
    public Servo jewelServo = null;
    public Servo leftClaw = null;
    public Servo rightClaw = null;
    public Servo upperRightClaw = null;
    public Servo upperLeftClaw = null;
    public Servo rightRelic = null;
    public Servo leftRelic = null;
    public Servo wristRelic = null;
    public Servo extensionRelic = null;
    /*
     * Define sensors
     */
    public ColorSensor ColorSensor = null;
    public ModernRoboticsI2cGyro gyroSensor = null;
    public ModernRoboticsI2cRangeSensor sonicOne = null;

    HardwareMap hwMap = null;
    private ElapsedTime runtime = new ElapsedTime();
    /*
     * Define Servo Constants
     */
    final double ARM_SERVO_UP = 0.88;
    final double ARM_SERVO_DOWN = 0.25;
    final double JEWEL_SERVO_MIDDLE = 0.47;
    final double JEWEL_SERVO_LEFT = 0;
    final double JEWEL_SERVO_RIGHT = 1;

    final int GLYPH_ARM_UP = 3100;
    final double RIGHT_CLAW_OPEN = 0.235;
    final double RIGHT_CLAW_CLOSED = 0.431;
    final double RIGHT_CLAW_SOFT = 0.283;
    final double LEFT_CLAW_OPEN = 0.627;
    final double LEFT_CLAW_CLOSED = 0.5;
    final double LEFT_CLAW_SOFT =0.58;
    final double UPPER_RIGHT_CLAW_OPEN = 0.471;
    final double UPPER_RIGHT_CLAW_CLOSED = 0.686;
    final double UPPER_RIGHT_CLAW_SOFT =0.53;
    final double UPPER_LEFT_CLAW_OPEN = 0.667;
    final double UPPER_LEFT_CLAW_CLOSED = 0.451;
    final double UPPER_LEFT_CLAW_SOFT = 0.550;
    final double RIGHT_RELIC_OPEN = 0.196;
    final double RIGHT_RELIC_CLOSED = 0.333;

    final double LEFT_RELIC_OPEN = 0.549;
    final double LEFT_RELIC_CLOSED = 0.411;

    final double INIT_RELIC_WRIST = 0.5;

    final int LEFT = 101011011;
    final int RIGHT = 10101111;
    public int armPosition;
    static final double COUNTS_PER_MOTOR_REV = 1080;    // eg: TETRIX Motor Encoder
    static final double DRIVE_GEAR_REDUCTION = 1;     // This is < 1.0 if geared UP
    static final double WHEEL_DIAMETER_INCHES = 4.0;     // For figuring circumference
    static final double COUNTS_PER_INCH = (COUNTS_PER_MOTOR_REV * DRIVE_GEAR_REDUCTION) /
            (WHEEL_DIAMETER_INCHES * 3.1415);
    public final int BLUE = 0;
    public final int RED = 1;
    public final float DEADZONE = 0.1f;
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
        relicArm = hwMap.dcMotor.get("RA");
        leftRelic = hwMap.servo.get("LR");
        rightRelic = hwMap.servo.get("RR");
        wristRelic = hwMap.servo.get("WR");
        extensionRelic = hwMap.servo.get("ER");
        leftWheel.setDirection(DcMotor.Direction.REVERSE);
        rightWheel.setDirection(DcMotor.Direction.FORWARD);
        armMotor.setDirection(DcMotor.Direction.REVERSE);
        armMotor.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        relicArm.setDirection(DcMotor.Direction.REVERSE);
        leftWheel.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        rightWheel.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        centerWheel.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);

        armServo = hwMap.servo.get("armServo");
        jewelServo = hwMap.servo.get("jewelServo");
        leftClaw = hwMap.servo.get("LC");
        rightClaw = hwMap.servo.get("RC");
        upperLeftClaw = hwMap.servo.get("uRC");
        upperRightClaw = hwMap.servo.get("uLC");
        ColorSensor = hwMap.colorSensor.get("colorSensor");
        ColorSensor.enableLed(true);
        gyroSensor = hwMap.get(ModernRoboticsI2cGyro.class, "gyro");
        sonicOne = hwMap.get(ModernRoboticsI2cRangeSensor.class, "S1");
        /*
         * Set initial servo positions
         */
        armServo.setPosition(ARM_SERVO_UP);
        jewelServo.setPosition(JEWEL_SERVO_LEFT);
        leftClaw.setPosition(LEFT_CLAW_OPEN);
        rightClaw.setPosition(RIGHT_CLAW_OPEN);
        leftRelic.setPosition(LEFT_RELIC_CLOSED);
        rightRelic.setPosition(RIGHT_RELIC_CLOSED);
        wristRelic.setPosition(INIT_RELIC_WRIST);
        upperRightClaw.setPosition(UPPER_RIGHT_CLAW_OPEN);
        upperLeftClaw.setPosition(UPPER_LEFT_CLAW_OPEN);
        extensionRelic.setPosition(.5);

        /*
        Initializing gyro
         */
        gyroSensor.calibrate();
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
        leftWheel.setPower(-speed);
        rightWheel.setPower(speed);
        runtime.reset();
        // keep looping while we are still active, and there is time left, and both motors are running.
        while (leftWheel.isBusy() && rightWheel.isBusy() && runtime.milliseconds() < timeInMS) {
            // Allow time for other processes to run.
            Thread.yield();

            telemetry.addData("left wheel encoder: ", leftWheel.getCurrentPosition());
            telemetry.addData("right wheel encoder", rightWheel.getCurrentPosition());
            telemetry.update();
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