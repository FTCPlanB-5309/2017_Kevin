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
import com.qualcomm.robotcore.hardware.I2cAddr;
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
    public DcMotor jewelArmMotor = null;
    /*
     * Define Servos
     */
    public Servo jewelServo = null;
    public Servo leftClaw = null;
    public Servo rightClaw = null;
    public Servo upperRightClaw = null;
    public Servo upperLeftClaw = null;
    public Servo rightRelic = null;
    public Servo leftRelic = null;
    public Servo wristRelic = null;
    public Servo extensionRelic = null;
    public Servo ExtensionReversed = null;
    /*
     * Define sensors
     */
    public ColorSensor JewelColorSensor = null;
    public ColorSensor FloorColorSensor = null;
    public ModernRoboticsI2cGyro gyroSensor = null;
    public ModernRoboticsI2cRangeSensor sonicOne = null;
    public ModernRoboticsI2cRangeSensor sonicTwo = null;

    HardwareMap hwMap = null;
    private ElapsedTime runtime = new ElapsedTime();

    int[] leftArray = new int[] {0, 15, 15, 15, 15};
    int[] centerArray = new int[] {0, 7, 7, 7, 7};
    int[] rightArray = new int[] {0, 2, 2, 2, 2};





    /*
     * Define Servo Constants
     */
    final double ARM_SERVO_UP = 0.88;
    final double ARM_SERVO_DOWN = 0.25;
    final double JEWEL_SERVO_MIDDLE = 0.47;
    final double JEWEL_SERVO_LEFT = 0;
    final double JEWEL_SERVO_RIGHT = 1;
    final double PLATFORM_SPEED = 0.15;
    final double MOVEMENT_INCREMENT =0.03;
    final int GLYPH_ARM_UP = 3100;
    final double RIGHT_CLAW_OPEN = 0.235;
    final double RIGHT_CLAW_CLOSED = 0.431;
    final double RIGHT_CLAW_SOFT = 0.33;
    final double RIGHT_CLAW_INIT = 0.19;
    final double LEFT_CLAW_OPEN = 0.627;
    final double LEFT_CLAW_CLOSED = 0.38;
    final double LEFT_CLAW_SOFT =0.58;
    final double LEFT_CLAW_INIT = 0.7;
    final double UPPER_RIGHT_CLAW_OPEN = 0.471;
    final double UPPER_RIGHT_CLAW_CLOSED = 0.686;
    final double UPPER_RIGHT_CLAW_SOFT =0.53;
    final double UPPER_RIGHT_CLAW_INIT = 0.4;
    final double UPPER_LEFT_CLAW_OPEN = 0.667;
    final double UPPER_LEFT_CLAW_CLOSED = 0.451;
    final double UPPER_LEFT_CLAW_SOFT = 0.550;
    final double UPPER_LEFT_CLAW_INIT = 0.73;
    final double RIGHT_RELIC_OPEN = 0.55;
    final double RIGHT_RELIC_CLOSED = 0.313;
    final double OPEN = 1;
    final double CLOSE = 0;
    final double SOFT = 2;
    final double CENTER = 85;
    final double LEFT_RELIC_OPEN = 0.27;
    final double LEFT_RELIC_CLOSED = 0.45;

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
    public final float DEADZONE = 0.15f;
    Telemetry telemetry;

    public RobotHardware(Telemetry telemetry){
        this.telemetry = telemetry;
    }

    public void init(HardwareMap ahwMap) {
        hwMap = ahwMap;

        leftWheel = hwMap.dcMotor.get("leftDrive");
        rightWheel = hwMap.dcMotor.get("rightDrive");
        centerWheel = hwMap.dcMotor.get("centerDrive");
        armMotor = hwMap.dcMotor.get("glyphArmMotor");
        relicArm = hwMap.dcMotor.get("relicArmMotor");
        jewelArmMotor = hwMap.dcMotor.get("jewelArmMotor");

        leftRelic = hwMap.servo.get("LR");
        rightRelic = hwMap.servo.get("RR");
        wristRelic = hwMap.servo.get("WR");
        extensionRelic = hwMap.servo.get("ER");
        ExtensionReversed = hwMap.servo.get("RE");
        leftWheel.setDirection(DcMotor.Direction.REVERSE);
        rightWheel.setDirection(DcMotor.Direction.FORWARD);
        armMotor.setDirection(DcMotor.Direction.REVERSE);
        armMotor.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        relicArm.setDirection(DcMotor.Direction.REVERSE);
        leftWheel.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        rightWheel.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        centerWheel.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        jewelArmMotor.setDirection(DcMotor.Direction.REVERSE);
        jewelArmMotor.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);

        jewelServo = hwMap.servo.get("jewelServo");
        leftClaw = hwMap.servo.get("LC");
        rightClaw = hwMap.servo.get("RC");
        upperLeftClaw = hwMap.servo.get("uRC");
        upperRightClaw = hwMap.servo.get("uLC");
        JewelColorSensor = hwMap.colorSensor.get("colorSensor");
        JewelColorSensor.setI2cAddress(I2cAddr.create8bit(0x10));
        JewelColorSensor.enableLed(true);
        FloorColorSensor = hwMap.colorSensor.get("floorcolorsensor");
        FloorColorSensor.enableLed(true);
        gyroSensor = hwMap.get(ModernRoboticsI2cGyro.class, "gyro");
        sonicOne = hwMap.get(ModernRoboticsI2cRangeSensor.class, "S1");
        sonicTwo = hwMap.get(ModernRoboticsI2cRangeSensor.class, "S2");
        sonicTwo.setI2cAddress(I2cAddr.create8bit(0x2a));
        /*
         * Set initial servo positions
         */
        jewelServo.setPosition(JEWEL_SERVO_LEFT);
        leftClaw.setPosition(LEFT_CLAW_INIT);
        rightClaw.setPosition(RIGHT_CLAW_INIT);
        leftRelic.setPosition(LEFT_RELIC_CLOSED);
        rightRelic.setPosition(RIGHT_RELIC_CLOSED);
        wristRelic.setPosition(INIT_RELIC_WRIST);
        upperRightClaw.setPosition(UPPER_RIGHT_CLAW_INIT);
        upperLeftClaw.setPosition(UPPER_LEFT_CLAW_INIT);
        extensionRelic.setPosition(.5);
        ExtensionReversed.setPosition(.5);

        /*
        Initializing gyro
         */
        gyroSensor.calibrate();
    }
    public void initTeleop(HardwareMap ahwMap) {
        hwMap = ahwMap;

        leftWheel = hwMap.dcMotor.get("leftDrive");
        rightWheel = hwMap.dcMotor.get("rightDrive");
        centerWheel = hwMap.dcMotor.get("centerDrive");
        armMotor = hwMap.dcMotor.get("glyphArmMotor");
        relicArm = hwMap.dcMotor.get("relicArmMotor");
        jewelArmMotor = hwMap.dcMotor.get("jewelArmMotor");

        leftRelic = hwMap.servo.get("LR");
        rightRelic = hwMap.servo.get("RR");
        wristRelic = hwMap.servo.get("WR");
        extensionRelic = hwMap.servo.get("ER");
        ExtensionReversed = hwMap.servo.get("RE");
        leftWheel.setDirection(DcMotor.Direction.REVERSE);
        rightWheel.setDirection(DcMotor.Direction.FORWARD);
        armMotor.setDirection(DcMotor.Direction.REVERSE);
        armMotor.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        relicArm.setDirection(DcMotor.Direction.REVERSE);
        leftWheel.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        rightWheel.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        centerWheel.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        jewelArmMotor.setDirection(DcMotor.Direction.REVERSE);
        jewelArmMotor.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);

        jewelServo = hwMap.servo.get("jewelServo");
        leftClaw = hwMap.servo.get("LC");
        rightClaw = hwMap.servo.get("RC");
        upperLeftClaw = hwMap.servo.get("uRC");
        upperRightClaw = hwMap.servo.get("uLC");


        leftRelic.setPosition(LEFT_RELIC_CLOSED);
        rightRelic.setPosition(RIGHT_RELIC_CLOSED);
        wristRelic.setPosition(INIT_RELIC_WRIST);
        extensionRelic.setPosition(0.5);
        ExtensionReversed.setPosition(0.5);
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
}