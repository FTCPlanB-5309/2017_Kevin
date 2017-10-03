package org.firstinspires.ftc.teamcode;

import android.test.suitebuilder.annotation.Suppress;

import com.qualcomm.robotcore.hardware.ColorSensor;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.I2cAddr;
import com.qualcomm.robotcore.hardware.I2cDevice;
import com.qualcomm.robotcore.hardware.I2cDeviceSynch;
import com.qualcomm.robotcore.hardware.I2cDeviceSynchImpl;
import com.qualcomm.robotcore.hardware.OpticalDistanceSensor;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.util.ElapsedTime;
import com.qualcomm.robotcore.util.Range;
@SuppressWarnings("unused")

public class RobotHardware {
    public OpticalDistanceSensor OpticalSensor = null;
    public ColorSensor ColorSensor = null;
    public Servo rightFinger = null;
    public Servo CServo = null;
    public DcMotor leftWheel = null;
    public DcMotor rightWheel = null;
    public DcMotor centerWheel = null;
    public DcMotor liftMotor1 = null;
    public DcMotor liftMotor2 = null;
    public DcMotor rightShooter = null;
    public DcMotor leftShooter = null;
    public DcMotor sweeper1 = null;
    public DcMotor sweeper2 = null;
    public int color;
    public int currentRed;
    public int currentBlue;
    HardwareMap hwMap = null;
    private ElapsedTime runtime = new ElapsedTime();
    final int RIGHT_FINGER_OUT = 1;
    final int RIGHT_FINGER_IN = 0;
    final int CLICKS_PER_DEGREE = 25;
    final int RED = 1;
    final int BLUE = 3;
    static final double COUNTS_PER_MOTOR_REV = 1180;    // eg: TETRIX Motor Encoder
    static final double DRIVE_GEAR_REDUCTION = 1;     // This is < 1.0 if geared UP
    static final double WHEEL_DIAMETER_INCHES = 4.0;     // For figuring circumference
    static final double COUNTS_PER_INCH = (COUNTS_PER_MOTOR_REV * DRIVE_GEAR_REDUCTION) /
            (WHEEL_DIAMETER_INCHES * 3.1415);
    byte[] range1Cache; //The read will return an array of bytes. They are stored in this variable
    byte[] range2Cache;
    static final int[] correctionArray = new int[] {-175, -140, -80, 0, 80, 165, 230, 245, 345, 397};

//    public I2cDevice RANGE2;
//    public I2cDeviceSynch RANGE2Reader;
//    public I2cDevice RANGE1;
//    public I2cDeviceSynch RANGE1Reader;

    public RobotHardware() {

    }

    public void init(HardwareMap ahwMap) {
        hwMap = ahwMap;

        leftWheel = hwMap.dcMotor.get("LD");
        rightWheel = hwMap.dcMotor.get("RD");
        centerWheel = hwMap.dcMotor.get("CD");
        leftWheel.setDirection(DcMotor.Direction.FORWARD);
        rightWheel.setDirection(DcMotor.Direction.REVERSE);
        leftWheel.setPower(0);
        rightWheel.setPower(0);
        centerWheel.setPower(0);
        leftWheel.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        rightWheel.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        centerWheel.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        /*
        RANGE1 = hwMap.i2cDevice.get("range");
        RANGE2 = hwMap.i2cDevice.get("range2");
        RANGE2Reader = new I2cDeviceSynchImpl(RANGE1, I2cAddr.create8bit(0x28), false);
        RANGE1Reader = new I2cDeviceSynchImpl(RANGE2, I2cAddr.create8bit(0x2a), false);
        leftShooter.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        rightShooter.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        RANGE2Reader.engage();
        RANGE1Reader.engage();
        rightFinger.setPosition(0.5);
        CServo.setPosition(0);
        ColorSensor.enableLed(false);*/
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
        int clicks = degrees * CLICKS_PER_DEGREE;

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
        int clicks = degrees * CLICKS_PER_DEGREE;

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

    public void findLine(double power) {
        leftWheel.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        rightWheel.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        leftWheel.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        rightWheel.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        leftWheel.setPower(power);
        rightWheel.setPower(power);
        while (OpticalSensor.getLightDetected() < .8) {
            Thread.yield();
        }
        // Stop all motion;
        rightWheel.setPower(0);
        leftWheel.setPower(0);
    }

    public void poke(int alliance, double speed) throws InterruptedException {
        currentBlue = ColorSensor.blue();
        currentRed = ColorSensor.red();
        if (currentBlue >= BLUE || currentRed >= RED) {
            if (currentBlue >= BLUE)
                color = BLUE;
            else
                color = RED;
            CServo.setPosition(0);
            if (alliance == color) {
                rightFinger.setPosition(RIGHT_FINGER_OUT);
            } else {
                forward(speed, 4, 750);
                rightFinger.setPosition(RIGHT_FINGER_OUT);
            }
            Thread.sleep(2500);
            rightFinger.setPosition(RIGHT_FINGER_IN);
        }
        CServo.setPosition(0);
    }

    public void rightTankTurn(int power, int degrees) {
        int clicks = degrees * CLICKS_PER_DEGREE;
        rightWheel.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        rightWheel.setTargetPosition(clicks);
        rightWheel.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        rightWheel.setPower(power);
        leftWheel.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        leftWheel.setTargetPosition((int) COUNTS_PER_INCH * 4);
        leftWheel.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        leftWheel.setPower(0.5);


        // keep looping while we are still active, and there is time left, and both motors are running.
        while (rightWheel.isBusy()) {
            // Allow time for other processes to run.
            Thread.yield();
        }
        rightWheel.setPower(0);
        leftWheel.setPower(0);
    }



    public void leftTankTurn(double power, int degrees) {
        int clicks = degrees * CLICKS_PER_DEGREE;

        leftWheel.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        leftWheel.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        leftWheel.setTargetPosition(clicks);
        leftWheel.setPower(power);
        rightWheel.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        rightWheel.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        rightWheel.setTargetPosition(((int) COUNTS_PER_INCH * 2) + 45);
        rightWheel.setPower(0.5);

        // keep looping while we are still active, and there is time left, and both motors are running.
        while (leftWheel.isBusy()) {
            // Allow time for other processes to run.
            Thread.yield();
        }

        // Stop all motion;
        leftWheel.setPower(0);
        rightWheel.setPower(0);
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