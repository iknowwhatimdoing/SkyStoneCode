package org.firstinspires.ftc.teamcode.SkyStone;


import com.qualcomm.hardware.bosch.BNO055IMU;
import com.qualcomm.hardware.bosch.JustLoggingAccelerationIntegrator;
import com.qualcomm.hardware.modernrobotics.ModernRoboticsI2cGyro;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.DistanceSensor;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.IntegratingGyroscope;
import com.qualcomm.robotcore.hardware.Servo;

import org.firstinspires.ftc.robotcore.external.navigation.Acceleration;
import org.firstinspires.ftc.robotcore.external.navigation.Orientation;


public class SkyStoneHardware {

    /* Public OpMode members. */
    public BNO055IMU imu;
    public Orientation angles;
    public Acceleration gravity;

    public DcMotor left_front;
    public DcMotor right_front;
    public DcMotor left_back;
    public DcMotor right_back;
    public DcMotor Lencoder;
    public DcMotor Rencoder;
    public DcMotor Hencoder;
    public DcMotor linear_slide;
    public DcMotor flipBackRight;
    public DcMotor flipBackLeft;
    public DcMotor skystoneArm;

    public Servo front_claw;
    public Servo grabServo;

    public DistanceSensor rightSideDist;
    public DistanceSensor leftSideDist;
    public DistanceSensor frontLeftDist;
    public DistanceSensor frontRightDist;

    public IntegratingGyroscope gyro;
    public ModernRoboticsI2cGyro modernRoboticsI2cGyro;





    /* local OpMode members. */
    HardwareMap hwMap =  null;

    /* Constructor */
    public SkyStoneHardware(){

    }

    /* Initialize standard Hardware interfaces */
    public void init(HardwareMap ahwMap) {
        // Save reference to Hardware map
        hwMap = ahwMap;


        modernRoboticsI2cGyro = hwMap.get(ModernRoboticsI2cGyro.class, "gyro");
        gyro = (IntegratingGyroscope)modernRoboticsI2cGyro;

        modernRoboticsI2cGyro.calibrate();



        // Initialize the IMU
        BNO055IMU.Parameters parameters = new BNO055IMU.Parameters();
        parameters.angleUnit = BNO055IMU.AngleUnit.DEGREES;
        parameters.accelUnit = BNO055IMU.AccelUnit.METERS_PERSEC_PERSEC;
        parameters.calibrationDataFile = "BNO055IMUCalibration.json"; // see the calibration sample opmode
        parameters.loggingEnabled = true;
        parameters.loggingTag = "IMU";
        parameters.accelerationIntegrationAlgorithm = new JustLoggingAccelerationIntegrator();
        imu = hwMap.get(BNO055IMU.class, "imu");
        imu.initialize(parameters);


        // Initialize the Motors and encoders
        left_front = hwMap.get(DcMotor.class, "left_front");
        right_front = hwMap.get(DcMotor.class, "right_front");
        left_back = hwMap.get(DcMotor.class, "left_back");
        right_back = hwMap.get(DcMotor.class, "right_back");
        Lencoder = hwMap.get(DcMotor.class,"left_front");
        Rencoder = hwMap.get(DcMotor.class, "right_front");
        Hencoder = hwMap.get(DcMotor.class, "right_back");

        flipBackLeft = hwMap.get(DcMotor.class, "left_flip");
        flipBackRight = hwMap.get(DcMotor.class, "right_flip");

        linear_slide = hwMap.get(DcMotor.class, "linear_slide");

        skystoneArm = hwMap.get(DcMotor.class,"skystoneArm");

        front_claw = hwMap.get(Servo.class, "frontclaw");

        grabServo = hwMap.get(Servo.class,"grabServo");


        // Initialize the sensors
        rightSideDist = hwMap.get(DistanceSensor.class,"rightSideDist");
        leftSideDist = hwMap.get(DistanceSensor.class,"leftSideDist");
        frontLeftDist = hwMap.get(DistanceSensor.class,"frontLeftDist");
        frontRightDist = hwMap.get(DistanceSensor.class,"frontRightDist");


        right_front.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        right_back.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        left_front.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        left_back.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        right_front.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        right_back.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        left_front.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        left_back.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        left_back.setDirection(DcMotor.Direction.REVERSE);
        right_front.setDirection(DcMotor.Direction.REVERSE);
        right_back.setDirection(DcMotor.Direction.REVERSE);


        Lencoder.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        Rencoder.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        Hencoder.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        Lencoder.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        Rencoder.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        Hencoder.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);


        flipBackRight.setDirection(DcMotorSimple.Direction.REVERSE);
        flipBackLeft.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        flipBackRight.setMode(DcMotor.RunMode.RUN_USING_ENCODER);


        linear_slide.setDirection(DcMotorSimple.Direction.REVERSE);
        linear_slide.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        linear_slide.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        //linear_slide.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);


        //skystoneArm.setDirection(DcMotorSimple.Direction.REVERSE);
        skystoneArm.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        skystoneArm.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);





        // Set all motors to zero power
        driveAll(0);
        flipBackLeft.setPower(0);
        flipBackRight.setPower(0);
        linear_slide.setPower(0);
        skystoneArm.setPower(0);
    }




    public void driveEach(double lf, double lb, double rf, double rb){

        left_front.setPower(lf);
        left_back.setPower(lb);
        right_front.setPower(rf);
        right_back.setPower(rb);
    }

    public void driveAll(double power){
        left_front.setPower(power);
        left_back.setPower(power);
        right_front.setPower(power);
        right_back.setPower(power);
    }
}

