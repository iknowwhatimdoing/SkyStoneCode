package org.firstinspires.ftc.teamcode.SkyStone.Tests;

import com.qualcomm.hardware.modernrobotics.ModernRoboticsI2cGyro;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.CRServo;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.Servo;

import org.firstinspires.ftc.robotcore.external.ClassFactory;
import org.opencv.core.Mat;


@Autonomous(name = "Test Program")
public class Tests extends LinearOpMode {

    final double COUNTS_PER_INCH = 309.023;


    DcMotor leftDrive;
    DcMotor rightDrive;

    Servo Claw;
    CRServo smallMotor;

    ModernRoboticsI2cGyro gyro;



    @Override
    public void runOpMode() throws InterruptedException {

        leftDrive = hardwareMap.get(DcMotor.class, "left");
        rightDrive = hardwareMap.get(DcMotor.class, "right");
        Claw = hardwareMap.get(Servo.class, "claw");
        smallMotor =hardwareMap.get(CRServo.class, "smallMotor");

        gyro = hardwareMap.get(ModernRoboticsI2cGyro.class, "gyro");

        waitForStart();

        moveDistance(5, .5);
        turnDegree(90,.8,5);
        turnDegree(-90,.8,5);
        moveDistance(-10,.5);
        turnDegree(90,.5,5);
        moveDistance(5,4);




    }



    public void moveDistance(double inches, double speed){
        double ticks = inches * COUNTS_PER_INCH;

        rightDrive.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        leftDrive.setMode(DcMotor.RunMode.RUN_USING_ENCODER);

        rightDrive.setTargetPosition((int) ticks);
        leftDrive.setTargetPosition((int) ticks);

        leftDrive.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        rightDrive.setMode(DcMotor.RunMode.RUN_TO_POSITION);

        while(opModeIsActive() && rightDrive.isBusy() || leftDrive.isBusy()){
            rightDrive.setPower(speed);
            leftDrive.setPower(speed);
        }
        rightDrive.setPower(0);
        leftDrive.setPower(0);

    }

    public void turnDegree(double angle, double speed, double offset){
        gyro.resetZAxisIntegrator();

        double rotation = gyro.getIntegratedZValue();

        double leftSpeed = speed * Math.signum(angle);
        double rightSpeed = -speed * Math.signum(angle);

        leftDrive.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        rightDrive.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);

        while(opModeIsActive() && Math.abs(rotation - angle) >= offset){
            rotation = gyro.getIntegratedZValue();

            if (Math.abs(rotation) > Math.abs((angle * .7))){
                leftSpeed = -.2 * Math.signum(angle);
                rightSpeed = .2 * Math.signum(angle);
            }else if (angle > 0 && rotation > angle){
                break;
            }else if (angle < 0 && rotation < angle){
                break;
            }else{
                leftSpeed = speed * -1 * Math.signum(angle);
                rightSpeed = speed * Math.signum(angle);
            }

            leftDrive.setPower(leftSpeed);
            rightDrive.setPower(rightSpeed);
        }
        leftDrive.setPower(0);
        rightDrive.setPower(0);

    }
}
