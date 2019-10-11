package org.firstinspires.ftc.teamcode.Programmers.Bryce;

import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.Servo;

@TeleOp(name = "Outreach Movement")

public class Bryce__OutReachDrive extends OpMode {
    DcMotor Rear_left_motor;
    DcMotor Rear_right_motor;
    Servo Servo_1;
    DcMotor Arm_Motor;


    @Override
    public void init() {
        Rear_left_motor = hardwareMap.get(DcMotor.class, "left motor");
        Rear_right_motor = hardwareMap.get(DcMotor.class, "right motor");
        //Servo_1 = hardwareMap.get(Servo.class, "servo");
        //Arm_Motor = hardwareMap.get(DcMotor.class, "arm motor");

        Rear_right_motor.setDirection(DcMotor.Direction.REVERSE);

    }

    @Override
    public void loop() {

        double drive = gamepad1.left_stick_y;
        double turn = gamepad1.right_stick_x;

        double leftPower = drive + turn;
        double rightpower = drive - turn;


        Rear_left_motor.setPower(leftPower);
        Rear_right_motor.setPower(rightpower);


    }
}






