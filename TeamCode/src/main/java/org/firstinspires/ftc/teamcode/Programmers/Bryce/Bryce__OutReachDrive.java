package org.firstinspires.ftc.teamcode.Programmers.Bryce;

import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.Servo;

@TeleOp(name = "Outreach Movement")
@Disabled

public class Bryce__OutReachDrive extends OpMode {

    DcMotor Rear_left_motor;
    DcMotor Rear_right_motor;
    //Servo Servo_1;
    DcMotor Arm_Motor;


    @Override
    public void init() {
        Rear_left_motor = hardwareMap.get(DcMotor.class, "left_motor");
        Rear_right_motor = hardwareMap.get(DcMotor.class, "right_motor");
        //Servo_1 = hardwareMap.get(Servo.class, "servo");
        Arm_Motor = hardwareMap.get(DcMotor.class, "arm_motor");
        Arm_Motor.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);

        Arm_Motor.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        Rear_right_motor.setDirection(DcMotor.Direction.REVERSE);

    }

    @Override
    public void loop() {

        double drive = gamepad1.left_stick_y;
        double turn = gamepad1.right_stick_x;

        double leftPower = drive - turn;
        double rightpower = drive + turn;


        Rear_left_motor.setPower(leftPower);
        Rear_right_motor.setPower(rightpower);


        if (gamepad2.right_trigger > 0) {
            Arm_Motor.setTargetPosition(-280);
            Arm_Motor.setMode(DcMotor.RunMode.RUN_TO_POSITION);
            Arm_Motor.setPower(0.7);
        }
        if (gamepad2.left_trigger > 0) {
            Arm_Motor.setTargetPosition(0);
            Arm_Motor.setMode(DcMotor.RunMode.RUN_TO_POSITION);
            Arm_Motor.setPower(0.7);
        }


    }
}






