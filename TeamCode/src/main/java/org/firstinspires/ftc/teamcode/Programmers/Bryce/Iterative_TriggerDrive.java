package org.firstinspires.ftc.teamcode.Programmers.Bryce;

import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.util.ElapsedTime;

@TeleOp(name="Iterative OpMode", group="Iterative Opmode")
@Disabled
public class Iterative_TriggerDrive extends OpMode
{

    private ElapsedTime runtime = new ElapsedTime();
    private DcMotor leftDrive;
    private DcMotor rightDrive;


    @Override
    public void init() {
        telemetry.addData("Status", "Initialized");


        leftDrive  = hardwareMap.get(DcMotor.class, "left_drive");
        rightDrive = hardwareMap.get(DcMotor.class, "right_drive");


        leftDrive.setDirection(DcMotor.Direction.FORWARD);
        rightDrive.setDirection(DcMotor.Direction.REVERSE);


        telemetry.addData("Status", "Initialized");
    }


    @Override
    public void init_loop() {
    }


    @Override
    public void start() {
        runtime.reset();
    }


    @Override
    public void loop() {

        double leftPower;
        double rightPower;

        leftPower = gamepad1.right_trigger + gamepad1.left_trigger;
        rightPower = gamepad1.right_trigger - gamepad1.left_trigger;


        leftDrive.setPower(leftPower);
        rightDrive.setPower(rightPower);

        telemetry.addData("Motors", "left (%.2f), right (%.2f)", leftPower, rightPower);
        telemetry.addData("Status", "Run Time: " + runtime.toString());

    }


    @Override
    public void stop() {
    }

}