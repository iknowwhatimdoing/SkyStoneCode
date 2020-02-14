package org.firstinspires.ftc.teamcode.SkyStone.Teleop.Outreach;

import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorSimple;


@TeleOp(name = "Outreach Bot")
public class OutreachWithOutreachBot2019 extends OpMode {


    DcMotor Left;
    DcMotor Right;
    DcMotor DumperArm;


    @Override
    public void init() {


        Left = hardwareMap.get(DcMotor.class, "Left");
        Right = hardwareMap.get(DcMotor.class, "Right");
        DumperArm = hardwareMap.get(DcMotor.class, "DumpArm");
        DumperArm.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        Left.setDirection(DcMotorSimple.Direction.REVERSE);

    }


    @Override
    public  void start(){


        DumperArm.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
    }



    @Override
    public void loop() {

        double Move = -gamepad1.right_stick_y;
        double Turn = gamepad1.left_stick_x;

        Left.setPower(Move + Turn);
        Right.setPower(Move - Turn);

        telemetry.addData("Pose", DumperArm.getCurrentPosition());
        telemetry.update();
    }
}
