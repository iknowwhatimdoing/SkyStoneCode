package org.firstinspires.ftc.teamcode.SkyStone.Teleop.Outreach;

import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.util.ElapsedTime;
import com.qualcomm.robotcore.util.Range;


@TeleOp(name = "OutReach Drive (Competition bot)")

public class OutreachWithTeamBot2019 extends OpMode {



    ElapsedTime slowDown = new ElapsedTime();
    boolean slideup = false;
    double power = 0;
    boolean lock_slide = true;
    boolean limit_speed = true;


    DcMotor left_front;
    DcMotor right_front;
    DcMotor left_back;
    DcMotor right_back;

    DcMotor linear_slide;

    DcMotor flipBackRight;
    DcMotor flipBackLeft;


    @Override
    public void init() {
        left_front = hardwareMap.get(DcMotor.class, "left_front");
        right_front = hardwareMap.get(DcMotor.class, "right_front");
        left_back = hardwareMap.get(DcMotor.class, "left_back");
        right_back = hardwareMap.get(DcMotor.class, "right_back");


        flipBackLeft = hardwareMap.get(DcMotor.class, "left_flip");
        flipBackRight = hardwareMap.get(DcMotor.class, "right_flip");

        flipBackRight.setDirection(DcMotorSimple.Direction.REVERSE);

        flipBackRight.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        flipBackLeft.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);

        linear_slide = hardwareMap.get(DcMotor.class, "linear_slide");

        linear_slide.setDirection(DcMotorSimple.Direction.REVERSE);
        linear_slide.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        linear_slide.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);


        left_back.setDirection(DcMotor.Direction.REVERSE);
        right_front.setDirection(DcMotor.Direction.REVERSE);
        right_back.setDirection(DcMotor.Direction.REVERSE);

    }

    @Override
    public void loop() {

        if (gamepad2.dpad_up) {
            lock_slide=false;
            telemetry.addLine("Slide: unlocked");
        }

        if (gamepad2.dpad_down) {
            lock_slide=true;
            telemetry.addLine("Slide: locked");

        }
        if (gamepad2.dpad_left) {
            limit_speed = true;
            telemetry.addLine("Speed: half speed");

        }

        if (gamepad2.dpad_right) {
            limit_speed=false;
            telemetry.addLine("Speed: full speed");

        }
        telemetry.update();

        double drive = -gamepad1.left_stick_y;
        double turn = gamepad1.right_stick_x;

        double leftPower = drive + turn;
        double rightpower = drive - turn;

        if (!slideup) {
            if (!limit_speed) {
                setPowerAll(leftPower, leftPower, rightpower, rightpower);
            } else {
                drive = drive/2;
                leftPower = drive + turn;
                rightpower = drive - turn;
                setPowerAll(leftPower, leftPower, rightpower, rightpower);
            }

            /*
            if (gamepad1.dpad_down) {
                flipBackLeft.setPower(-.4);
                flipBackRight.setPower(-.4);
            } else if (gamepad1.dpad_up) {
                flipBackLeft.setPower(.4);
                flipBackRight.setPower(.4);
            } else if (!gamepad1.dpad_up && !gamepad1.dpad_down) {
                flipBackLeft.setPower(0);
                flipBackRight.setPower(0);
            }
             */
        }


        if (!lock_slide) {
            if (gamepad1.y && !slideup) {

                if (!(Math.abs(linear_slide.getCurrentPosition() - linear_slide.getTargetPosition()) > 5)) {
                    power = 0;
                }
                linear_slide.setTargetPosition(3840);
                linear_slide.setMode(DcMotor.RunMode.RUN_TO_POSITION);

                slideup = true;


            } else if (gamepad1.a && slideup) {
                if (!(Math.abs(linear_slide.getCurrentPosition() - linear_slide.getTargetPosition()) > 5)) {
                    power = 0;
                }

                linear_slide.setTargetPosition(0);
                linear_slide.setMode(DcMotor.RunMode.RUN_TO_POSITION);


                slideup = false;
            }

            if (linear_slide.isBusy()) {
                if ((Math.abs(linear_slide.getCurrentPosition() - linear_slide.getTargetPosition()) > (linear_slide.getTargetPosition() / 4)) && power <= .8) {
                    power += .02;
                } else if ((Math.abs(linear_slide.getCurrentPosition() - linear_slide.getTargetPosition()) > 5) && power >= 0) {
                    power -= .04;
                }
                linear_slide.setPower(Range.clip(power, -.8, .8));

            } else if (!linear_slide.isBusy()) {
                power = 0;
            }

        }
    }

    public void setPowerAll(double lf, double lb, double rf, double rb) {
        left_front.setPower(lf);
        left_back.setPower(lb);
        right_back.setPower(rb);
        right_front.setPower(rf);


    }
}






