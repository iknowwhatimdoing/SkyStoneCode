package org.firstinspires.ftc.teamcode.SkyStone.Teleop;

import android.accounts.AuthenticatorDescription;

import com.qualcomm.hardware.bosch.BNO055IMU;
import com.qualcomm.hardware.bosch.JustLoggingAccelerationIntegrator;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.DistanceSensor;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.util.ElapsedTime;
import com.qualcomm.robotcore.util.Range;

import org.firstinspires.ftc.robotcore.external.navigation.Acceleration;
import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.robotcore.external.navigation.AxesOrder;
import org.firstinspires.ftc.robotcore.external.navigation.AxesReference;
import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit;
import org.firstinspires.ftc.robotcore.external.navigation.Orientation;
import org.firstinspires.ftc.robotcore.external.navigation.Position;
import org.firstinspires.ftc.robotcore.external.navigation.Velocity;
import org.firstinspires.ftc.teamcode.SkyStone.SkyStoneHardware;


@TeleOp(name = "Competition 2019")

public class CompetitionTeleOp extends OpMode {

    SkyStoneHardware robot = new SkyStoneHardware();

    double assist = 0;
    double current = 0;
    boolean getCurrent = false;
    boolean accurateSpeed = false;

    ElapsedTime timeBetweenPress = new ElapsedTime();


    @Override
    public void init() {
        robot.init(hardwareMap);
    }

    @Override
    public void start() {
        robot.skystoneArm.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);

        robot.linear_slide.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        robot.linear_slide.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);

        robot.flipBackLeft.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        robot.flipBackRight.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        robot.imu.startAccelerationIntegration(new Position(), new Velocity(), 1000);

        robot.flipBackLeft.setTargetPosition(0);
        robot.flipBackRight.setTargetPosition(0);
        robot.flipBackLeft.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        robot.flipBackRight.setMode(DcMotor.RunMode.RUN_TO_POSITION);

        robot.flipBackLeft.setPower(1);
        robot.flipBackRight.setPower(1);
    }

    @Override
    public void loop() {

        robot.angles = robot.imu.getAngularOrientation(AxesReference.INTRINSIC, AxesOrder.ZYX, AngleUnit.DEGREES);


        /*
        telemetry.addLine("Right");
        telemetry.addData("range", String.format("%.01f cm", rightSideDist.getDistance(DistanceUnit.CM)));
        telemetry.addData("range", String.format("%.01f in", rightSideDist.getDistance(DistanceUnit.INCH)));

        telemetry.addLine("Left");
        telemetry.addData("range", String.format("%.01f cm", leftSideDist.getDistance(DistanceUnit.CM)));
        telemetry.addData("range", String.format("%.01f in", leftSideDist.getDistance(DistanceUnit.INCH)));
        telemetry.update();


         */

        /*
        telemetry.addData("lencoder ", Lencoder.getCurrentPosition());
        telemetry.addData("rencoder", Rencoder.getCurrentPosition());
        telemetry.addData("hencoder", Hencoder.getCurrentPosition());
        telemetry.update();


         */



        double driveforward = -gamepad1.left_stick_y;
        //double leftSidePower = -gamepad1.left_stick_y;
        //double rightSidePower = -gamepad1.right_stick_y;
        double driveSideways = gamepad1.left_stick_x;
        double turn = gamepad1.right_stick_x;

        if (gamepad2.a){
            robot.skystoneArm.setTargetPosition(210);
            robot.skystoneArm.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        }else if (gamepad2.y){
            robot.skystoneArm.setTargetPosition(0);
        }

        PID(robot.skystoneArm);



        if (gamepad2.x){
            robot.grabServo.setPosition(.75);
        }else if (gamepad2.b) {
            robot.grabServo.setPosition(1);
        }







        if (gamepad1.right_bumper){
            robot.driveEach(.1,-.4,-.1,.4);
        }else if (gamepad1.left_bumper){
            robot.driveEach(-.1,.4,.1,-.4);
        }




        //slow speed
        if (timeBetweenPress.seconds() > .5) {
            if (gamepad1.left_stick_button) {
                if (!accurateSpeed) {
                    accurateSpeed = true;
                } else if (accurateSpeed) {
                    accurateSpeed = false;
                }
                timeBetweenPress.reset();
            }
        }


        //strafing
        if (gamepad1.dpad_left || gamepad1.dpad_right) {

            if (!getCurrent) {
                current = robot.angles.firstAngle;
                getCurrent = true;
            }

            double turnDivider = 1;
            double speedDivider = 1;
            if (accurateSpeed) {
                turnDivider = 4;
                speedDivider = 4;
            }

            robot.angles = robot.imu.getAngularOrientation(AxesReference.INTRINSIC, AxesOrder.ZYX, AngleUnit.DEGREES);
            double difference = current - robot.angles.firstAngle;

            assist = difference / 40;
            driveSideways = -.5;
            double lfpower = driveforward / speedDivider + turn / turnDivider + driveSideways / speedDivider - assist;
            double lbpower = driveforward / speedDivider + turn / turnDivider - driveSideways / speedDivider - assist;
            double rfpower = driveforward / speedDivider - turn / turnDivider - driveSideways / speedDivider + assist;
            double rbpower = driveforward / speedDivider - turn / turnDivider + driveSideways / speedDivider + assist;


            robot.driveEach(lfpower, lbpower, rfpower, rbpower);
        }


        if (!gamepad1.dpad_left && !gamepad1.dpad_right) {
            getCurrent = false;
        }


        //drivng
        if (!gamepad1.dpad_left && !gamepad1.dpad_right&& !gamepad1.left_bumper && !gamepad1.right_bumper) {

                double turnDivider = 1;
                double speedDivider = 1;
                if (accurateSpeed) {
                    turnDivider = 4;
                    speedDivider = 4;
                }
                double lfpower = driveforward / speedDivider + turn / turnDivider + driveSideways / speedDivider;
                double lbpower = driveforward / speedDivider + turn / turnDivider - driveSideways / speedDivider;
                double rfpower = driveforward / speedDivider - turn / turnDivider - driveSideways / speedDivider;
                double rbpower = driveforward / speedDivider - turn / turnDivider + driveSideways / speedDivider;

                robot.driveEach(lfpower, lbpower, rfpower, rbpower);

        }


        //flip back
        if (!(robot.linear_slide.getCurrentPosition() > 5)) {
            if (gamepad2.dpad_down) {
                robot.flipBackLeft.setTargetPosition(-300);
                robot.flipBackRight.setTargetPosition(-300);

            } else if (gamepad2.dpad_up) {
                robot.flipBackLeft.setTargetPosition(0);
                robot.flipBackRight.setTargetPosition(0);


            }

            robot.flipBackLeft.setMode(DcMotor.RunMode.RUN_TO_POSITION);
            robot.flipBackRight.setMode(DcMotor.RunMode.RUN_TO_POSITION);

            if ((robot.flipBackLeft.isBusy() || robot.flipBackRight.isBusy())) {

                double flipPower = .3;

                robot.flipBackRight.setPower(flipPower);
                robot.flipBackLeft.setPower(flipPower);

            }
        }


        //claw

        robot.front_claw.setPosition(1 - (gamepad2.right_trigger/3.125));
        //telemetry.addData("pos", front_claw.getPosition());
        //telemetry.update();


        //linear slide
        if (gamepad1.left_trigger > 0) {

            if (robot.linear_slide.getCurrentPosition() < 3888){
                robot.linear_slide.setPower(gamepad1.left_trigger);
            }else{
                robot.linear_slide.setPower(0);
            }

        } else if (gamepad1.right_trigger > 0) {

            if (robot.linear_slide.getCurrentPosition() >30) {
                robot.linear_slide.setPower(-gamepad1.right_trigger);
            }else{
                robot.linear_slide.setPower(0);
            }

        }else{
            robot.linear_slide.setPower(0);
        }


    }


    public void PID(DcMotor movingMotor){

        if (Math.abs(movingMotor.getCurrentPosition()-movingMotor.getTargetPosition())>=5 ) {
            double targetPosition = movingMotor.getTargetPosition();
            double integral = 0;
            ElapsedTime timer = new ElapsedTime();

            double error = movingMotor.getCurrentPosition() - targetPosition;
            double lastError = 0;
            double Kp = 0.001;
            double Ki = (0.00012);
            double Kd = 0;

            if (Math.abs(error) >= 5) {
                error = movingMotor.getCurrentPosition() - targetPosition;
                double deltaError = lastError - error;
                integral += deltaError * timer.time();
                double derivative = deltaError / timer.time();
                movingMotor.setPower(-1*(Kp * error + Ki * integral));
                lastError = error;
                timer.reset();

            }

        }
    }


}


