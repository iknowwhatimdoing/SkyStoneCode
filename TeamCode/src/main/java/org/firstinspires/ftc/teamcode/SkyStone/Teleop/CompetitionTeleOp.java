package org.firstinspires.ftc.teamcode.SkyStone.Teleop;

import android.accounts.AuthenticatorDescription;

import com.qualcomm.hardware.bosch.BNO055IMU;
import com.qualcomm.hardware.bosch.JustLoggingAccelerationIntegrator;
import com.qualcomm.robotcore.eventloop.opmode.Disabled;
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
import org.opencv.core.Mat;


@TeleOp(name = "Competition 2019")

public class CompetitionTeleOp extends OpMode {

    SkyStoneHardware robot = new SkyStoneHardware();

    double assist = 0;
    double current = 0;
    boolean getCurrent = false;
    boolean accurateSpeed = false;
    double turnDivider = 1;
    double speedDivider = 1;
    double legosTall = 0;
    boolean resetSlide = false;
    boolean recalibrate = false;

    boolean linearSlideDone = true;


    boolean clawOpen = true;
    ElapsedTime clickTime = new ElapsedTime();


    Servo odometryServo;
    Servo capstone;


    ElapsedTime timeBetweenCapDrop = new ElapsedTime();
    boolean capdropperDown = false;

    boolean flippedBack = false;
    ElapsedTime timeForFlip = new ElapsedTime();

    //ElapsedTime runTime = new ElapsedTime();


    @Override
    public void init() {
        robot.init(hardwareMap, false);
        odometryServo = hardwareMap.get(Servo.class, "odometryServo");

        capstone = hardwareMap.get(Servo.class, "capstone");


    }

    @Override
    public void start() {

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

        odometryServo.setPosition(1);
        robot.front_claw.setPosition(.71428);


    }

    @Override
    public void loop() {

        robot.angles = robot.imu.getAngularOrientation(AxesReference.INTRINSIC, AxesOrder.ZYX, AngleUnit.DEGREES);




        /*
        telemetry.addData("left", robot.Lencoder.getCurrentPosition());
        telemetry.addData("right", robot.Rencoder.getCurrentPosition());
        telemetry.addData("horizontal", robot.Hencoder.getCurrentPosition());
        telemetry.update();




        telemetry.addLine("Left");
        telemetry.addData("range", String.format("%.01f in", robot.leftSideDist.getDistance(DistanceUnit.INCH)));
        telemetry.update();

         */


        //flip the linear slide up if it didn't flip in autonomous. Must hold the button for 2 seconds
        ElapsedTime holdToReset = new ElapsedTime();
        if (gamepad2.back) {

            while (!recalibrate) {
                robot.flipBackRight.setTargetPosition(300);
                robot.flipBackLeft.setTargetPosition(300);
                robot.flipBackLeft.setMode(DcMotor.RunMode.RUN_TO_POSITION);
                robot.flipBackRight.setMode(DcMotor.RunMode.RUN_TO_POSITION);
                robot.flipBackRight.setPower(.5);
                robot.flipBackLeft.setPower(.5);

                if (!robot.flipBackLeft.isBusy()) {
                    robot.flipBackLeft.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
                    robot.flipBackRight.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
                    recalibrate = true;
                }
            }
        } else {
            holdToReset.reset();
        }


        double driveforward = -gamepad1.left_stick_y;
        double driveSideways = gamepad1.left_stick_x;
        double turn = gamepad1.right_stick_x;




        /*
        if (gamepad1.right_bumper) {
            robot.driveEach(.1, -.4, -.1, .4);
        } else if (gamepad1.left_bumper) {
            robot.driveEach(-.1, .4, .1, -.4);
        }

         */


        //slow speed
        if (gamepad1.right_bumper) {
            speedDivider = 4;
            turnDivider = 2;
        } else if (gamepad1.left_bumper) {
            speedDivider = 1;
            turnDivider = 1;
        }


        //-----------------------------
        //strafing
        //-----------------------------
        if (gamepad1.dpad_left || gamepad1.dpad_right) {

            if (!getCurrent) {
                current = robot.angles.firstAngle;
                getCurrent = true;
            }

            robot.angles = robot.imu.getAngularOrientation(AxesReference.INTRINSIC, AxesOrder.ZYX, AngleUnit.DEGREES);
            double difference = current - robot.angles.firstAngle;
            assist = difference / 15;

            if (gamepad1.dpad_left) {
                driveSideways = -1;
            } else if (gamepad1.dpad_right) {
                driveSideways = 1;
            }
            double lfpower = driveforward / speedDivider + turn / turnDivider + driveSideways / speedDivider - assist;
            double lbpower = driveforward / speedDivider + turn / turnDivider - driveSideways / speedDivider - assist;
            double rfpower = driveforward / speedDivider - turn / turnDivider - driveSideways / speedDivider + assist;
            double rbpower = driveforward / speedDivider - turn / turnDivider + driveSideways / speedDivider + assist;

            robot.driveEach(lfpower, lbpower, rfpower, rbpower);
        }


        if (gamepad1.dpad_up || gamepad1.dpad_down) {

            if (!getCurrent) {
                current = robot.angles.firstAngle;
                getCurrent = true;
            }

            double direction = 1;

            if (gamepad1.dpad_down) {
                direction = -1;
            }

            robot.angles = robot.imu.getAngularOrientation(AxesReference.INTRINSIC, AxesOrder.ZYX, AngleUnit.DEGREES);
            double difference = current - robot.angles.firstAngle;
            assist = difference / 40;


            robot.driveEach((1 * direction) / speedDivider - assist, (1 * direction) / speedDivider - assist, (1 * direction) / speedDivider + assist, (1 * direction) / speedDivider + assist);
        }
        if (!gamepad1.dpad_up && !gamepad1.dpad_down && !gamepad1.dpad_up && !gamepad1.dpad_down) {
            getCurrent = false;
        }


        //-----------------------------
        //driving
        //-----------------------------
        if (!gamepad1.dpad_left && !gamepad1.dpad_right && !gamepad1.dpad_down && !gamepad1.dpad_up) {
            double lfpower = driveforward / speedDivider + turn / turnDivider + driveSideways / speedDivider;
            double lbpower = driveforward / speedDivider + turn / turnDivider - driveSideways / speedDivider;
            double rfpower = driveforward / speedDivider - turn / turnDivider - driveSideways / speedDivider;
            double rbpower = driveforward / speedDivider - turn / turnDivider + driveSideways / speedDivider;

            robot.driveEach(lfpower, lbpower, rfpower, rbpower);
        }


        //-----------------------------
        //flip back
        //-----------------------------
        if (!(robot.linear_slide.getCurrentPosition() > 5)) {
            if (gamepad2.dpad_right && timeForFlip.seconds() > .5) {

                if (flippedBack) {
                    robot.flipBackLeft.setTargetPosition(0);
                    robot.flipBackRight.setTargetPosition(0);
                    flippedBack = false;
                } else if (!flippedBack) {
                    robot.flipBackLeft.setTargetPosition(-300);
                    robot.flipBackRight.setTargetPosition(-300);
                    flippedBack = true;
                }

                timeForFlip.reset();
            }

            robot.flipBackLeft.setMode(DcMotor.RunMode.RUN_TO_POSITION);
            robot.flipBackRight.setMode(DcMotor.RunMode.RUN_TO_POSITION);

            if ((robot.flipBackLeft.isBusy() || robot.flipBackRight.isBusy())) {
                double flipPower = .3;
                robot.flipBackRight.setPower(flipPower);
                robot.flipBackLeft.setPower(flipPower);

            }
        }


        //-----------------------------
        //claw
        //-----------------------------
        //robot.front_claw.setPosition((1 - gamepad1.right_trigger) / 1.4);

        if (gamepad1.right_trigger > 0 && clickTime.milliseconds() > 500) {
            if (clawOpen) {
                robot.front_claw.setPosition(0);
                clawOpen = false;
            } else if (!clawOpen) {
                robot.front_claw.setPosition(.71428);
                clawOpen = true;
            }
            clickTime.reset();
        }


        //drop capstone
        if (timeBetweenCapDrop.seconds() > .5) {
            if (gamepad2.y) {
                if (capdropperDown) {
                    capstone.setPosition(.87);
                    capdropperDown = false;
                } else if (!capdropperDown) {
                    capstone.setPosition(.5);
                    capdropperDown = true;
                }
                timeBetweenCapDrop.reset();
            }
        }



        /*
        if (timeBetweenPress.seconds() > .5) {
            if (gamepad2.dpad_up) {
                if (legosTall < 5) {
                    legosTall += 1;
                }
                timeBetweenPress.reset();
            } else if (gamepad2.dpad_down) {
                if (legosTall > 0) {
                    legosTall -= 1;
                }
                timeBetweenPress.reset();
            }
        }

        telemetry.addData("Legos Tall", legosTall);
        telemetry.update();


        if (gamepad2.y || !linearSlideDone){
            robot.linear_slide.setMode(DcMotor.RunMode.RUN_USING_ENCODER);

            int targPos = (int) (legosTall * 800);
            robot.linear_slide.setTargetPosition(targPos);

            robot.linear_slide.setMode(DcMotor.RunMode.RUN_TO_POSITION);

            if (robot.linear_slide.isBusy()) {
                linearSlideDone = false;
                robot.linear_slide.setPower(1);
            }else if (!robot.linear_slide.isBusy()){
                linearSlideDone = true;
                robot.linear_slide.setPower(0);
            }
        }

         */


        //-----------------------------
        //linear slide
        //-----------------------------
        if (gamepad2.right_trigger > 0 && !resetSlide) {

            robot.linear_slide.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);

            if (robot.linear_slide.getCurrentPosition() < 3888) {
                robot.linear_slide.setPower(gamepad2.right_trigger);

            } else {
                robot.linear_slide.setPower(0);
            }

        } else if (gamepad2.left_trigger > 0 && !resetSlide) {

            robot.linear_slide.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);

            if (robot.linear_slide.getCurrentPosition() > 30) {
                robot.linear_slide.setPower(-gamepad2.left_trigger);
            } else {
                robot.linear_slide.setPower(0);
            }

        } else {
            if (resetSlide == false) {
                robot.linear_slide.setPower(0);
            }
        }

        //reset linear slide
        if (gamepad2.x || resetSlide) {
            resetSlide = true;
            robot.linear_slide.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
            robot.linear_slide.setTargetPosition(0);
            robot.linear_slide.setMode(DcMotor.RunMode.RUN_TO_POSITION);

            double distance = Math.abs(0 - robot.linear_slide.getCurrentPosition());
            double speed = distance;
            if (distance < 50) {
                speed /= 100;
            } else {
                speed /= 20;
            }


            robot.linear_slide.setPower(speed);


            if (distance < 5 || !robot.linear_slide.isBusy() || gamepad2.b) {
                robot.linear_slide.setPower(0);
                resetSlide = false;
                robot.linear_slide.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
            }
        }

        //fix flip back jiggle
        if (gamepad2.dpad_left) {
            if (robot.flipBackLeft.getCurrentPosition() < 10) {
                robot.flipBackRight.setPower(0);
                robot.flipBackLeft.setPower(0);
                robot.flipBackLeft.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
                robot.flipBackRight.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
            }
        }


        if (gamepad2.a) {
            speedDivider = 3;
            turnDivider = 1.5;
        }


    }


    public void PID(DcMotor movingMotor) {

        if (Math.abs(movingMotor.getCurrentPosition() - movingMotor.getTargetPosition()) >= 5) {
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
                movingMotor.setPower(-1 * (Kp * error + Ki * integral));
                lastError = error;
                timer.reset();

            }
        }
    }

}


