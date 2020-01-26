package org.firstinspires.ftc.teamcode.SkyStone.Teleop;

import android.accounts.AuthenticatorDescription;

import com.qualcomm.hardware.bosch.BNO055IMU;
import com.qualcomm.hardware.bosch.JustLoggingAccelerationIntegrator;
import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.DigitalChannel;
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


@TeleOp(name = "Competition 2019 (Fast but Inaccurate)")

public class CompetitionTeleOp_QuickStart extends OpMode {




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

    DigitalChannel frontTouch;
    Servo foundationClampLeft;
    Servo foundationClampRight;
    Servo capper;



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

    public Servo front_claw;
    public Servo grabServo;



    @Override
    public void init() {
        left_front = hardwareMap.get(DcMotor.class, "left_front");
        right_front = hardwareMap.get(DcMotor.class, "right_front");
        left_back = hardwareMap.get(DcMotor.class, "left_back");
        right_back = hardwareMap.get(DcMotor.class, "right_back");
        Lencoder = hardwareMap.get(DcMotor.class, "left_front");
        Rencoder = hardwareMap.get(DcMotor.class, "right_front");
        Hencoder = hardwareMap.get(DcMotor.class, "right_back");

        flipBackLeft = hardwareMap.get(DcMotor.class, "left_flip");
        flipBackRight = hardwareMap.get(DcMotor.class, "right_flip");

        linear_slide = hardwareMap.get(DcMotor.class, "linear_slide");


        front_claw = hardwareMap.get(Servo.class, "frontclaw");

        grabServo = hardwareMap.get(Servo.class, "grabServo");
        capper = hardwareMap.get(Servo.class, "capstone");
        odometryServo = hardwareMap.get(Servo.class, "odometryServo");




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


        frontTouch = hardwareMap.get(DigitalChannel.class, "sensor_digital");
        frontTouch.setMode(DigitalChannel.Mode.INPUT);


        foundationClampLeft = hardwareMap.get(Servo.class, "foundationClampLeft");
        foundationClampRight = hardwareMap.get(Servo.class, "foundationClampRight");




    }

    @Override
    public void start() {

       linear_slide.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        linear_slide.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);

        flipBackLeft.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        flipBackRight.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);

        flipBackLeft.setTargetPosition(0);
        flipBackRight.setTargetPosition(0);
        flipBackLeft.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        flipBackRight.setMode(DcMotor.RunMode.RUN_TO_POSITION);

        flipBackLeft.setPower(1);
        flipBackRight.setPower(1);

        odometryServo.setPosition(1);
        front_claw.setPosition(.71428);

    }

    @Override
    public void loop() {







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

            while (!recalibrate && gamepad2.back) {
                flipBackRight.setTargetPosition(200);
                flipBackLeft.setTargetPosition(200);
                flipBackLeft.setMode(DcMotor.RunMode.RUN_TO_POSITION);
                flipBackRight.setMode(DcMotor.RunMode.RUN_TO_POSITION);
                flipBackRight.setPower(.5);
                flipBackLeft.setPower(.5);

                if (!flipBackLeft.isBusy()) {
                    flipBackLeft.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
                    flipBackRight.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
                    recalibrate = true;
                }
            }

            flipBackLeft.setPower(0);
            flipBackRight.setPower(0);
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



            if (gamepad1.dpad_left) {
                driveSideways = -1;
            } else if (gamepad1.dpad_right) {
                driveSideways = 1;
            }
            double lfpower = driveforward / speedDivider + turn / turnDivider + driveSideways / speedDivider ;
            double lbpower = driveforward / speedDivider + turn / turnDivider - driveSideways / speedDivider ;
            double rfpower = driveforward / speedDivider - turn / turnDivider - driveSideways / speedDivider ;
            double rbpower = driveforward / speedDivider - turn / turnDivider + driveSideways / speedDivider ;

            driveEach(lfpower, lbpower, rfpower, rbpower);
        }


        if (gamepad1.dpad_up || gamepad1.dpad_down) {



            double direction = 1;

            if (gamepad1.dpad_down) {
                direction = -1;
            }

            driveEach((1 * direction) / speedDivider , (1 * direction) / speedDivider , (1 * direction) / speedDivider , (1 * direction) / speedDivider );
        }



        //-----------------------------
        //driving
        //-----------------------------
        if (!gamepad1.dpad_left && !gamepad1.dpad_right && !gamepad1.dpad_down && !gamepad1.dpad_up) {
            double lfpower = driveforward / speedDivider + turn / turnDivider + driveSideways / speedDivider;
            double lbpower = driveforward / speedDivider + turn / turnDivider - driveSideways / speedDivider;
            double rfpower = driveforward / speedDivider - turn / turnDivider - driveSideways / speedDivider;
            double rbpower = driveforward / speedDivider - turn / turnDivider + driveSideways / speedDivider;

            driveEach(lfpower, lbpower, rfpower, rbpower);
        }


        //-----------------------------
        //flip back
        //-----------------------------
        if (!(linear_slide.getCurrentPosition() > 5)) {
            if (gamepad2.dpad_right && timeForFlip.seconds() > .5) {

                if (flippedBack) {
                    flipBackLeft.setTargetPosition(0);
                    flipBackRight.setTargetPosition(0);
                    flippedBack = false;
                } else if (!flippedBack) {
                    flipBackLeft.setTargetPosition(-300);
                    flipBackRight.setTargetPosition(-300);
                    flippedBack = true;
                }

                timeForFlip.reset();
            }

            flipBackLeft.setMode(DcMotor.RunMode.RUN_TO_POSITION);
            flipBackRight.setMode(DcMotor.RunMode.RUN_TO_POSITION);

            if ((flipBackLeft.isBusy() || flipBackRight.isBusy())) {
                double flipPower = .3;
                flipBackRight.setPower(flipPower);
                flipBackLeft.setPower(flipPower);

            }
        }


        //-----------------------------
        //claw
        //-----------------------------
        //robot.front_claw.setPosition((1 - gamepad1.right_trigger) / 1.4);

        if (gamepad1.right_trigger > 0 && clickTime.milliseconds() > 500) {
            if (clawOpen) {
                front_claw.setPosition(0);
                clawOpen = false;
            } else if (!clawOpen) {
                front_claw.setPosition(.71428);
                clawOpen = true;
            }
            clickTime.reset();
        }


        //drop capstone
        if (timeBetweenCapDrop.seconds() > .5) {
            if (gamepad2.y) {
                if (capdropperDown) {
                    capper.setPosition(.87);
                    capdropperDown = false;
                } else if (!capdropperDown) {
                    capper.setPosition(.5);
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

            linear_slide.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);

            if (linear_slide.getCurrentPosition() < 3888) {
                linear_slide.setPower(gamepad2.right_trigger);

            } else {
                linear_slide.setPower(0);
            }

        } else if (gamepad2.left_trigger > 0 && !resetSlide) {

            linear_slide.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);

            if (linear_slide.getCurrentPosition() > 30) {
                linear_slide.setPower(-gamepad2.left_trigger);
            } else {
                linear_slide.setPower(0);
            }

        } else {
            if (resetSlide == false) {
                linear_slide.setPower(0);
            }
        }

        //reset linear slide
        if (gamepad2.x || resetSlide) {
            resetSlide = true;
            linear_slide.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
            linear_slide.setTargetPosition(0);
            linear_slide.setMode(DcMotor.RunMode.RUN_TO_POSITION);

            double distance = Math.abs(0 - linear_slide.getCurrentPosition());
            double speed = distance;
            if (distance < 50) {
                speed /= 100;
            } else {
                speed /= 20;
            }


            linear_slide.setPower(speed);


            if (distance < 5 || !linear_slide.isBusy() || gamepad2.b) {
                linear_slide.setPower(0);
                resetSlide = false;
                linear_slide.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
            }
        }

        //fix flip back jiggle
        if (gamepad2.dpad_left) {
            if (flipBackLeft.getCurrentPosition() < 10) {
                flipBackRight.setPower(0);
                flipBackLeft.setPower(0);
                flipBackLeft.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
                flipBackRight.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
            }
        }


        if (gamepad2.a) {
            speedDivider = 3;
            turnDivider = 1.5;
        }


    }


    public void driveEach(double lf, double lb, double rf, double rb) {

        left_front.setPower(lf);
        left_back.setPower(lb);
        right_front.setPower(rf);
        right_back.setPower(rb);
    }

    public void driveAll(double power) {
        left_front.setPower(power);
        left_back.setPower(power);
        right_front.setPower(power);
        right_back.setPower(power);
    }

}


