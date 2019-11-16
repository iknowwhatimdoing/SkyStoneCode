package org.firstinspires.ftc.teamcode.SkyStone.Teleop;

import com.qualcomm.hardware.bosch.BNO055IMU;
import com.qualcomm.hardware.bosch.JustLoggingAccelerationIntegrator;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.util.ElapsedTime;
import com.qualcomm.robotcore.util.Range;

import org.firstinspires.ftc.robotcore.external.navigation.Acceleration;
import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.robotcore.external.navigation.AxesOrder;
import org.firstinspires.ftc.robotcore.external.navigation.AxesReference;
import org.firstinspires.ftc.robotcore.external.navigation.Orientation;
import org.firstinspires.ftc.robotcore.external.navigation.Position;
import org.firstinspires.ftc.robotcore.external.navigation.Velocity;


@TeleOp(name = "Competition 2019")

public class CompetitionTeleOp extends OpMode {

    double assist = 0;
    double current = 0;
    boolean getCurrent = false;

    boolean accurateSpeed = false;


    boolean holdHeading = false;


    BNO055IMU imu;

    Orientation angles;
    Acceleration gravity;


    ElapsedTime slowDown = new ElapsedTime();
    ElapsedTime timeBetweenPress = new ElapsedTime();

    boolean slideup = false;
    double power = 0;


    DcMotor left_front;
    DcMotor right_front;
    DcMotor left_back;
    DcMotor right_back;

    DcMotor linear_slide;

    DcMotor flipBackRight;
    DcMotor flipBackLeft;

    Servo left_claw;
    Servo right_claw;


    @Override
    public void init() {


        BNO055IMU.Parameters parameters = new BNO055IMU.Parameters();
        parameters.angleUnit = BNO055IMU.AngleUnit.DEGREES;
        parameters.accelUnit = BNO055IMU.AccelUnit.METERS_PERSEC_PERSEC;
        parameters.calibrationDataFile = "BNO055IMUCalibration.json"; // see the calibration sample opmode
        parameters.loggingEnabled = true;
        parameters.loggingTag = "IMU";
        parameters.accelerationIntegrationAlgorithm = new JustLoggingAccelerationIntegrator();


        imu = hardwareMap.get(BNO055IMU.class, "imu");
        imu.initialize(parameters);


        left_front = hardwareMap.get(DcMotor.class, "left_front");
        right_front = hardwareMap.get(DcMotor.class, "right_front");
        left_back = hardwareMap.get(DcMotor.class, "left_back");
        right_back = hardwareMap.get(DcMotor.class, "right_back");


        flipBackLeft = hardwareMap.get(DcMotor.class, "left_flip");
        flipBackRight = hardwareMap.get(DcMotor.class, "right_flip");

        flipBackRight.setDirection(DcMotorSimple.Direction.REVERSE);

        flipBackLeft.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        flipBackRight.setMode(DcMotor.RunMode.RUN_USING_ENCODER);

        linear_slide = hardwareMap.get(DcMotor.class, "linear_slide");

        linear_slide.setDirection(DcMotorSimple.Direction.REVERSE);
        linear_slide.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        linear_slide.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);


        left_back.setDirection(DcMotor.Direction.REVERSE);
        right_front.setDirection(DcMotor.Direction.REVERSE);
        right_back.setDirection(DcMotor.Direction.REVERSE);


        left_claw = hardwareMap.get(Servo.class, "lc");
        right_claw = hardwareMap.get(Servo.class, "rc");

    }

    @Override
    public void start() {
        imu.startAccelerationIntegration(new Position(), new Velocity(), 1000);
    }

    @Override
    public void loop() {

        angles = imu.getAngularOrientation(AxesReference.INTRINSIC, AxesOrder.ZYX, AngleUnit.DEGREES);

        double driveforward = -gamepad1.left_stick_y;
        double driveSideways = gamepad1.left_stick_x;
        double turn = gamepad1.right_stick_x;


        if (gamepad1.dpad_left) {
            telemetry.addLine("Holding rotation");
            holdHeading = true;
            current = angles.firstAngle;

        } else if (gamepad1.dpad_right) {
            holdHeading = false;
            telemetry.addLine("Press left dpad to hold rotation");
        }

        telemetry.update();


        if(timeBetweenPress.seconds() > .5) {
            if (gamepad1.left_stick_button) {
                if (!accurateSpeed) {
                    accurateSpeed = true;
                } else if (accurateSpeed) {
                    accurateSpeed = false;
                }
                timeBetweenPress.reset();
            }
        }





        if (gamepad1.left_trigger > 0) {

            if (!getCurrent) {
                current = angles.firstAngle;
                getCurrent = true;
            }

            angles = imu.getAngularOrientation(AxesReference.INTRINSIC, AxesOrder.ZYX, AngleUnit.DEGREES);
            double difference = current - angles.firstAngle;

            assist = difference / 40;
            driveSideways = -gamepad1.left_trigger;
            double lfpower = driveforward + turn + driveSideways - assist;
            double lbpower = driveforward + turn - driveSideways - assist;
            double rfpower = driveforward - turn - driveSideways + assist;
            double rbpower = driveforward - turn + driveSideways + assist;


            setPowerAll(lfpower, lbpower, rfpower, rbpower);
        } else if (gamepad1.right_trigger > 0) {

            if (!getCurrent) {
                current = angles.firstAngle;
                getCurrent = true;
            }


            angles = imu.getAngularOrientation(AxesReference.INTRINSIC, AxesOrder.ZYX, AngleUnit.DEGREES);
            double difference = current - angles.firstAngle;

            assist = difference / 40;
            driveSideways = gamepad1.right_trigger;
            double lfpower = driveforward + turn + driveSideways - assist;
            double lbpower = driveforward + turn - driveSideways - assist;
            double rfpower = driveforward - turn - driveSideways + assist;
            double rbpower = driveforward - turn + driveSideways + assist;

            setPowerAll(lfpower, lbpower, rfpower, rbpower);
        }

        if (gamepad1.right_trigger == 0 && gamepad1.left_trigger == 0) {
            getCurrent = false;
        }

        if (gamepad1.right_trigger == 0 && gamepad1.left_trigger == 0) {

            if (!holdHeading) {

                double turnDivider = 1;
                double speedDivider = 1;
                if(accurateSpeed){
                    turnDivider = 4;
                    speedDivider = 4;
                }
                double lfpower = driveforward/speedDivider + turn/turnDivider + driveSideways/speedDivider;
                double lbpower = driveforward/speedDivider + turn/turnDivider - driveSideways/speedDivider;
                double rfpower = driveforward/speedDivider - turn/turnDivider - driveSideways/speedDivider;
                double rbpower = driveforward/speedDivider - turn/turnDivider + driveSideways/speedDivider;

                setPowerAll(lfpower, lbpower, rfpower, rbpower);

            } else if (holdHeading) {
                angles = imu.getAngularOrientation(AxesReference.INTRINSIC, AxesOrder.ZYX, AngleUnit.DEGREES);
                double difference = current - angles.firstAngle;

                assist = difference / 40;
                double lfpower = driveforward + driveSideways - assist;
                double lbpower = driveforward - driveSideways - assist;
                double rfpower = driveforward - driveSideways + assist;
                double rbpower = driveforward + driveSideways + assist;

                setPowerAll(lfpower, lbpower, rfpower, rbpower);

            }

        }


        if (!slideup) {
            if (gamepad2.dpad_down) {
                flipBackLeft.setTargetPosition(-350);
                flipBackRight.setTargetPosition(-350);
                flipBackLeft.setMode(DcMotor.RunMode.RUN_TO_POSITION);
                flipBackRight.setMode(DcMotor.RunMode.RUN_TO_POSITION);

            } else if (gamepad2.dpad_up) {
                flipBackLeft.setTargetPosition(0);
                flipBackRight.setTargetPosition(0);
                flipBackLeft.setMode(DcMotor.RunMode.RUN_TO_POSITION);
                flipBackRight.setMode(DcMotor.RunMode.RUN_TO_POSITION);

            }

            if ((flipBackLeft.isBusy() || flipBackRight.isBusy()) && Math.abs(flipBackLeft.getCurrentPosition() - flipBackLeft.getTargetPosition()) >= 5) {

                double flipPower = 0;
                if (Math.abs(flipBackLeft.getCurrentPosition() - flipBackLeft.getTargetPosition()) >= Math.abs(flipBackLeft.getTargetPosition() / 2)) {
                    flipPower = .2;
                } else if (Math.abs(flipBackLeft.getCurrentPosition() - flipBackLeft.getTargetPosition()) < Math.abs(flipBackLeft.getTargetPosition() / 3.5)) {
                    flipPower = .1;
                }

                flipBackRight.setPower(flipPower);
                flipBackLeft.setPower(flipPower);

            }
        }


        left_claw.setPosition(Range.clip((gamepad2.right_trigger + .25), 0, .41));
        right_claw.setPosition(Range.clip((1 - gamepad2.right_trigger), .841, 1));


        if (gamepad1.y && !slideup && (Math.abs(-flipBackLeft.getCurrentPosition() - 300) > 200)) {

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

    public void setPowerAll(double lf, double lb, double rf, double rb) {
        left_front.setPower(lf);
        left_back.setPower(lb);
        right_back.setPower(rb);
        right_front.setPower(rf);


    }
}






