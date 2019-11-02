package org.firstinspires.ftc.teamcode.SkyStone.Teleop;

import com.qualcomm.hardware.bosch.BNO055IMU;
import com.qualcomm.hardware.bosch.JustLoggingAccelerationIntegrator;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
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

    double assist=0;
    boolean doOnce = false;
    double current = 0;

    boolean holdHeading = false;


    BNO055IMU imu;

    Orientation angles;
    Acceleration gravity;


    ElapsedTime slowDown = new ElapsedTime();
    boolean slideup = false;
    double power = 0;


    DcMotor left_front;
    DcMotor right_front;
    DcMotor left_back;
    DcMotor right_back;

    DcMotor linear_slide;

    DcMotor flipBackRight;
    DcMotor flipBackLeft;


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
    public void start(){
        imu.startAccelerationIntegration(new Position(), new Velocity(), 1000);
    }
    @Override
    public void loop() {

        angles = imu.getAngularOrientation(AxesReference.INTRINSIC, AxesOrder.ZYX, AngleUnit.DEGREES);

        double driveforward = -gamepad1.left_stick_y;
        double driveSideways = gamepad1.left_stick_x;
        double turn = gamepad1.right_stick_x;

        if(gamepad1.a){
            current = angles.firstAngle;
        }

        if(gamepad1.left_bumper){
            holdHeading = true;

        }else if(gamepad1.right_bumper){
            holdHeading = false;
        }

        if (gamepad1.left_trigger > 0) {

            angles = imu.getAngularOrientation(AxesReference.INTRINSIC, AxesOrder.ZYX, AngleUnit.DEGREES);
            double difference = current - angles.firstAngle;

            assist = difference / 40;
            driveSideways = -gamepad1.left_trigger;
            double lfpower = driveforward + turn + driveSideways - assist;
            double lbpower = driveforward + turn - driveSideways - assist;
            double rfpower = driveforward - turn - driveSideways + assist;
            double rbpower = driveforward - turn + driveSideways + assist;


            setPowerAll(lfpower, lbpower, rfpower, rbpower);
        }else if(gamepad1.right_trigger > 0) {


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
        if(gamepad1.right_trigger == 0 && gamepad1.left_trigger == 0){

            if(!holdHeading) {
                double lfpower = driveforward + turn + driveSideways;
                double lbpower = driveforward + turn - driveSideways;
                double rfpower = driveforward - turn - driveSideways;
                double rbpower = driveforward - turn + driveSideways;

                setPowerAll(lfpower, lbpower, rfpower, rbpower);

            }else if(holdHeading){
                angles = imu.getAngularOrientation(AxesReference.INTRINSIC, AxesOrder.ZYX, AngleUnit.DEGREES);
                double difference = current - angles.firstAngle;

                assist = difference / 40;
                double lfpower = driveforward  + driveSideways - assist;
                double lbpower = driveforward  - driveSideways - assist;
                double rfpower = driveforward  - driveSideways + assist;
                double rbpower = driveforward  + driveSideways + assist;

                setPowerAll(lfpower, lbpower, rfpower, rbpower);

            }

        }




        if (!slideup) {
            if (gamepad1.dpad_down) {
                flipBackLeft.setPower(-.4);
                flipBackRight.setPower(-.4);
            } else if (gamepad1.dpad_up) {
                flipBackLeft.setPower(.4);
                flipBackRight.setPower(.4);
            } else {
                flipBackLeft.setPower(0);
                flipBackRight.setPower(0);
            }
        }


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

    public void setPowerAll(double lf, double lb, double rf, double rb) {
        left_front.setPower(lf);
        left_back.setPower(lb);
        right_back.setPower(rb);
        right_front.setPower(rf);


    }
}






