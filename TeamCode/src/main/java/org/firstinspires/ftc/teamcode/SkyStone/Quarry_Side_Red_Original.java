//Copyright (c) 2019 FIRST. All rights reserved.

package org.firstinspires.ftc.teamcode.SkyStone;

import com.disnodeteam.dogecv.detectors.skystone.SkystoneDetector;
import com.qualcomm.hardware.modernrobotics.ModernRoboticsI2cGyro;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.DistanceSensor;
import com.qualcomm.robotcore.hardware.IntegratingGyroscope;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.openftc.easyopencv.OpenCvCamera;
import org.openftc.easyopencv.OpenCvCameraRotation;
import org.openftc.easyopencv.OpenCvInternalCamera;


@Autonomous(name = "Quarry Side (Red)")
public class Quarry_Side_Red_Original extends LinearOpMode {



    Servo odometryServo;
    Servo capstone;

    Servo frontClaw;
    DcMotor flipBackLeft;
    DcMotor flipBackRight;

    DcMotor right_front, right_back, left_front, left_back;    //Drive Motors
    DcMotor verticalLeft, verticalRight, horizontal;           //Odometry Wheels

    final double COUNTS_PER_INCH = 307.699557;

    String rfName = "right_front", rbName = "right_back", lfName = "left_front", lbName = "left_back";
    String verticalLeftEncoderName = lfName, verticalRightEncoderName = rfName, horizontalEncoderName = rbName;

    //OdometryGlobalCoordinatePosition globalPositionUpdate;


    private OpenCvCamera phoneCam;
    private SkystoneDetector skyStoneDetector;

    private String configuration = "Not Decided";
    private boolean stonesLeft = true;

    public IntegratingGyroscope gyro;
    public ModernRoboticsI2cGyro modernRoboticsI2cGyro;

    ElapsedTime timeout = new ElapsedTime();



    @Override
    public void runOpMode() {

        initDriveHardwareMap(rfName, rbName, lfName, lbName, verticalLeftEncoderName, verticalRightEncoderName, horizontalEncoderName);

        modernRoboticsI2cGyro = hardwareMap.get(ModernRoboticsI2cGyro.class, "gyro");
        gyro = (IntegratingGyroscope) modernRoboticsI2cGyro;

        modernRoboticsI2cGyro.calibrate();
        while (!isStopRequested() && modernRoboticsI2cGyro.isCalibrating()) {
            telemetry.addLine("Gyro: Calibrating");
            telemetry.update();
        }
        telemetry.addLine("Gyro: Done calibrating");
        telemetry.update();

        frontClaw = hardwareMap.get(Servo.class, "frontclaw");

        flipBackLeft = hardwareMap.get(DcMotor.class, "left_flip");
        flipBackRight = hardwareMap.get(DcMotor.class, "right_flip");

        flipBackRight.setDirection(DcMotorSimple.Direction.REVERSE);
        flipBackLeft.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        flipBackRight.setMode(DcMotor.RunMode.RUN_USING_ENCODER);

        odometryServo = hardwareMap.get(Servo.class, "odometryServo");
        capstone = hardwareMap.get(Servo.class, "capstone");


        telemetry.addData("Status", "Init Complete");
        telemetry.update();


        waitForStart();

        odometryServo.setPosition(.5);






        //make sure the claw is up
        frontClaw.setPosition(.7);


        /*
        Stone 1
         */
        //Move close to the stone line
        moveDistanceEncoder(23, .5);


        capstone.setPosition(.87);

        //flip the linear slide down
        flipBackLeft.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        flipBackRight.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        flipBackLeft.setTargetPosition(350);
        flipBackRight.setTargetPosition(350);
        flipBackRight.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        flipBackLeft.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        while (opModeIsActive() && flipBackLeft.isBusy()) {
            flipBackLeft.setPower(.4);
            flipBackRight.setPower(.4);
        }
        flipBackLeft.setPower(0);
        flipBackRight.setPower(0);

        //move to be aligned with the left most stone. (no scanning for skystone yet)
        strafeEncoder(6, .5);

        //Move a little more forward to get close enough to the stone to grab it
        moveDistanceEncoder(1.5, .25);

        //Close the claw and wait for it to reach the closed position
        frontClaw.setPosition(0);
        sleep(600);

        //Flip the linear slide back into the robot
        flipBackLeft.setTargetPosition(0);
        flipBackRight.setTargetPosition(0);
        flipBackRight.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        flipBackLeft.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        while (opModeIsActive() && flipBackLeft.isBusy()) {
            flipBackLeft.setPower(.2);
            flipBackRight.setPower(.2);
        }
        flipBackLeft.setPower(0);
        flipBackRight.setPower(0);




        moveDistanceEncoder(-2, .85);//back up from stones
        turnDegree(-90, .5, 5);  //turn left 90 to drive
        strafeEncoder(-1, .5);  //strafe back to the stones
        moveDistanceEncoder(60, .85); //move to the other side
        turnDegree(90, .5, 5); //turn 90 right to drop on platform
        moveDistanceEncoder(6, .5); // move forward a little


        //flip the linear slide down
        flipBackLeft.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        flipBackRight.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        flipBackLeft.setTargetPosition(250);
        flipBackRight.setTargetPosition(250);
        flipBackRight.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        flipBackLeft.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        while (opModeIsActive() && flipBackLeft.isBusy()) {
            flipBackLeft.setPower(.6);
            flipBackRight.setPower(.6);
        }
        flipBackLeft.setPower(0);
        flipBackRight.setPower(0);

        //open claw
        frontClaw.setPosition(1);
        sleep(300);
        //Flip the linear slide back into the robot
        flipBackLeft.setTargetPosition(0);
        flipBackRight.setTargetPosition(0);
        flipBackRight.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        flipBackLeft.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        while (opModeIsActive() && flipBackLeft.isBusy()) {
            flipBackLeft.setPower(.2);
            flipBackRight.setPower(.2);
        }
        flipBackLeft.setPower(0);
        flipBackRight.setPower(0);












        /*
        Go for stone 2
         */
        moveDistanceEncoder(-3, .5); // back up
        turnDegree(89, .7, 5); //turn left 90 to go back
        moveDistanceEncoder(68, .85); // move to the stones side
        //strafeEncoder(-2,.5);  // to the side for the turn
        turnDegree(-89, .5, 5);  //turn to face stones


        //flip the linear slide down
        flipBackLeft.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        flipBackRight.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        flipBackLeft.setTargetPosition(350);
        flipBackRight.setTargetPosition(350);
        flipBackRight.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        flipBackLeft.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        while (opModeIsActive() && flipBackLeft.isBusy()) {
            flipBackLeft.setPower(.4);
            flipBackRight.setPower(.4);
        }
        flipBackLeft.setPower(0);
        flipBackRight.setPower(0);


        moveDistanceEncoder(6, .5); //move close to stone


        //Close the claw and wait for it to reach the closed position
        frontClaw.setPosition(0);
        sleep(600);
        //Flip the linear slide back into the robot
        flipBackLeft.setTargetPosition(0);
        flipBackRight.setTargetPosition(0);
        flipBackRight.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        flipBackLeft.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        while (opModeIsActive() && flipBackLeft.isBusy()) {
            flipBackLeft.setPower(.2);
            flipBackRight.setPower(.2);
        }
        flipBackLeft.setPower(0);
        flipBackRight.setPower(0);


        moveDistanceEncoder(-2, .85);//back up from stones
        turnDegree(-84, .5, 5);  //turn left 90 to drive
        strafeEncoder(-.5, .5);  //strafe back to the stones
        moveDistanceEncoder(40, .85); //move to the other side
        //turnDegree(-88,.85,5); //turn 90 right to drop
        //moveDistanceEncoder(4,.7); // move forward a little


        //   strafeEncoder(-3,.5);

        //flip the linear slide down
        flipBackLeft.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        flipBackRight.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        flipBackLeft.setTargetPosition(300);
        flipBackRight.setTargetPosition(300);
        flipBackRight.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        flipBackLeft.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        while (opModeIsActive() && flipBackLeft.isBusy()) {
            flipBackLeft.setPower(.4);
            flipBackRight.setPower(.4);
        }
        flipBackLeft.setPower(0);
        flipBackRight.setPower(0);


        //open claw
        frontClaw.setPosition(1);
        sleep(300);



        //park
        moveDistanceEncoder(-7, .5); // back up








    }







    /*-------------------------------------------------
     *
     * Methods
     *
     * --------------------------------------------------*/



    public void moveDistanceEncoder(double inches, double speed) {

        verticalLeft.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        verticalRight.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        horizontal.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);

        double ticks = inches * COUNTS_PER_INCH;

        double targetLeft = verticalLeft.getCurrentPosition() + ticks;
        double targetRight = verticalRight.getCurrentPosition() + ticks;

        double leftPower = speed;
        double rightPower = speed;
        double adjust = 0;

        if (inches < 0) {
            leftPower *= -1;
            rightPower *= -1;
        }

        right_front.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        right_back.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        left_front.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        left_back.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);

        timeout.reset();
        modernRoboticsI2cGyro.resetZAxisIntegrator();
        double integratedZ = modernRoboticsI2cGyro.getIntegratedZValue();

        //timeout is new
        while (opModeIsActive()  && ((Math.abs(verticalRight.getCurrentPosition() - targetRight) >= 100) &&
                (Math.abs(verticalLeft.getCurrentPosition() - targetLeft) >= 100))) {


            while (opModeIsActive() && timeout.seconds() > 6){
                driveAll(0);
                telemetry.addLine("Robot: stuck");
                telemetry.update();
            }

            integratedZ = modernRoboticsI2cGyro.getIntegratedZValue();

            if (integratedZ > 0) {
                adjust = .04;
            } else if (integratedZ < 0) {
                adjust = -.04;
            }


            if (inches > 0 && ((verticalRight.getCurrentPosition() >= (2 * (ticks / 3))) ||
                    (verticalLeft.getCurrentPosition() >= (2 * (ticks / 3))))) {
                leftPower = .25;
                rightPower = .25;
            } else if (inches < 0 && ((verticalRight.getCurrentPosition() <= (2 * (ticks / 3))) ||
                    (verticalLeft.getCurrentPosition() <= (2 * (ticks / 3))))) {
                leftPower = -.25;
                rightPower = -.25;
            }

            // driveEach(leftPower, leftPower, rightPower, rightPower);
            left_front.setPower(leftPower + adjust);
            right_front.setPower(rightPower - adjust);
            left_back.setPower(leftPower + adjust);
            right_back.setPower(rightPower - adjust);


            if (inches > 0 && ((verticalLeft.getCurrentPosition() > targetLeft) || (verticalRight.getCurrentPosition() > targetRight))) {
                break;
            } else if (inches < 0 && ((verticalLeft.getCurrentPosition() < targetLeft) || (verticalRight.getCurrentPosition() < targetRight))) {
                break;
            }
        }
        driveAll(0);


    }
    public void strafeEncoder(double inches, double speed) {
        verticalLeft.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        verticalRight.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        horizontal.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);

        double ticks = inches * COUNTS_PER_INCH;

        double targetHorizontal = horizontal.getCurrentPosition() + ticks;

        double power = speed;

        if (inches < 0) {
            power *= -1;
        }
        double adjust = 0;

        right_front.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        right_back.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        left_front.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        left_back.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);

        modernRoboticsI2cGyro.resetZAxisIntegrator();
        double integratedZ = modernRoboticsI2cGyro.getIntegratedZValue();
        while (opModeIsActive() && (Math.abs(horizontal.getCurrentPosition() - targetHorizontal) >= 100)) {

            integratedZ = modernRoboticsI2cGyro.getIntegratedZValue();

            if (integratedZ > 0) {
                adjust = .04;
            } else if (integratedZ < 0) {
                adjust = -.04;
            }


            if (inches > 0 && horizontal.getCurrentPosition() >= (2 * (ticks / 3))) {
                power = .25;
            } else if (inches < 0 && horizontal.getCurrentPosition() <= (2 * (ticks / 3))) {
                power = -.25;
            }else if(inches > 0 && horizontal.getCurrentPosition() > ticks){
                break;
            }else if(inches < 0 && horizontal.getCurrentPosition() < ticks){
                break;
            }

            // driveEach(leftPower, leftPower, rightPower, rightPower);
            left_front.setPower(power + adjust);
            right_front.setPower(-power - adjust);
            left_back.setPower(-power + adjust);
            right_back.setPower(power - adjust);
        }
        driveAll(0);


    }
    public void turnDegree(double degrees, double speed, double allowedError) {
        modernRoboticsI2cGyro.resetZAxisIntegrator();
        //double heading = modernRoboticsI2cGyro.getHeading();
        double integratedZ = modernRoboticsI2cGyro.getIntegratedZValue();
        //right is positive

        //modernRoboticsI2cGyro.resetZAxisIntegrator();

        double leftSpeed = speed;
        double rightSpeed = -speed;

        if (degrees > 0) {
            leftSpeed *= -1;
            rightSpeed *= -1;
        }

        right_front.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        right_back.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        left_front.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        left_back.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);

        while (opModeIsActive() && Math.abs(integratedZ - degrees) >= allowedError) {
            integratedZ = modernRoboticsI2cGyro.getIntegratedZValue();
            telemetry.addData("angle", integratedZ);
            telemetry.addData("speed", leftSpeed);
            telemetry.addData("degrees", degrees);
            telemetry.update();


            if (Math.abs(integratedZ) > Math.abs((degrees * 0.7))) {
                leftSpeed = -.2 * Math.signum(degrees);
                rightSpeed = .2 * Math.signum(degrees);
            } else if (degrees > 0 && integratedZ > degrees) {
                break;
            } else if (degrees < 0 && integratedZ < degrees) {
                break;
            } else {
                leftSpeed = speed * -1 * Math.signum(degrees);
                rightSpeed = speed * Math.signum(degrees);
            }
            driveEach(leftSpeed, leftSpeed, rightSpeed, rightSpeed);

        }
        driveAll(0);

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






    /*
    public void goToPosition(double targetXPos, double targetYPos, double power, double desiredRobotOrientation, double allowedDistanceError) {
        targetXPos *= COUNTS_PER_INCH;
        targetYPos *= COUNTS_PER_INCH;
        allowedDistanceError *= COUNTS_PER_INCH;

        double distanceToXTarget = targetXPos - globalPositionUpdate.returnXCoordinate();
        double distanceToYTarget = targetYPos - globalPositionUpdate.returnYCoordinate();

        double distance = Math.hypot(distanceToXTarget, distanceToYTarget);
        while (opModeIsActive() && distance > allowedDistanceError) {


            distanceToXTarget = targetXPos - globalPositionUpdate.returnXCoordinate();
            distanceToYTarget = targetYPos - globalPositionUpdate.returnYCoordinate();

            distance = Math.hypot(distanceToXTarget, distanceToYTarget);


            double robotMovementAngle = Math.toDegrees(Math.atan2(distanceToXTarget, distanceToYTarget));

            double robotMovementXComponent = calculateX(robotMovementAngle, power);
            double robotMovementYComponent = calculateY(robotMovementAngle, power);
            double pivotCorrection = desiredRobotOrientation - globalPositionUpdate.returnOrientation();

            //may need to flip the signs for the pivotCorrection
            left_front.setPower(Range.clip((robotMovementXComponent + robotMovementYComponent + pivotCorrection), -1, 1));
            left_back.setPower(Range.clip((-robotMovementXComponent + robotMovementYComponent + pivotCorrection), -1, 1));
            right_front.setPower(Range.clip((-robotMovementXComponent + robotMovementYComponent - pivotCorrection), -1, 1));
            right_back.setPower(Range.clip((robotMovementXComponent + robotMovementYComponent - pivotCorrection), -1, 1));
        }
    }


     */




    private void initDriveHardwareMap(String rfName, String rbName, String lfName, String lbName, String vlEncoderName, String vrEncoderName, String hEncoderName) {
        right_front = hardwareMap.dcMotor.get(rfName);
        right_back = hardwareMap.dcMotor.get(rbName);
        left_front = hardwareMap.dcMotor.get(lfName);
        left_back = hardwareMap.dcMotor.get(lbName);

        verticalLeft = hardwareMap.dcMotor.get(vlEncoderName);
        verticalRight = hardwareMap.dcMotor.get(vrEncoderName);
        horizontal = hardwareMap.dcMotor.get(hEncoderName);

        right_front.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        right_back.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        left_front.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        left_back.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);

        right_front.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        right_back.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        left_front.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        left_back.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);

        verticalLeft.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        verticalRight.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        horizontal.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);

        verticalLeft.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        verticalRight.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        horizontal.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);


        right_front.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        right_back.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        left_front.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        left_back.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);

        left_back.setDirection(DcMotorSimple.Direction.REVERSE);
        right_front.setDirection(DcMotorSimple.Direction.REVERSE);
        right_back.setDirection(DcMotorSimple.Direction.REVERSE);

        telemetry.addData("Status", "Hardware Map Init Complete");
        telemetry.update();
    }

    /**
     * Calculate the power in the x direction
     *
     * @param desiredAngle angle on the x axis
     * @param speed        robot's speed
     * @return the x vector
     */
    private double calculateX(double desiredAngle, double speed) {
        return Math.sin(Math.toRadians(desiredAngle)) * speed;
    }

    /**
     * Calculate the power in the y direction
     *
     * @param desiredAngle angle on the y axis
     * @param speed        robot's speed
     * @return the y vector
     */
    private double calculateY(double desiredAngle, double speed) {
        return Math.cos(Math.toRadians(desiredAngle)) * speed;

    }

}
