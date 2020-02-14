//Copyright (c) 2019 FIRST. All rights reserved.

package org.firstinspires.ftc.teamcode.SkyStone.Tests;

import com.qualcomm.hardware.modernrobotics.ModernRoboticsI2cGyro;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.DigitalChannel;
import com.qualcomm.robotcore.hardware.IntegratingGyroscope;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.util.ElapsedTime;
import com.qualcomm.robotcore.util.Range;

import org.firstinspires.ftc.teamcode.SkyStone.Odometry.OdometryGlobalCoordinatePosition;
import org.openftc.easyopencv.OpenCvCamera;
import org.openftc.easyopencv.OpenCvCameraRotation;
import org.openftc.easyopencv.OpenCvInternalCamera;


@Autonomous(name = "Test foundation movement")
@Disabled
public class foundationTester extends LinearOpMode {

    //movement offsets for the pattern
    public double patternOffsetX = 0;
    public double patternOffsetY = 0;

    //constants
    private static final double SIDE_CLAW_OPEN = .55;
    private static final double SIDE_CLAW_CLOSED = .25;
    final double COUNTS_PER_INCH = 307.699557;

    //OpenCV initialization
    private OpenCvCamera phoneCam;
    private OpenCV openCV;

    //timeout to stop the robot from driving while stuck
    ElapsedTime timeout = new ElapsedTime();

    //distance sensors if needed
    //public DistanceSensor rightSideDist;
    //public DistanceSensor leftSideDist;
    //public DistanceSensor frontLeftDist;
    //public DistanceSensor frontRightDist;

    //servo that can pull up/ down the odometry wheel (horizontal)
    Servo odometryServo;

    //side arm, foundation clamps, and side arm clamp
    DcMotor sideArm;
    Servo clampL, clampR, sideGrabber;


    //front claw, flip back motors, linear slide, and drive
    Servo frontClaw;
    DcMotor flipBackLeft, flipBackRight;
    DcMotor linear_slide;
    DcMotor right_front, right_back, left_front, left_back;    //Drive Motors
    DcMotor verticalLeft, verticalRight, horizontal;           //Odometry Wheels


    //hardware map names for wheels
    String rfName = "right_front", rbName = "right_back", lfName = "left_front", lbName = "left_back";
    String verticalLeftEncoderName = lfName, verticalRightEncoderName = rfName, horizontalEncoderName = rbName;

    //starting odometry
    OdometryGlobalCoordinatePosition globalPositionUpdate;


    //front touch sensor
    DigitalChannel frontTouch;


    //gyro
    public IntegratingGyroscope gyro;
    public ModernRoboticsI2cGyro modernRoboticsI2cGyro;


    @Override
    public void runOpMode() {
        initDriveHardwareMap(rfName, rbName, lfName, lbName, verticalLeftEncoderName, verticalRightEncoderName, horizontalEncoderName);
        initOpenCV();


        flipBackLeft.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        flipBackRight.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);

        sleep(1000);
        telemetry.addData("Status", "Init Complete");
        telemetry.update();








        waitForStart();

        //lower the middle odometry wheel
        odometryServo.setPosition(.5);



        //Create and start GlobalCoordinatePosition thread to constantly update the global coordinate positions
        globalPositionUpdate = new OdometryGlobalCoordinatePosition(verticalLeft, verticalRight, horizontal, COUNTS_PER_INCH, 75);
        Thread positionThread = new Thread(globalPositionUpdate);
        positionThread.start();

        //globalPositionUpdate.reverseRightEncoder();
        globalPositionUpdate.reverseNormalEncoder();


        //make sure the claw is closed and foundation Clamp is up
        frontClaw.setPosition(0);
        clampR.setPosition(1);
        clampL.setPosition(.6);






        //move back to the foundation side
        goToPosition(23.5, 73, .85, 0, 5);


        //lower the arm to place the stone
        sideArmControl("dropping", .2);

        //open the claw
        sideGrabber.setPosition(SIDE_CLAW_OPEN);
        sleep(300);

        //flip the arm back into the robot
        sideArmControl("in", .2);







        /*
        moving the foundation
         */

        //back up from the foundation to turn
        goToPosition(18,80,.6,0,5);

        /*
        goToPosition(18,73,.6,0,5);
        turnDegree(-90,.7,5);
         */
        turnDegree(-90,.7,5);


        strafeEncoder(-2,.5);


        //move the side arm and flip in the main arm
        sideArmControl("hover", .6);
        mainArmControl("in", .6);

        ElapsedTime allowedMoveTime = new ElapsedTime();
        while (opModeIsActive() && frontTouch.getState() && allowedMoveTime.seconds() < 5) {
            moveWithoutEndGoal(.3);
        }
        driveAll(.1);

        //close the clamps
        clampL.setPosition(1);
        clampR.setPosition(.8);
        sleep(300);

        driveAll(0);



        moveDistanceEncoder(-5,.8);

        //turn a bit with the foundation, back up, and finish the turn
        foundationMovement(45, 1,-.4);
        moveDistanceEncoder(-5,.8);
        foundationMovement(45,1,-.4);



        //open the claws
        clampL.setPosition(.6);
        clampR.setPosition(1);
        sleep(300);


        //push the foundation into the zone
        ElapsedTime moveTime = new ElapsedTime();
        while (opModeIsActive() && moveTime.seconds() < .3) {
            driveAll(.5);
        }
        driveAll(0);

        goToPosition(22,70,.8,0,5);


        /*
        //push the foundation into the zone
        ElapsedTime moveTime = new ElapsedTime();
        while (opModeIsActive() && moveTime.seconds() < .3) {
            driveAll(.5);
        }
        driveAll(0);

         */






        //----park----------------


        //back up
        goToPosition(22,60,.85,0,10);

        turnDegree(-90,.7,5);


        //lower the main arm
        mainArmControl("out", .6);

        //bring the side arm into the robot
        sideArmControl("in", .6);

        //drop the capstone
        frontClaw.setPosition(1);
        sleep(300);


        turnDegree(-90,.6,5);
        //move to park
        goToPosition(22,50,.85,180,10);


    }








    /*-------------------------------------------------
     *
     * Methods
     *
     * --------------------------------------------------*/

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
            left_front.setPower(Range.clip((robotMovementXComponent + robotMovementYComponent + (pivotCorrection/20)), -1, 1));
            left_back.setPower(Range.clip((-robotMovementXComponent + robotMovementYComponent + (pivotCorrection/20)), -1, 1));
            right_front.setPower(Range.clip((-robotMovementXComponent + robotMovementYComponent - (pivotCorrection/20)), -1, 1));
            right_back.setPower(Range.clip((robotMovementXComponent + robotMovementYComponent - (pivotCorrection/20)), -1, 1));
        }
        driveAll(0);
    }
    public void moveWithoutEndGoal(double speed) {
        double power = speed;
        double adjust = 0;
        right_front.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        right_back.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        left_front.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        left_back.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);

        double integratedZ = modernRoboticsI2cGyro.getIntegratedZValue();
        if (opModeIsActive()) {
            integratedZ = modernRoboticsI2cGyro.getIntegratedZValue();
            if (integratedZ > 0) {
                adjust = .04;
            } else if (integratedZ < 0) {
                adjust = -.04;
            }
            driveEach(power + adjust, power + adjust, power - adjust, power - adjust);
        }
    }
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
        while (opModeIsActive() && ((Math.abs(verticalRight.getCurrentPosition() - targetRight) >= 100) &&
                (Math.abs(verticalLeft.getCurrentPosition() - targetLeft) >= 100))) {


            while (opModeIsActive() && timeout.seconds() > 8) {
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

        double targetHorizontal = -horizontal.getCurrentPosition() + ticks;

        double power = speed;

        if (inches < 0) {
            power *= -1;
        }
        double adjust = 0;
        double moveAdjust = 0;

        right_front.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        right_back.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        left_front.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        left_back.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);

        modernRoboticsI2cGyro.resetZAxisIntegrator();
        double integratedZ = modernRoboticsI2cGyro.getIntegratedZValue();
        while (opModeIsActive() && (Math.abs(-horizontal.getCurrentPosition() - targetHorizontal) >= 100)) {

            integratedZ = modernRoboticsI2cGyro.getIntegratedZValue();

            if (integratedZ > 0) {
                adjust = .04;
            } else if (integratedZ < 0) {
                adjust = -.04;
            }

            if (verticalLeft.getCurrentPosition() > 0){
                moveAdjust = -.04;
            }else if (verticalLeft.getCurrentPosition() < 0){
                moveAdjust = .04;
            }


            if (inches > 0 && -horizontal.getCurrentPosition() >= (2 * (ticks / 3))) {
                power = .25;
            } else if (inches < 0 && -horizontal.getCurrentPosition() <= (2 * (ticks / 3))) {
                power = -.25;
            } else if (inches > 0 && -horizontal.getCurrentPosition() > ticks) {
                break;
            } else if (inches < 0 && -horizontal.getCurrentPosition() < ticks) {
                break;
            }

            // driveEach(leftPower, leftPower, rightPower, rightPower);
            left_front.setPower(power + adjust + moveAdjust);
            right_front.setPower(-power - adjust+ moveAdjust);
            left_back.setPower(-power + adjust+ moveAdjust);
            right_back.setPower(power - adjust+ moveAdjust);
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
    public void foundationMovement(double rotation, double speed, double assistSpeed) {
        modernRoboticsI2cGyro.resetZAxisIntegrator();
        double integratedZ = modernRoboticsI2cGyro.getIntegratedZValue();
        //right is positive

        //modernRoboticsI2cGyro.resetZAxisIntegrator();

        double leftSpeed = -speed;


        right_front.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        right_back.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        left_front.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        left_back.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);

        while (opModeIsActive() && Math.abs(integratedZ - rotation) >= 2) {
            integratedZ = modernRoboticsI2cGyro.getIntegratedZValue();


            if (integratedZ > rotation / 2) {
                leftSpeed = -.42;
            }
            driveEach(leftSpeed, leftSpeed, assistSpeed, assistSpeed);

            if (integratedZ > rotation) {
                break;
            }

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

    public void sideArmControl(String position, double speed){
        switch (position){
            case "out":
                sideGrabber.setPosition(SIDE_CLAW_OPEN);
                sleep(200);
                //sideArm.setTargetPosition(-148);
                //PID(sideArm);
                slowArmMovement(.55,-148);
                break;
            case "hover":
                sideArm.setTargetPosition(-20);
                sideArm.setMode(DcMotor.RunMode.RUN_TO_POSITION);
                sideArm.setPower(speed);
                break;
            case "dropping":
                sideArm.setTargetPosition(-75);
                PID(sideArm);
                break;
            case "in":
                sideGrabber.setPosition(SIDE_CLAW_CLOSED);
                sleep(200);
                sideArm.setTargetPosition(-5);
                PID(sideArm);
                break;
        }
    }
    public void mainArmControl(String position, double speed){
        switch (position){
            case "out":
                timeout.reset();
                flipBackLeft.setTargetPosition(250);
                flipBackRight.setTargetPosition(250);
                flipBackRight.setMode(DcMotor.RunMode.RUN_TO_POSITION);
                flipBackLeft.setMode(DcMotor.RunMode.RUN_TO_POSITION);
                while (opModeIsActive() && flipBackLeft.isBusy() && timeout.seconds() < 2) {
                    flipBackLeft.setPower(speed);
                    flipBackRight.setPower(speed);
                }
                flipBackLeft.setPower(0);
                flipBackRight.setPower(0);
                break;
            case "hover":
                flipBackLeft.setTargetPosition(200);
                flipBackRight.setTargetPosition(200);
                flipBackRight.setMode(DcMotor.RunMode.RUN_TO_POSITION);
                flipBackLeft.setMode(DcMotor.RunMode.RUN_TO_POSITION);
                flipBackLeft.setPower(speed);
                flipBackRight.setPower(speed);
                break;
            case "in":
                timeout.reset();
                flipBackLeft.setTargetPosition(0);
                flipBackRight.setTargetPosition(0);
                flipBackRight.setMode(DcMotor.RunMode.RUN_TO_POSITION);
                flipBackLeft.setMode(DcMotor.RunMode.RUN_TO_POSITION);
                while (opModeIsActive() && flipBackLeft.isBusy() && timeout.seconds() < 2) {
                    flipBackLeft.setPower(speed);
                    flipBackRight.setPower(speed);
                }
                flipBackLeft.setPower(0);
                flipBackRight.setPower(0);
                break;
        }
    }

    public String scan () {
        String pos;
        telemetry.addData("x", openCV.getScreenPosition().x);
        telemetry.addData("y", openCV.getScreenPosition().y);
        telemetry.update();

        double distFromA = Math.abs(99- openCV.getScreenPosition().x);
        double distFromB = Math.abs(167 - openCV.getScreenPosition().x);
        double distFromC = Math.abs(236 - openCV.getScreenPosition().x);

        if (distFromA < distFromB){
            if (distFromA < distFromC){
                pos = "A";
            }else{
                pos = "C";
            }
        }else{
            if (distFromB < distFromC){
                pos = "B";
            }else{
                pos = "C";
            }

        }
        return pos;
    }


    public void slowArmMovement(double power, double target){

        double error = Math.abs(sideArm.getCurrentPosition() - target);

        while (error >= 5){
            error = Math.abs(sideArm.getCurrentPosition() - target);

            if (error >= (Math.abs(target)* (.666666666))){
                power = .3 * Math.signum(target);
            }

            sideArm.setPower(power);

        }

        sideArm.setPower(0);
    }



    public void PID(DcMotor movingMotor) {

        movingMotor.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);

        if (Math.abs(movingMotor.getCurrentPosition() - movingMotor.getTargetPosition()) >= 5) {
            double targetPosition = movingMotor.getTargetPosition();
            double integral = 0;
            ElapsedTime timer = new ElapsedTime();

            double error = movingMotor.getCurrentPosition() - targetPosition;
            double lastError = 0;
            double Kp = 0.0035;
            double Ki = (0.00012);
            double Kd = 0;

            while (opModeIsActive() && Math.abs(error) >= 5) {
                error = movingMotor.getCurrentPosition() - targetPosition;
                double deltaError = lastError - error;
                integral += deltaError * timer.time();
                double derivative = deltaError / timer.time();
                double power = (-1 * (Kp * error + Ki * integral));
                movingMotor.setPower(power);
                telemetry.addData("pos", movingMotor.getCurrentPosition());
                telemetry.addData("error", error);
                telemetry.addData("speed", power);
                telemetry.update();
                lastError = error;
                timer.reset();

            }
            movingMotor.setPower(0);
        }
    }



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

        right_front.setDirection(DcMotorSimple.Direction.REVERSE);

        initOtherHardware();
    }
    private void initOtherHardware(){
        modernRoboticsI2cGyro = hardwareMap.get(ModernRoboticsI2cGyro.class, "gyro");
        gyro = (IntegratingGyroscope) modernRoboticsI2cGyro;

        frontTouch = hardwareMap.get(DigitalChannel.class, "sensor_digital");
        frontTouch.setMode(DigitalChannel.Mode.INPUT);


        modernRoboticsI2cGyro.calibrate();
        while (!isStopRequested() && modernRoboticsI2cGyro.isCalibrating()) {
            telemetry.addLine("Gyro: Calibrating");
            telemetry.update();
        }
        telemetry.addLine("Gyro: Done calibrating");
        telemetry.update();

        frontClaw = hardwareMap.get(Servo.class, "frontclaw");
        odometryServo = hardwareMap.get(Servo.class, "odometryServo");


        flipBackLeft = hardwareMap.get(DcMotor.class, "left_flip");
        flipBackRight = hardwareMap.get(DcMotor.class, "right_flip");

        linear_slide = hardwareMap.get(DcMotor.class, "linear_slide");
        linear_slide.setDirection(DcMotorSimple.Direction.REVERSE);
        linear_slide.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        linear_slide.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);


        flipBackRight.setDirection(DcMotorSimple.Direction.REVERSE);
        flipBackLeft.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        flipBackRight.setMode(DcMotor.RunMode.RUN_USING_ENCODER);


        //rightSideDist = hardwareMap.get(DistanceSensor.class, "rightSideDist");
        //leftSideDist = hardwareMap.get(DistanceSensor.class, "leftSideDist");
        //frontLeftDist = hardwareMap.get(DistanceSensor.class, "frontLeftDist");
        //frontRightDist = hardwareMap.get(DistanceSensor.class, "frontRightDist");


        clampL = hardwareMap.get(Servo.class, "clampL");
        clampR = hardwareMap.get(Servo.class, "clampR");
        sideGrabber = hardwareMap.get(Servo.class, "sideGrabber");


        sideArm = hardwareMap.get(DcMotor.class, "skystoneArm");
        sideArm.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        sideArm.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);


    }
    public void initOpenCV(){
        int cameraMonitorViewId = hardwareMap.appContext.getResources().getIdentifier("cameraMonitorViewId", "id", hardwareMap.appContext.getPackageName());
        phoneCam = new OpenCvInternalCamera(OpenCvInternalCamera.CameraDirection.BACK, cameraMonitorViewId);
        phoneCam.openCameraDevice();
        openCV = new OpenCV();
        phoneCam.setPipeline(openCV);
        phoneCam.startStreaming(320, 240, OpenCvCameraRotation.SIDEWAYS_LEFT);

    }

    private double calculateX(double desiredAngle, double speed) {
        return Math.sin(Math.toRadians(desiredAngle)) * speed;
    }
    private double calculateY(double desiredAngle, double speed) {
        return Math.cos(Math.toRadians(desiredAngle)) * speed;
    }

}