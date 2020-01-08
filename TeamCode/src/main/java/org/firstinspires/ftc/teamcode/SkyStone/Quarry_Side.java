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
import com.qualcomm.robotcore.util.Range;
import com.qualcomm.robotcore.util.ReadWriteFile;
import com.vuforia.CameraDevice;

import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.robotcore.external.navigation.AxesOrder;
import org.firstinspires.ftc.robotcore.external.navigation.AxesReference;
import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit;
import org.firstinspires.ftc.robotcore.internal.system.AppUtil;
import org.firstinspires.ftc.teamcode.SkyStone.Odometry.OdometryGlobalCoordinatePosition;
import org.openftc.easyopencv.OpenCvCamera;
import org.openftc.easyopencv.OpenCvCameraRotation;
import org.openftc.easyopencv.OpenCvInternalCamera;

import java.io.File;
import java.security.Policy;


@Autonomous(name = "Quarry Side")
public class Quarry_Side extends LinearOpMode {


    String foundationPos = "";


    public DistanceSensor rightSideDist;
    public DistanceSensor leftSideDist;
    public DistanceSensor frontLeftDist;
    public DistanceSensor frontRightDist;


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


        rightSideDist = hardwareMap.get(DistanceSensor.class, "rightSideDist");
        leftSideDist = hardwareMap.get(DistanceSensor.class, "leftSideDist");
        frontLeftDist = hardwareMap.get(DistanceSensor.class, "frontLeftDist");
        frontRightDist = hardwareMap.get(DistanceSensor.class, "frontRightDist");


        telemetry.addData("Status", "Init Complete");
        telemetry.update();







        waitForStart();


        frontClaw.setPosition(.7);
        sleep(800);





        //Move close to the stone line
        moveDistanceEncoder(23, .5);


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


        //move to be aligned with the left most stone. (no scanning for skystone yet)
        strafeEncoder(-6, .5);

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


        //long strafe
        modernRoboticsI2cGyro.resetZAxisIntegrator();
        double savedRot = modernRoboticsI2cGyro.getIntegratedZValue();   //save rotation (0)
        moveDistanceEncoder(-1, .5);    //move forward a little to get ready for the strafe
        strafeEncoder(-20, .5);         //strafe halfway
        double disOff = savedRot - modernRoboticsI2cGyro.getIntegratedZValue();   //get the amount it rotated during the strafe
        turnDegree(disOff, .45,1);     //turn by the amount off
        strafeEncoder(-24, .5);      //strafe the rest of the way

        disOff = savedRot - modernRoboticsI2cGyro.getIntegratedZValue();   //get the amount it rotated during the strafe
        turnDegree(disOff, .45,1);     //turn by the amount off

        //Google Doc Code Here
       moveDistanceEncoder(-1,.75);


        //use distance sensor to find where the foundation is
        while (opModeIsActive() && leftSideDist.getDistance(DistanceUnit.INCH) < 15) {
            telemetry.addData("dist",leftSideDist.getDistance(DistanceUnit.INCH));
            telemetry.update();
            sleep(1500);
        }
        if ((leftSideDist.getDistance(DistanceUnit.INCH)) < 30) {
            foundationPos = "shortOnWall";
        } else {
            foundationPos = "unMoved";
        }

        moveDistanceEncoder(1,.75);



        switch (foundationPos){
            case "shortOnWall":
                turnDegree(90,.5,1);
                moveDistanceEncoder(6.5, .5);
                strafeEncoder(-4,.5);
                break;
            case "longOnWall":
                turnDegree(90,.5,1);
                moveDistanceEncoder(10,.7);
                break;
            case "unMoved":
                //strafeEncoder(-10,.5);
                break;
        }


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

        //open claw
        frontClaw.setPosition(1);
        sleep(1000);

        //Flip the linear slide back into the robot
        flipBackLeft.setTargetPosition(0);
        flipBackRight.setTargetPosition(0);
        flipBackRight.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        flipBackLeft.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        while (opModeIsActive() && flipBackLeft.isBusy()) {
            flipBackLeft.setPower(.2);
            flipBackRight.setPower(.2);
        }


        //park on line
        switch (foundationPos){
            case "shortOnWall":

                strafeEncoder(2,.5);
                moveDistanceEncoder(-15,.5);
                break;
            case "longOnWall":
                moveDistanceEncoder(-20,.7);
                break;
            case "unMoved":
                //moveDistanceEncoder(3,.5);
                strafeEncoder(20,.5);
                moveDistanceEncoder(3,.5);
                break;
        }






        //36-38, long side against wall
        //26-28, short side against wall
        //


        //scanCV();


        //scan values   {(223/225), (154/197), (284/285)}



        //collect the stones
        /*
        boolean skyStoneCollected = false;
        boolean leftCollected = false;
        boolean middleCollected = false;
        boolean rightCollected = false;

        while (opModeIsActive() && stonesLeft == true) {

            if (!skyStoneCollected) {
                if (configuration == "A") {
                    grabStone("left");
                    leftCollected = true;
                    skyStoneCollected = true;
                } else if (configuration == "B") {
                    grabStone("middle");
                    middleCollected = true;
                    skyStoneCollected = true;
                } else if (configuration == "C") {
                    grabStone("right");
                    skyStoneCollected = true;
                    rightCollected = true;
                }
            } else {

                if (!leftCollected) {
                    grabStone("left");
                    leftCollected = true;
                } else if (!middleCollected) {
                    grabStone("middle");
                    middleCollected = true;
                } else if (!rightCollected) {
                    grabStone("right");
                    rightCollected = true;
                } else {
                    stonesLeft = false;
                }

            }

            if (stonesLeft == true) {
                placeOnFoundation();

                returnToHomingPosition();
            }

        }

         */

        //park on middle line
        //drive from the homing position


        //globalPositionUpdate.stop();


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

        modernRoboticsI2cGyro.resetZAxisIntegrator();
        double integratedZ = modernRoboticsI2cGyro.getIntegratedZValue();
        while (opModeIsActive() && ((Math.abs(verticalRight.getCurrentPosition() - targetRight) >= 100) &&
                (Math.abs(verticalLeft.getCurrentPosition() - targetLeft) >= 100))) {


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


            if (degrees > 0 && integratedZ > degrees / 2) {
                leftSpeed = -.25;
                rightSpeed = .25;
            } else if (degrees < 0 && integratedZ > degrees / 2) {
                leftSpeed = .25;
                rightSpeed = -.25;
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


    private void scanCV() {

        int cameraMonitorViewId = hardwareMap.appContext.getResources().getIdentifier("cameraMonitorViewId", "id", hardwareMap.appContext.getPackageName());
        phoneCam = new OpenCvInternalCamera(OpenCvInternalCamera.CameraDirection.BACK, cameraMonitorViewId);
        phoneCam.openCameraDevice();
        skyStoneDetector = new SkystoneDetector();
        phoneCam.setPipeline(skyStoneDetector);
        phoneCam.startStreaming(320, 240, OpenCvCameraRotation.SIDEWAYS_LEFT);

        while (opModeIsActive() && configuration == "Not Decided") {
            while (skyStoneDetector.getScreenPosition().x == 0) {
                telemetry.addData("Stone Position X", skyStoneDetector.getScreenPosition().x);
                telemetry.update();
            }
            double xPos = skyStoneDetector.getScreenPosition().x;

            double leftDist = Math.abs(284 - xPos);
            double midDist = Math.abs(154 - xPos);
            double rightDist = Math.abs(223 - xPos);

            String smallest = "";

            if (leftDist < midDist) {
                if (leftDist < rightDist) {
                    smallest = "leftDist";
                } else if (leftDist > rightDist) {
                    smallest = "rightDist";
                }
            } else if (leftDist > midDist) {
                if (midDist < rightDist) {
                    smallest = "midDist";
                } else if (midDist > rightDist) {
                    smallest = "rightDist";
                }
            }

            if (smallest == "leftDist") {
                configuration = "A";
                telemetry.addLine("left");
            } else if (smallest == "midDist") {
                configuration = "B";
                telemetry.addLine("mid");
            } else if (smallest == "rightDist") {
                configuration = "C";
                telemetry.addLine("right");
            }

            telemetry.update();
            sleep(3000);
        }
        phoneCam.stopStreaming();

    }


    private void grabStone(String position) {


        if (position == "left") {
            //drive sideways left
        } else if (position == "middle") {
            //drive sideways to align
        } else if (position == "right") {
            //drive sideways right
        }

        //drive forward
        //moveDistancePID(.5, 30);

        // grab it

        //back up
        //moveDistancePID(.5,-30);

        if (position == "left") {
            //drive back
        } else if (position == "middle") {
            //drive back
        } else if (position == "right") {
            //drive back
        }

    }


    private void placeOnFoundation() {
        //back up more if needed

        //turn based on witch side your on
        //use distance sensor to see witch team your on an turn accordingly

        //drive forward until reach the foundation
        //wait till safe auto and test to find how to tell where the foundation is

        //turn by how you need to

        //drive forward

        //drop

        //dive back stuff
    }


    private void returnToHomingPosition() {
        //maybe use vuforia to line self up with target
        //add stuff late if needed
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
