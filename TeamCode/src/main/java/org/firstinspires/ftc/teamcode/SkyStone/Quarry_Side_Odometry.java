//Copyright (c) 2019 FIRST. All rights reserved.

package org.firstinspires.ftc.teamcode.SkyStone;

import com.qualcomm.hardware.modernrobotics.ModernRoboticsI2cGyro;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.DigitalChannel;
import com.qualcomm.robotcore.hardware.DistanceSensor;
import com.qualcomm.robotcore.hardware.IntegratingGyroscope;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.util.ElapsedTime;
import com.qualcomm.robotcore.util.Range;

import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit;
import org.firstinspires.ftc.teamcode.SkyStone.Odometry.OdometryGlobalCoordinatePosition;
import org.firstinspires.ftc.teamcode.SkyStone.Tests.OpenCV;
import org.openftc.easyopencv.OpenCvCamera;
import org.openftc.easyopencv.OpenCvCameraRotation;
import org.openftc.easyopencv.OpenCvInternalCamera;


@Autonomous(name = "Quarry Side (odometry)")
public class Quarry_Side_Odometry extends LinearOpMode {


    private static final double ALLOWED_ERROR = 1;
    private static final double SIDE_CLAW_OPEN = 1;
    private static final double SIDE_CLAW_CLOSED = .43;

    private OpenCvCamera phoneCam;
    private OpenCV openCV;


    ElapsedTime timeout = new ElapsedTime();


    public DistanceSensor rightSideDist;
    //public DistanceSensor leftSideDist;
    //public DistanceSensor frontLeftDist;
    //public DistanceSensor frontRightDist;


    Servo odometryServo;
    //Servo capstone;

    DcMotor sideArm;
    Servo clampL, clampR, sideGrabber;


    Servo frontClaw;
    DcMotor flipBackLeft, flipBackRight;
    DcMotor linear_slide;
    DcMotor right_front, right_back, left_front, left_back;    //Drive Motors
    DcMotor verticalLeft, verticalRight, horizontal;           //Odometry Wheels

    final double COUNTS_PER_INCH = 307.699557;

    String rfName = "right_front", rbName = "right_back", lfName = "left_front", lbName = "left_back";
    String verticalLeftEncoderName = lfName, verticalRightEncoderName = rfName, horizontalEncoderName = rbName;

    OdometryGlobalCoordinatePosition globalPositionUpdate;


    DigitalChannel frontTouch;


    public IntegratingGyroscope gyro;
    public ModernRoboticsI2cGyro modernRoboticsI2cGyro;


    @Override
    public void runOpMode() {
        initDriveHardwareMap(rfName, rbName, lfName, lbName, verticalLeftEncoderName, verticalRightEncoderName, horizontalEncoderName);
        initOtherHardware();




        int cameraMonitorViewId = hardwareMap.appContext.getResources().getIdentifier("cameraMonitorViewId", "id", hardwareMap.appContext.getPackageName());
        phoneCam = new OpenCvInternalCamera(OpenCvInternalCamera.CameraDirection.BACK, cameraMonitorViewId);
        phoneCam.openCameraDevice();
        openCV = new OpenCV();
        phoneCam.setPipeline(openCV);
        phoneCam.startStreaming(320, 240, OpenCvCameraRotation.SIDEWAYS_LEFT);


        sleep(2000);
        telemetry.addData("Status", "Init Complete");
        telemetry.update();








        waitForStart();


        //Create and start GlobalCoordinatePosition thread to constantly update the global coordinate positions
        globalPositionUpdate = new OdometryGlobalCoordinatePosition(verticalLeft, verticalRight, horizontal, COUNTS_PER_INCH, 75);
        Thread positionThread = new Thread(globalPositionUpdate);
        positionThread.start();

        //globalPositionUpdate.reverseRightEncoder();
        globalPositionUpdate.reverseNormalEncoder();

        //lower the middle odometry wheel
        odometryServo.setPosition(.5);
        //make sure the claw is closed and foundation Clamp is up
        frontClaw.setPosition(0);
        clampR.setPosition(1);
        clampL.setPosition(.6);





        String pattern = scan();
        // A: 99,   B: 167,  C: 236

        telemetry.addData("Pattern", pattern);
        telemetry.update();
        phoneCam.stopStreaming();
        //sleep(200);




        //move a little depending on where the skystone is
        switch (pattern){
            case "A":
                goToPosition(23,1,.5,0,5);

                break;
            case "B":
                goToPosition(23,-3,.5,0,5);
                break;
            case "C":
                goToPosition(23,-5,.5,0,5);
                break;
        }

        //attempt to lower the side arm. If it can't, back up and try again
        timeout.reset();
        sideGrabber.setPosition(1);
        sideArm.setTargetPosition(-150);
        sideArm.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        while(sideArm.isBusy() && opModeIsActive() && timeout.seconds() < 2) {
            sideArm.setPower(.4);
        }
        sideArm.setPower(0);




        goToPosition(3,0,.5,0,5);

        //close the claw on the skystone. Wait for it to close
        sideGrabber.setPosition(SIDE_CLAW_CLOSED);
        sleep(300);



        //flip the linear slide down
        mainArmControl("out", .6);


        //flip the side arm into the robot
        sideArmControl("in", .4);










        /*
        deliver stone 1
         */


        //move to the other side (near the foundation)
        switch (pattern){
            case "A":
                goToPosition(0,76,.85,0,5);
                break;
            case "B":
                goToPosition(0,83,.85,0,5);
                break;
            case "C":
                goToPosition(0,92,.85,0,5);
                break;
        }

        //push into the wall to reset position
        timeout.reset();
        while (timeout.seconds() < .5){
            driveAll(.3);
        }
        driveAll(0);


        //lower the arm to place the stone
        sideArmControl("dropping", .4);

        //open the claw
        sideGrabber.setPosition(SIDE_CLAW_OPEN);
        sleep(300);

        //flip the arm back into the robot
        sideArmControl("in", .6);


        //move back to the stones
        switch (pattern){
            case "A":
                goToPosition(0,-85,.85,0,5);

                break;
            case "B":
                goToPosition(0,-92,.85,0,5);
                break;
            case "C":
                goToPosition(0,-101,.85,0,5);
                //push against the wall to make sure your in the right spot
                timeout.reset();
                while (timeout.seconds() < .5){
                    driveAll(-.3);
                }
                driveAll(0);

                break;
        }

        //flip the claw down
        sideArmControl("out", .4);

        //close claw on the stone
        sideGrabber.setPosition(SIDE_CLAW_CLOSED);
        sleep(300);

        //flip the claw in
        sideArmControl("in",.6);








        /*
        deliver stone 2
         */


        //move back to the foundation side
        switch (pattern){
            case "A":
                goToPosition(0,77,.85,0,5);
                break;
            case "B":
                goToPosition(0,84,.85,0,5);
                break;
            case "C":
                goToPosition(0,93,.85,0,5);
                break;
        }

        //lower the arm to place the stone
        sideArmControl("dropping", .4);

        //open the claw
        sideGrabber.setPosition(SIDE_CLAW_OPEN);
        sleep(300);

        //flip the arm back into the robot
        sideArmControl("in", .6);







        //----------delete below if the robot doesn't have time--------------

        //back up to the stone line for the final stone
        switch (pattern){
            case "A":
                //back up to align with the second stone in the row
                goToPosition(0,-65,.85,0,5);
                break;
            default:
                //back up to align with the first stone in the row
                goToPosition(0,-60,.85,0,5);
                break;
        }


        //flip the claw down
        sideArmControl("out", .4);

        //close claw on the stone
        sideGrabber.setPosition(SIDE_CLAW_CLOSED);
        sleep(300);

        //flip the claw in
        sideArmControl("in",.6);









        /*
        deliver stone 3
         */


        //move back to the foundation side
        switch (pattern){
            case "A":
                goToPosition(0,65,.85,0,5);
                break;
            default:
                goToPosition(0,60,.85,0,5);
                break;
        }

        //lower the arm to place the stone
        sideArmControl("dropping", .4);

        //open the claw
        sideGrabber.setPosition(SIDE_CLAW_OPEN);
        sleep(300);

        //flip the arm back into the robot
        sideArmControl("in", .6);





        /*
        moving the foundation
         */

        //back up from the foundation to turn
        goToPosition(-5,0,.85,0,5);


        //turn to face the foundation
        turnDegree(-90,.8,5);

        //move the side arm and flip in the main arm
        sideArmControl("hover", .6);
        mainArmControl("in", .7);

        ElapsedTime allowedMoveTime = new ElapsedTime();
        while (opModeIsActive() && frontTouch.getState() && allowedMoveTime.seconds() < 5) {
            moveWithoutEndGoal(.4);
        }
        driveAll(.1);

        //close the clamps
        clampL.setPosition(1);
        clampR.setPosition(.8);
        sleep(300);

        driveAll(0);

        //back up with the foundation
        goToPosition(0,-5,.6,0,5);

        //turn a bit with the foundation, back up, and finish the turn
        foundationMovement(45, .8,-.3);
        goToPosition(0,-5,.6,0,5);
        foundationMovement(45,.8,-.3);

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

        goToPosition(15,0,.5,0,5);


        moveTime.reset();
        while (opModeIsActive() && moveTime.seconds() < .4) {
            driveAll(.5);
        }
        driveAll(0);






        //----park----------------


        //back up
        goToPosition(0,-5,.85,0,5);


        //lower the main arm
        mainArmControl("out", .8);

        //bring the side arm into the robot
        sideArmControl("in", .6);

        //turn right 90, drop the capstone, and turn right another 90 (180 total)
        turnDegree(-90,.8,5);
        frontClaw.setPosition(1);
        sleep(300);
        turnDegree(-90, .8 ,5);

        //move to park
        goToPosition(0,10,.85,0,5);


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
            left_front.setPower(Range.clip((robotMovementXComponent + robotMovementYComponent + pivotCorrection), -1, 1));
            left_back.setPower(Range.clip((-robotMovementXComponent + robotMovementYComponent + pivotCorrection), -1, 1));
            right_front.setPower(Range.clip((-robotMovementXComponent + robotMovementYComponent - pivotCorrection), -1, 1));
            right_back.setPower(Range.clip((robotMovementXComponent + robotMovementYComponent - pivotCorrection), -1, 1));
        }
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
                timeout.reset();
                sideArm.setTargetPosition(-150);
                sideArm.setMode(DcMotor.RunMode.RUN_TO_POSITION);
                while(sideArm.isBusy() && opModeIsActive() && timeout.seconds() < 2) {
                    sideArm.setPower(speed);
                }
                sideArm.setPower(0);
                break;
            case "hover":
                sideArm.setTargetPosition(-10);
                sideArm.setMode(DcMotor.RunMode.RUN_TO_POSITION);
                sideArm.setPower(speed);
                break;
            case "dropping":
                timeout.reset();
                sideArm.setTargetPosition(-100);
                sideArm.setMode(DcMotor.RunMode.RUN_TO_POSITION);
                while(sideArm.isBusy() && opModeIsActive() && timeout.seconds() < 2) {
                    sideArm.setPower(speed);
                }
                sideArm.setPower(0);
                break;
            case "in":
                timeout.reset();
                sideArm.setTargetPosition(10);
                sideArm.setMode(DcMotor.RunMode.RUN_TO_POSITION);
                while(sideArm.isBusy() && opModeIsActive() && timeout.seconds() < 2) {
                    sideArm.setPower(speed);
                }
                sideArm.setPower(0);
                break;
        }
    }

    public void mainArmControl(String position, double speed){
        switch (position){
            case "out":
                timeout.reset();
                flipBackLeft.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
                flipBackRight.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
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
                flipBackLeft.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
                flipBackRight.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
                flipBackLeft.setTargetPosition(200);
                flipBackRight.setTargetPosition(200);
                flipBackRight.setMode(DcMotor.RunMode.RUN_TO_POSITION);
                flipBackLeft.setMode(DcMotor.RunMode.RUN_TO_POSITION);
                flipBackLeft.setPower(speed);
                flipBackRight.setPower(speed);
                break;
            case "in":
                timeout.reset();
                flipBackLeft.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
                flipBackRight.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
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
        String pos = "";
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

        telemetry.addData("Status", "Hardware Map Init Complete");
        telemetry.update();
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
        //capstone = hardwareMap.get(Servo.class, "capstone");


        flipBackLeft = hardwareMap.get(DcMotor.class, "left_flip");
        flipBackRight = hardwareMap.get(DcMotor.class, "right_flip");

        linear_slide = hardwareMap.get(DcMotor.class, "linear_slide");
        linear_slide.setDirection(DcMotorSimple.Direction.REVERSE);
        linear_slide.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        linear_slide.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);


        flipBackRight.setDirection(DcMotorSimple.Direction.REVERSE);
        flipBackLeft.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        flipBackRight.setMode(DcMotor.RunMode.RUN_USING_ENCODER);


        rightSideDist = hardwareMap.get(DistanceSensor.class, "rightSideDist");
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

    private double calculateX(double desiredAngle, double speed) {
        return Math.sin(Math.toRadians(desiredAngle)) * speed;
    }
    private double calculateY(double desiredAngle, double speed) {
        return Math.cos(Math.toRadians(desiredAngle)) * speed;
    }

}