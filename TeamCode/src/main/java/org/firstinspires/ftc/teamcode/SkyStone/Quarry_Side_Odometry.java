//Copyright (c) 2019 FIRST. All rights reserved.

package org.firstinspires.ftc.teamcode.SkyStone;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.util.ElapsedTime;
import com.qualcomm.robotcore.util.Range;
import com.vuforia.CameraDevice;
import com.vuforia.HINT;
import com.vuforia.Vuforia;

import org.firstinspires.ftc.robotcore.external.ClassFactory;
import org.firstinspires.ftc.robotcore.external.matrices.OpenGLMatrix;
import org.firstinspires.ftc.robotcore.external.matrices.VectorF;
import org.firstinspires.ftc.robotcore.external.navigation.Orientation;
import org.firstinspires.ftc.robotcore.external.navigation.VuforiaLocalizer;
import org.firstinspires.ftc.robotcore.external.navigation.VuforiaTrackable;
import org.firstinspires.ftc.robotcore.external.navigation.VuforiaTrackableDefaultListener;
import org.firstinspires.ftc.robotcore.external.navigation.VuforiaTrackables;
import org.firstinspires.ftc.teamcode.SkyStone.Detectors.RedFoundationDetector;
import org.firstinspires.ftc.teamcode.SkyStone.Odometry.OdometryGlobalCoordinatePosition;

import java.util.ArrayList;
import java.util.List;

import static org.firstinspires.ftc.robotcore.external.navigation.AngleUnit.DEGREES;
import static org.firstinspires.ftc.robotcore.external.navigation.AxesOrder.XYZ;
import static org.firstinspires.ftc.robotcore.external.navigation.AxesOrder.YZX;
import static org.firstinspires.ftc.robotcore.external.navigation.AxesReference.EXTRINSIC;
import static org.firstinspires.ftc.robotcore.external.navigation.VuforiaLocalizer.CameraDirection.BACK;
import org.firstinspires.ftc.teamcode.SkyStone.Odometry.OdometryGlobalCoordinatePosition;



@Autonomous(name = "Quarry Side (Odometry)")
public class Quarry_Side_Odometry extends LinearOpMode {


    //Drive motors
    DcMotor right_front, right_back, left_front, left_back;
    //Odometry Wheels
    DcMotor verticalLeft, verticalRight, horizontal;

    final double COUNTS_PER_INCH = 307.699557;

    //Hardware Map Names for drive motors and odometry wheels. THIS WILL CHANGE ON EACH ROBOT, YOU NEED TO UPDATE THESE VALUES ACCORDINGLY
    String rfName = "rf", rbName = "rb", lfName = "lf", lbName = "lb";
    String verticalLeftEncoderName = rbName, verticalRightEncoderName = lfName, horizontalEncoderName = rfName;

    OdometryGlobalCoordinatePosition globalPositionUpdate;

    private String configuration = "Not Decided";
    private boolean stonesLeft = true;
    //private RedFoundationDetector detector;


    private static final double middleBound = 0;

    private static final VuforiaLocalizer.CameraDirection CAMERA_CHOICE = BACK;
    private static final boolean PHONE_IS_PORTRAIT = false;

    private static final String VUFORIA_KEY =
            "AeVN5hL/////AAABmeiiuuNfCUG7o7nFHIB8wOIJd0HGVkf4o9TQNISpW19DjyAqW6a1+DIXgQEWJlZoXbHJhjRNbpdhsZiyF8YZS7mSkspYXxB9vhRl3NdBzba6lb450R331LCujbFW4f1z5ZiERvZsxUn1bl8FeugGjQAag3tO7DU7ZNFMRVev0pTtIo0tIRtVlW1zmZqCAQCmCLtAUPjpUv2atadLfVqleYVydV3AN2upMHu14tb3zBuOmM2SfHu//Nuo8xh/U2a0Joi9B286UYurYN7S/+2mzpe6cFcqdDjVV7C3sjvHUw3ivtGy9M7BXwsVZkF/aSQr0cjDfTv/si4DW8lm/WLG5thfi2kHv/B21I2C67dt/oGI";

    private static final float mmPerInch = 25.4f;
    // Constant for Stone target
    private static final float stoneZ = 2.00f * mmPerInch;

    // Class Members
    private OpenGLMatrix lastLocation = null;
    private VuforiaLocalizer vuforia = null;
    private boolean targetVisible = false;
    private float phoneXRotate = 0;
    private float phoneYRotate = 0;
    private float phoneZRotate = 0;

    @Override
    public void runOpMode() {

        //Initialize hardware map values. PLEASE UPDATE THESE VALUES TO MATCH YOUR CONFIGURATION
        initDriveHardwareMap(rfName, rbName, lfName, lbName, verticalLeftEncoderName, verticalRightEncoderName, horizontalEncoderName);

        telemetry.addData("Status", "Init Complete");
        telemetry.update();


        int cameraMonitorViewId = hardwareMap.appContext.getResources().getIdentifier("cameraMonitorViewId", "id", hardwareMap.appContext.getPackageName());
        VuforiaLocalizer.Parameters parameters = new VuforiaLocalizer.Parameters(cameraMonitorViewId);
        // VuforiaLocalizer.Parameters parameters = new VuforiaLocalizer.Parameters();

        parameters.vuforiaLicenseKey = VUFORIA_KEY;
        parameters.cameraDirection = CAMERA_CHOICE;

        vuforia = ClassFactory.getInstance().createVuforia(parameters);
        VuforiaTrackables targetsSkyStone = this.vuforia.loadTrackablesFromAsset("Skystone");
        Vuforia.setHint(HINT.HINT_MAX_SIMULTANEOUS_IMAGE_TARGETS, 3);


        VuforiaTrackable stoneTarget = targetsSkyStone.get(0);
        stoneTarget.setName("Stone Target");

        List<VuforiaTrackable> allTrackables = new ArrayList<VuforiaTrackable>();
        allTrackables.addAll(targetsSkyStone);


        // If you are standing in the Red Alliance Station looking towards the center of the field,
        //    - The X axis runs from your left to the right. (positive from the center to the right)
        //    - The Y axis runs from the Red Alliance Station towards the other side of the field
        //      where the Blue Alliance Station is. (Positive is from the center, towards the BlueAlliance station)
        //    - The Z axis runs from the floor, upwards towards the ceiling.  (Positive is above the floor)
        //
        //Before being transformed, each target image is conceptually located at the origin of the field's
        //  coordinate system (the center of the field), facing up.
        //

        stoneTarget.setLocation(OpenGLMatrix
                .translation(0, 0, stoneZ)
                .multiplied(Orientation.getRotationMatrix(EXTRINSIC, XYZ, DEGREES, 90, 0, -90)));


        // Lock screen in portrait mode
        //
        // The phone starts out lying flat, with the screen facing Up and with the physical top of the phone
        // pointing to the LEFT side of the Robot.

        // Rotate to say the camera is facing outward on side
        if (CAMERA_CHOICE == BACK) {
            phoneYRotate = -90;
        } else {
            phoneYRotate = 90;
        }

        // Rotate the phone vertical if in portrait
        if (PHONE_IS_PORTRAIT) {
            phoneXRotate = 90;
        }


        final float CAMERA_FORWARD_DISPLACEMENT = 4.0f * mmPerInch;   // Inches in front of the robot's center
        final float CAMERA_VERTICAL_DISPLACEMENT = 8.0f * mmPerInch;   // Inches off the ground
        final float CAMERA_LEFT_DISPLACEMENT = 0;     // Inches displaced. Negative = off center to the left

        OpenGLMatrix robotFromCamera = OpenGLMatrix
                .translation(CAMERA_FORWARD_DISPLACEMENT, CAMERA_LEFT_DISPLACEMENT, CAMERA_VERTICAL_DISPLACEMENT)
                .multiplied(Orientation.getRotationMatrix(EXTRINSIC, YZX, DEGREES, phoneYRotate, phoneZRotate, phoneXRotate));


        for (VuforiaTrackable trackable : allTrackables) {
            ((VuforiaTrackableDefaultListener) trackable.getListener()).setPhoneInformation(robotFromCamera, parameters.cameraDirection);
        }


        waitForStart();

        //Create and start GlobalCoordinatePosition thread to constantly update the global coordinate positions
        globalPositionUpdate = new OdometryGlobalCoordinatePosition(verticalLeft, verticalRight, horizontal, COUNTS_PER_INCH, 75);
        Thread positionThread = new Thread(globalPositionUpdate);
        positionThread.start();

        globalPositionUpdate.reverseRightEncoder();
        globalPositionUpdate.reverseNormalEncoder();

        //drive to where you can see the stones
        //moveDistancePID(.5,20);
        //goToPosition(1,1,.5,0,.5);

        //scan the stones
        scan(targetsSkyStone, allTrackables);
        targetsSkyStone.deactivate();


        //collect the stones
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

        //park on middle line
        //drive from the homing position


        globalPositionUpdate.stop();


    }







    /*-------------------------------------------------
     *
     * Methods
     *
     * --------------------------------------------------*/

/*
    public void moveDistancePID(double speed, double inches) {


        double targetPosition = (int) (inches * COUNTS_PER_INCH);
        double intergral = 0;
        double iteratoins = 0;
        ElapsedTime timer = new ElapsedTime();

        double error = robot.leftfront.getCurrentPosition() - targetPosition;
        double lastError = 0;
        double Kp = 0.01;
        double Ki = 0.008;
        double Kd = 0.008;

        while (opModeIsActive() && Math.abs(error) <= 5) {
            error = robot.leftfront.getCurrentPosition() - targetPosition;
            double deltaError = lastError - error;
            intergral += deltaError * timer.time();
            double derivative = deltaError / timer.time();
            robot.driveAll(Range.clip(Kp * error + Ki * intergral + Kd + derivative,-speed,speed));
            error = lastError;
            iteratoins++;
            timer.reset();

        }


    }

 */


    public void goToPosition(double targetXPos, double targetYPos, double power, double desiredRobotOrientation, double allowedDistanceError) {
        targetXPos *= COUNTS_PER_INCH;
        targetYPos *=COUNTS_PER_INCH;
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
            left_front.setPower(Range.clip((robotMovementXComponent + robotMovementYComponent + pivotCorrection ),-1,1));
            left_back.setPower(Range.clip((-robotMovementXComponent + robotMovementYComponent + pivotCorrection),-1,1));
            right_front.setPower(Range.clip((-robotMovementXComponent + robotMovementYComponent - pivotCorrection),-1,1));
            right_back.setPower(Range.clip((robotMovementXComponent + robotMovementYComponent - pivotCorrection),-1,1));
        }
    }






    private void scan(VuforiaTrackables targetsSkyStone, List<VuforiaTrackable> allTrackables) {

        CameraDevice.getInstance().setFlashTorchMode(true);

        // Note: To use the remote camera preview:
        // AFTER you hit Init on the Driver Station, use the "options menu" to select "Camera Stream"
        // Tap the preview window to receive a fresh image.

        targetsSkyStone.activate();
        while (!isStopRequested() && configuration == "Not Decided") {


            // check all the trackable targets to see which one (if any) is visible.
            targetVisible = false;
            ElapsedTime waitToSeeTarget = new ElapsedTime();

            while (!isStopRequested() && !targetVisible && waitToSeeTarget.seconds() < 2) {
                for (VuforiaTrackable trackable : allTrackables) {
                    if (((VuforiaTrackableDefaultListener) trackable.getListener()).isVisible()) {
                        telemetry.addData("Visible Target", trackable.getName());


                        if (trackable.getName().equals("Stone Target")) {
                            targetVisible = true;

                        }


                        // getUpdatedRobotLocation() will return null if no new information is available since
                        // the last time that call was made, or if the trackable is not currently visible.
                        OpenGLMatrix robotLocationTransform = ((VuforiaTrackableDefaultListener) trackable.getListener()).getUpdatedRobotLocation();
                        if (robotLocationTransform != null) {
                            lastLocation = robotLocationTransform;
                        }
                        break;
                    }
                }

            }

            // Provide feedback as to where the robot is located (if we know).
            if (targetVisible) {


                //drive up to it

                VectorF translation = lastLocation.getTranslation();

                ElapsedTime timer = new ElapsedTime();
                double xposition = 0;
                double iterations = 0;


                translation = lastLocation.getTranslation();
                // express position (translation) of robot in inches.
                telemetry.addData("Pos (in)", "{X, Y, Z} = %.1f, %.1f, %.1f",
                        translation.get(0) / mmPerInch, translation.get(1) / mmPerInch, translation.get(2) / mmPerInch);

                // express the rotation of the robot in degrees.
                Orientation rotation = Orientation.getOrientation(lastLocation, EXTRINSIC, XYZ, DEGREES);
                telemetry.addData("Rot (deg)", "{Roll, Pitch, Heading} = %.0f, %.0f, %.0f", rotation.firstAngle, rotation.secondAngle, rotation.thirdAngle);

                telemetry.update();


                telemetry.addData("x: ", translation.get(1) / mmPerInch);
                telemetry.update();
                sleep(2000);


                if (translation.get(1) / mmPerInch <= middleBound) {
                    configuration = "A";
                    telemetry.addLine("Pattern A");
                } else if (translation.get(1) / mmPerInch >= middleBound) {
                    configuration = "B";
                    telemetry.addLine("Pattern B");
                }
            } else {
                configuration = "C";
                telemetry.addLine("Pattern C");
            }
            telemetry.update();
        }


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

        left_front.setDirection(DcMotorSimple.Direction.REVERSE);
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
