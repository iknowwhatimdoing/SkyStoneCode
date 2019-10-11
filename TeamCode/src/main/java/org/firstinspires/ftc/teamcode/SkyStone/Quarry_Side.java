//Copyright (c) 2019 FIRST. All rights reserved.

package org.firstinspires.ftc.teamcode.SkyStone;

import com.disnodeteam.dogecv.CameraViewDisplay;
import com.disnodeteam.dogecv.DogeCV;
import com.disnodeteam.dogecv.detectors.GenericDetector;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.DcMotor;
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

import java.util.ArrayList;
import java.util.List;

import static org.firstinspires.ftc.robotcore.external.navigation.AngleUnit.DEGREES;
import static org.firstinspires.ftc.robotcore.external.navigation.AxesOrder.XYZ;
import static org.firstinspires.ftc.robotcore.external.navigation.AxesOrder.YZX;
import static org.firstinspires.ftc.robotcore.external.navigation.AxesReference.EXTRINSIC;
import static org.firstinspires.ftc.robotcore.external.navigation.VuforiaLocalizer.CameraDirection.BACK;


@Autonomous(name = "Vuforia Scan Quarry")
public class Quarry_Side extends LinearOpMode {

    //SkyStoneHardware robot = new SkyStoneHardware();


    //drive by encoders math
    static final double COUNTS_PER_MOTOR_REV = 2786;
    static final double DRIVE_GEAR_REDUCTION = 2.0;     // This is < 1.0 if geared UP
    static final double WHEEL_DIAMETER_INCHES = 4.0;
    static final double COUNTS_PER_INCH = (COUNTS_PER_MOTOR_REV * DRIVE_GEAR_REDUCTION) / (WHEEL_DIAMETER_INCHES * 3.1415);


    private String configuration = "Not Decided";
    private boolean stonesLeft = true;
    private RedFoundationDetector detector;


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

        //robot.init(hardwareMap);


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


        //drive to where you can see the stones
        //moveDistancePID(.5,20);

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

}
