//Copyright (c) 2019 FIRST. All rights reserved.

package org.firstinspires.ftc.teamcode.SkyStone;

import com.disnodeteam.dogecv.CameraViewDisplay;
import com.disnodeteam.dogecv.DogeCV;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.util.ElapsedTime;
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


@Autonomous(name = "Vuforia Scan Method")
public class Quarry_Side extends LinearOpMode {

    SkyStoneHardware robot = new SkyStoneHardware();


    private ElapsedTime runtime = new ElapsedTime();

    static final double COUNTS_PER_MOTOR_REV = 2786;
    static final double DRIVE_GEAR_REDUCTION = 2.0;     // This is < 1.0 if geared UP
    static final double WHEEL_DIAMETER_INCHES = 4.0;
    static final double COUNTS_PER_INCH = (COUNTS_PER_MOTOR_REV * DRIVE_GEAR_REDUCTION) / (WHEEL_DIAMETER_INCHES * 3.1415);


    private String configuration = "Not Decided";

    private RedFoundationDetector detector;


    private static final double leftBound = 0;
    private static final double rightBound = 0;

    private static final VuforiaLocalizer.CameraDirection CAMERA_CHOICE = BACK;
    private static final boolean PHONE_IS_PORTRAIT = false;

    private static final String VUFORIA_KEY =
            "AeVN5hL/////AAABmeiiuuNfCUG7o7nFHIB8wOIJd0HGVkf4o9TQNISpW19DjyAqW6a1+DIXgQEWJlZoXbHJhjRNbpdhsZiyF8YZS7mSkspYXxB9vhRl3NdBzba6lb450R331LCujbFW4f1z5ZiERvZsxUn1bl8FeugGjQAag3tO7DU7ZNFMRVev0pTtIo0tIRtVlW1zmZqCAQCmCLtAUPjpUv2atadLfVqleYVydV3AN2upMHu14tb3zBuOmM2SfHu//Nuo8xh/U2a0Joi9B286UYurYN7S/+2mzpe6cFcqdDjVV7C3sjvHUw3ivtGy9M7BXwsVZkF/aSQr0cjDfTv/si4DW8lm/WLG5thfi2kHv/B21I2C67dt/oGI";


    private static final float mmPerInch = 25.4f;

    // Constant for Stone Target
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

        robot.init(hardwareMap);


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


        //targetsSkyStone.deactivate();


        waitForStart();


        initDetector();
        findFoundation();
        if (detector != null) detector.disable();

        //drive around, move foundation, etc


        //scan(targetsSkyStone, allTrackables);


        // Disable Tracking when we are done;
        //targetsSkyStone.deactivate();


        // stuff to do when you know the configuration
        //collectionPath();

        // stuff after collection


    }






    /*-------------------------------------------------
     *
     * Methods
     *
     * --------------------------------------------------*/





    public void moveDistanvePID(double speed, double distance) {


        double targetPosition = (int)(distance * COUNTS_PER_INCH);
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
            robot.driveAll(Kp * error + Ki * intergral + Kd + derivative);
            error = lastError;
            iteratoins++;
            timer.reset();

        }


    }


    public void initDetector() {
        telemetry.addData("Status", "DogeCV 2019.1 - Gold Align Example");

        // Set up detector
        detector = new RedFoundationDetector(); // Create detector
        detector.init(hardwareMap.appContext, CameraViewDisplay.getInstance()); // Initialize it with the app context and camera
        detector.useDefaults(); // Set detector to use default settings

        // Optional tuning
        detector.alignSize = 200; // How wide (in pixels) is the range in which the gold object will be aligned. (Represented by green bars in the preview)
        detector.alignPosOffset = 0; // How far from center frame to offset this alignment zone.
        detector.downscale = 0.4; // How much to downscale the input frames

        detector.areaScoringMethod = DogeCV.AreaScoringMethod.MAX_AREA; // Can also be PERFECT_AREA
        //detector.perfectAreaScorer.perfectArea = 10000; // if using PERFECT_AREA scoring
        detector.maxAreaScorer.weight = 0.005; //

        detector.ratioScorer.weight = 5; //
        detector.ratioScorer.perfectRatio = 1.0; // Ratio adjustment

        detector.enable(); // Start the detector!


    }


    private void findFoundation() {

        while (!isStopRequested()) {
            telemetry.addData("IsAligned", detector.getAligned()); // Is the bot aligned with the gold mineral?
            telemetry.addData("X Pos", detector.getXPosition()); // Gold X position.
            telemetry.update();
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
            while (!isStopRequested() && !targetVisible) {
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

                //grab it, move it where it goes, come back and start agian

                VectorF translation = lastLocation.getTranslation();

                ElapsedTime timer = new ElapsedTime();
                while (!isStopRequested() && timer.seconds() < 8) {


                    translation = lastLocation.getTranslation();
                    // express position (translation) of robot in inches.
                    telemetry.addData("Pos (in)", "{X, Y, Z} = %.1f, %.1f, %.1f",
                            translation.get(0) / mmPerInch, translation.get(1) / mmPerInch, translation.get(2) / mmPerInch);

                    // express the rotation of the robot in degrees.
                    Orientation rotation = Orientation.getOrientation(lastLocation, EXTRINSIC, XYZ, DEGREES);
                    telemetry.addData("Rot (deg)", "{Roll, Pitch, Heading} = %.0f, %.0f, %.0f", rotation.firstAngle, rotation.secondAngle, rotation.thirdAngle);

                    telemetry.update();
                }


                if (translation.get(0) <= leftBound) {
                    configuration = "A";
                    telemetry.addLine("Patter A");
                } else if (translation.get(0) >= leftBound && translation.get(0) <= rightBound) {
                    configuration = "B";
                    telemetry.addLine("Patter B");
                } else if (translation.get(0) >= rightBound) {
                    configuration = "C";
                    telemetry.addLine("Patter C");
                }

            } else {
                telemetry.addData("Visible Target", "none");
            }
            telemetry.update();
        }


    }


    // can be adapted later to collect skyStones from other group of three.
    private void collectionPath() {

        switch (configuration) {

            case "A":
                collectStones("left", "middle", "right");
                break;

            case "B":
                collectStones("middle", "left", "right");
                break;

            case "C":
                collectStones("right", "left", "middle");
                break;

        }

    }


    private void collectStones(String first, String second, String third) {

        switch (first) {

            case "left":
                // stuff to do
                break;
            case "middle":
                // stuff to do
                break;
            case "right":
                // stuff to do
                break;
        }

        switch (second) {

            case "left":
                // stuff to do
                break;
            case "middle":
                // stuff to do
                break;
            case "right":
                // stuff to do
                break;
        }

        switch (third) {

            case "left":
                // stuff to do
                break;
            case "middle":
                // stuff to do
                break;
            case "right":
                // stuff to do
                break;
        }


    }
}
