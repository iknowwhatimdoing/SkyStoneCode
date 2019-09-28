/* Copyright (c) 2019 FIRST. All rights reserved.
 *
 * Redistribution and use in source and binary forms, with or without modification,
 * are permitted (subject to the limitations in the disclaimer below) provided that
 * the following conditions are met:
 *
 * Redistributions of source code must retain the above copyright notice, this list
 * of conditions and the following disclaimer.
 *
 * Redistributions in binary form must reproduce the above copyright notice, this
 * list of conditions and the following disclaimer in the documentation and/or
 * other materials provided with the distribution.
 *
 * Neither the name of FIRST nor the names of its contributors may be used to endorse or
 * promote products derived from this software without specific prior written permission.
 *
 * NO EXPRESS OR IMPLIED LICENSES TO ANY PARTY'S PATENT RIGHTS ARE GRANTED BY THIS
 * LICENSE. THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS
 * "AS IS" AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO,
 * THE IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE
 * ARE DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT OWNER OR CONTRIBUTORS BE LIABLE
 * FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL
 * DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR
 * SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER
 * CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY,
 * OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE
 * OF THIS SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.
 */

package org.firstinspires.ftc.teamcode.SkyStone;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.vuforia.CameraDevice;
import com.vuforia.Vuforia;

import org.firstinspires.ftc.robotcore.external.ClassFactory;
import org.firstinspires.ftc.robotcore.external.matrices.OpenGLMatrix;
import org.firstinspires.ftc.robotcore.external.matrices.VectorF;
import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.robotcore.external.navigation.AxesOrder;
import org.firstinspires.ftc.robotcore.external.navigation.AxesReference;
import org.firstinspires.ftc.robotcore.external.navigation.Orientation;
import org.firstinspires.ftc.robotcore.external.navigation.VuforiaLocalizer;
import org.firstinspires.ftc.robotcore.external.navigation.VuforiaTrackable;
import org.firstinspires.ftc.robotcore.external.navigation.VuforiaTrackableDefaultListener;
import org.firstinspires.ftc.robotcore.external.navigation.VuforiaTrackables;
import org.firstinspires.ftc.teamcode.R;

import java.util.ArrayList;
import java.util.List;

import static com.vuforia.HINT.HINT_MAX_SIMULTANEOUS_IMAGE_TARGETS;
import static org.firstinspires.ftc.robotcore.external.navigation.AngleUnit.DEGREES;
import static org.firstinspires.ftc.robotcore.external.navigation.AxesOrder.XYZ;
import static org.firstinspires.ftc.robotcore.external.navigation.AxesOrder.YZX;
import static org.firstinspires.ftc.robotcore.external.navigation.AxesReference.EXTRINSIC;
import static org.firstinspires.ftc.robotcore.external.navigation.VuforiaLocalizer.CameraDirection.BACK;


@TeleOp(name="Vuforia Scan test", group ="Concept")
//@Disabled
public class VuforiaScantest extends LinearOpMode {







        // Variables to be used for later

        private VuforiaLocalizer vuforiaLocalizer;

        private VuforiaLocalizer.Parameters parameters;

        private VuforiaTrackables visionTargets;

        private VuforiaTrackable target;

        private VuforiaTrackableDefaultListener listener;



        private OpenGLMatrix lastKnownLocation;

        private OpenGLMatrix phoneLocation;



        private static final String VUFORIA_KEY =             "AeVN5hL/////AAABmeiiuuNfCUG7o7nFHIB8wOIJd0HGVkf4o9TQNISpW19DjyAqW6a1+DIXgQEWJlZoXbHJhjRNbpdhsZiyF8YZS7mSkspYXxB9vhRl3NdBzba6lb450R331LCujbFW4f1z5ZiERvZsxUn1bl8FeugGjQAag3tO7DU7ZNFMRVev0pTtIo0tIRtVlW1zmZqCAQCmCLtAUPjpUv2atadLfVqleYVydV3AN2upMHu14tb3zBuOmM2SfHu//Nuo8xh/U2a0Joi9B286UYurYN7S/+2mzpe6cFcqdDjVV7C3sjvHUw3ivtGy9M7BXwsVZkF/aSQr0cjDfTv/si4DW8lm/WLG5thfi2kHv/B21I2C67dt/oGI";
    // Insert your own key here



        private float robotX = 0;

        private float robotY = 0;

        private float robotAngle = 0;



        public void runOpMode() throws InterruptedException

        {

            setupVuforia();



            // We don't know where the robot is, so set it to the origin

            // If we don't include this, it would be null, which would cause errors later on

            lastKnownLocation = createMatrix(0, 0, 0, 0, 0, 0);



            waitForStart();



            // Start tracking the targets

            visionTargets.activate();



            while(opModeIsActive())

            {

                // Ask the listener for the latest information on where the robot is

                OpenGLMatrix latestLocation = listener.getUpdatedRobotLocation();



                // The listener will sometimes return null, so we check for that to prevent errors

                if(latestLocation != null)

                    lastKnownLocation = latestLocation;



                float[] coordinates = lastKnownLocation.getTranslation().getData();



                robotX = coordinates[0];

                robotY = coordinates[1];

                robotAngle = Orientation.getOrientation(lastKnownLocation, AxesReference.EXTRINSIC, AxesOrder.XYZ, AngleUnit.DEGREES).thirdAngle;



                // Send information about whether the target is visible, and where the robot is

                telemetry.addData("Tracking " + target.getName(), listener.isVisible());

                telemetry.addData("Last Known Location", formatMatrix(lastKnownLocation));



                // Send telemetry and idle to let hardware catch up

                telemetry.update();

                idle();

            }

        }



        private void setupVuforia()

        {

            // Setup parameters to create localizer

            parameters = new VuforiaLocalizer.Parameters(R.id.cameraMonitorViewId); // To remove the camera view from the screen, remove the R.id.cameraMonitorViewId

            parameters.vuforiaLicenseKey = VUFORIA_KEY;

            parameters.cameraDirection = VuforiaLocalizer.CameraDirection.BACK;

            parameters.useExtendedTracking = false;

            vuforiaLocalizer = ClassFactory.createVuforiaLocalizer(parameters);



            // These are the vision targets that we want to use

            // The string needs to be the name of the appropriate .xml file in the assets folder

            visionTargets = vuforiaLocalizer.loadTrackablesFromAsset("Skystone");

            Vuforia.setHint(HINT_MAX_SIMULTANEOUS_IMAGE_TARGETS, 4);



            // Setup the target to be tracked

            target = visionTargets.get(0); // 0 corresponds to the wheels target

            target.setName("Stone");

            target.setLocation(createMatrix(0, 500, 0, 90, 0, 90));



            // Set phone location on robot

            phoneLocation = createMatrix(0, 225, 0, 90, 0, 0);



            // Setup listener and inform it of phone information

            listener = (VuforiaTrackableDefaultListener) target.getListener();

            listener.setPhoneInformation(phoneLocation, parameters.cameraDirection);

        }



        // Creates a matrix for determining the locations and orientations of objects

        // Units are millimeters for x, y, and z, and degrees for u, v, and w

        private OpenGLMatrix createMatrix(float x, float y, float z, float u, float v, float w)

        {

            return OpenGLMatrix.translation(x, y, z).

                    multiplied(Orientation.getRotationMatrix(

                            AxesReference.EXTRINSIC, AxesOrder.XYZ, AngleUnit.DEGREES, u, v, w));

        }



        // Formats a matrix into a readable string

        private String formatMatrix(OpenGLMatrix matrix)

        {

            return matrix.formatAsTransform();

        }




}
