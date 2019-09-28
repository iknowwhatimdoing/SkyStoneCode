package org.firstinspires.ftc.teamcode.SkyStone;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.vuforia.HINT;
import com.vuforia.Vuforia;

import org.firstinspires.ftc.robotcore.external.ClassFactory;
import org.firstinspires.ftc.robotcore.external.matrices.OpenGLMatrix;
import org.firstinspires.ftc.robotcore.external.matrices.VectorF;
import org.firstinspires.ftc.robotcore.external.navigation.VuforiaLocalizer;
import org.firstinspires.ftc.robotcore.external.navigation.VuforiaTrackable;
import org.firstinspires.ftc.robotcore.external.navigation.VuforiaTrackableDefaultListener;
import org.firstinspires.ftc.robotcore.external.navigation.VuforiaTrackables;
import org.firstinspires.ftc.teamcode.R;
import org.opencv.core.Mat;


@Autonomous (name = "Pls work")
public class vuforiaMultipleObjects extends LinearOpMode {

    private VuforiaLocalizer vuforia = null;


    @Override
    public void runOpMode() throws InterruptedException {
        VuforiaLocalizer.Parameters params = new VuforiaLocalizer.Parameters(R.id.cameraMonitorViewId);
        params.cameraDirection = VuforiaLocalizer.CameraDirection.BACK;
        params.vuforiaLicenseKey = "AeVN5hL/////AAABmeiiuuNfCUG7o7nFHIB8wOIJd0HGVkf4o9TQNISpW19DjyAqW6a1+DIXgQEWJlZoXbHJhjRNbpdhsZiyF8YZS7mSkspYXxB9vhRl3NdBzba6lb450R331LCujbFW4f1z5ZiERvZsxUn1bl8FeugGjQAag3tO7DU7ZNFMRVev0pTtIo0tIRtVlW1zmZqCAQCmCLtAUPjpUv2atadLfVqleYVydV3AN2upMHu14tb3zBuOmM2SfHu//Nuo8xh/U2a0Joi9B286UYurYN7S/+2mzpe6cFcqdDjVV7C3sjvHUw3ivtGy9M7BXwsVZkF/aSQr0cjDfTv/si4DW8lm/WLG5thfi2kHv/B21I2C67dt/oGI";

        vuforia = ClassFactory.getInstance().createVuforia(params);
        Vuforia.setHint(HINT.HINT_MAX_SIMULTANEOUS_IMAGE_TARGETS,3);

        VuforiaTrackables things = vuforia.loadTrackablesFromAsset("Skystone");
        things.get(0).setName("Stone");

        waitForStart();

        things.activate();

        while(opModeIsActive()){

            for (VuforiaTrackable thing: things){
                OpenGLMatrix position = ((VuforiaTrackableDefaultListener) thing.getListener()).getPose();

                if (position != null){
                    VectorF translation = position.getTranslation();

                    telemetry.addData(thing.getName() + " -Translation",translation);

                    double degreesToTurn = Math.toDegrees(Math.atan2(translation.get(0),translation.get(2)));

                    telemetry.addData(thing.getName() + "-Degrees",degreesToTurn);





                }
            }
            telemetry.update();

        }

    }
}
