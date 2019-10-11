
package org.firstinspires.ftc.teamcode.Programmers.Bryce;

import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.DigitalChannel;
import com.qualcomm.robotcore.hardware.DistanceSensor;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.util.ElapsedTime;


@TeleOp(name = "Basic: Linear OpMode", group = "Linear Opmode")
@Disabled
public class Linear_Opmode extends LinearOpMode {

    // Declare OpMode members.
    private ElapsedTime runtime = new ElapsedTime();
    private DcMotor leftDrive = null;
    private DcMotor rightDrive = null;
    private DcMotor drop = null;
    private Servo drop_pin = null;
    private DistanceSensor Range_Boy = null;
    private DigitalChannel Touchy_Boy = null;


    @Override
    public void runOpMode() {
        telemetry.addData("Status", "Initialized");
        telemetry.update();

        // Initialize the hardware variables. Note that the strings used here as parameters
        // to 'get' must correspond to the names assigned during the robot configuration
        // step (using the FTC Robot Controller app on the phone).
        leftDrive = hardwareMap.get(DcMotor.class, "left motor");
        rightDrive = hardwareMap.get(DcMotor.class, "right motor");
        //drop = hardwareMap.get(DcMotor.class, "drop");
        //drop_pin = hardwareMap.get(Servo.class, "drop_pin");
        //Range_Boy = hardwareMap.get(DistanceSensor.class, "Range_Boy");
        //Touchy_Boy = hardwareMap.get(DigitalChannel.class, "Touchy_Boy");
        //Touchy_Boy.setMode(DigitalChannel.Mode.INPUT);


        leftDrive.setDirection(DcMotor.Direction.REVERSE);
        rightDrive.setDirection(DcMotor.Direction.FORWARD);

        leftDrive.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        rightDrive.setMode(DcMotor.RunMode.RUN_USING_ENCODER);



        waitForStart();
        runtime.reset();

        Movement2(0.5, 200);


        while(opModeIsActive()){

            telemetry.addData("encoder: ", leftDrive.getCurrentPosition());
            telemetry.update();
        }



    }


    public void  Movement(double Power,double drive){

        leftDrive.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        rightDrive.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        while(opModeIsActive() && Math.abs(leftDrive.getCurrentPosition() - drive) >= 5 ) {


            leftDrive.setPower(.5);
            rightDrive.setPower(.5);
        }

        telemetry.addLine("done");
        telemetry.update();
        leftDrive.setPower(0);
        rightDrive.setPower(0);





    }

    public void  Movement2(double power, int drive) {
        leftDrive.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        rightDrive.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);

        leftDrive.setTargetPosition(drive);
        rightDrive.setTargetPosition(drive);

        leftDrive.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        rightDrive.setMode(DcMotor.RunMode.RUN_TO_POSITION);

        leftDrive.setPower(power);
        rightDrive.setPower(power);

        while(leftDrive.isBusy()||rightDrive.isBusy()){

            telemetry.addData("encodeer",leftDrive.getCurrentPosition());
            telemetry.update();
        }
        leftDrive.setPower(0);
        rightDrive.setPower(0);

        leftDrive.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        rightDrive.setMode(DcMotor.RunMode.RUN_USING_ENCODER);

    }





}
