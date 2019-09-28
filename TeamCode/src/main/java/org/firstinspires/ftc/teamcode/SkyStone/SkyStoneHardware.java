package org.firstinspires.ftc.teamcode.SkyStone;


import com.qualcomm.hardware.modernrobotics.ModernRoboticsI2cGyro;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.IntegratingGyroscope;


public class SkyStoneHardware {

    /* Public OpMode members. */
    public DcMotor leftfront   = null;
    public DcMotor  rightfront  = null;
    public DcMotor leftback   = null;
    public DcMotor  rightback  = null;


    public IntegratingGyroscope gyro = null;
    public ModernRoboticsI2cGyro modernRoboticsI2cGyro = null;



    /* local OpMode members. */
    HardwareMap hwMap =  null;

    /* Constructor */
    public SkyStoneHardware(){

    }

    /* Initialize standard Hardware interfaces */
    public void init(HardwareMap ahwMap) {
        // Save reference to Hardware map
        hwMap = ahwMap;


        modernRoboticsI2cGyro = hwMap.get(ModernRoboticsI2cGyro.class, "gyro");
        gyro = (IntegratingGyroscope)modernRoboticsI2cGyro;

        // Define and Initialize Motors
        leftfront  = hwMap.get(DcMotor.class, "lf");
        leftback = hwMap.get(DcMotor.class, "lb");
        rightfront    = hwMap.get(DcMotor.class, "rf");
        rightback = hwMap.get(DcMotor.class, "rb");

        rightback.setDirection(DcMotor.Direction.REVERSE); // Set to REVERSE if using AndyMark motors
        rightfront.setDirection(DcMotor.Direction.REVERSE);// Set to FORWARD if using AndyMark motors

        // Set all motors to zero power
        leftfront.setPower(0);
        leftback.setPower(0);
        rightfront.setPower(0);
        rightback.setPower(0);


        leftfront.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        leftback.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        rightfront.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        rightback.setMode(DcMotor.RunMode.RUN_USING_ENCODER);


    }


    public  void driveEach(double lf, double lb, double rf, double rb){

        leftfront.setPower(lf);
        leftback.setPower(lb);
        rightfront.setPower(rf);
        rightback.setPower(rb);
    }

    public void driveAll(double power){
        leftfront.setPower(power);
        leftback.setPower(power);
        rightfront.setPower(power);
        rightback.setPower(power);
    }
}

