/*
Copyright (c) 2018 FIRST

All rights reserved.

Redistribution and use in source and binary forms, with or without modification,
are permitted (subject to the limitations in the disclaimer below) provided that
the following conditions are met:

Redistributions of source code must retain the above copyright notice, this list
of conditions and the following disclaimer.

Redistributions in binary form must reproduce the above copyright notice, this
list of conditions and the following disclaimer in the documentation and/or
other materials provided with the distribution.

Neither the name of FIRST nor the names of its contributors may be used to
endorse or promote products derived from this software without specific prior
written permission.

NO EXPRESS OR IMPLIED LICENSES TO ANY PARTY'S PATENT RIGHTS ARE GRANTED BY THIS
LICENSE. THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS
"AS IS" AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO,
THE IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE
ARE DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT OWNER OR CONTRIBUTORS BE LIABLE
FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL
DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR
SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER
CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY, OR
TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE OF
THIS SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.
*/
package org.firstinspires.ftc.teamcode.Team_Resources;

import com.qualcomm.hardware.rev.Rev2mDistanceSensor;
import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DistanceSensor;

import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit;


@TeleOp(name = "distance sensor example", group = "Sensor")
@Disabled

public class REV_Distance_Sensor extends LinearOpMode {

    //making the variable to stone the distance sensor. You can name it what ever you want
    private DistanceSensor name;

    @Override
    public void runOpMode() {
        // Finding out what the sensor is called in on the phone. Also where it is plugged in on the REV hub.
        name = hardwareMap.get(DistanceSensor.class, "sensor_range");

        // You have to write this to access the values of the sensor
        Rev2mDistanceSensor sensorTimeOfFlight = (Rev2mDistanceSensor)name;

        // put words on the phone to tell the user what is going to happen
        telemetry.addData(">>", "Press start to continue");
        telemetry.update();

        //wait for the start button to be pressed
        waitForStart();

        // loop to continue putting the values on the phone as long as the program is running.
        while(opModeIsActive()) {

            // Getting all sorts of different information.
            telemetry.addData("deviceName",name.getDeviceName() );     // what the sensor is

            // telling distance in mm, cm, meters, and inches
            telemetry.addData("range", String.format("%.01f mm", name.getDistance(DistanceUnit.MM)));
            telemetry.addData("range", String.format("%.01f cm", name.getDistance(DistanceUnit.CM)));
            telemetry.addData("range", String.format("%.01f m", name.getDistance(DistanceUnit.METER)));
            telemetry.addData("range", String.format("%.01f in", name.getDistance(DistanceUnit.INCH)));

            // Special information that you can get from the sensor, you usually wont use these.
            telemetry.addData("ID", String.format("%x", sensorTimeOfFlight.getModelID()));
            telemetry.addData("did time out", Boolean.toString(sensorTimeOfFlight.didTimeoutOccur()));

            telemetry.update();
        }
    }

}