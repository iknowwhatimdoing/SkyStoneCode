/* Copyright (c) 2017 FIRST. All rights reserved.
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

package org.firstinspires.ftc.teamcode.Programmers.Drew;

import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.CRServo;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.util.ElapsedTime;
import com.qualcomm.robotcore.util.Range;

@TeleOp(name = "Basic: Iterative OpMode", group = "Iterative Opmode")
@Disabled
public class BasicOpMode_Iterative extends OpMode {


    DcMotor left;
    DcMotor right;
    Servo spaghettinator;
    CRServo pizzaspinner;


    @Override
    public void init() {

        left = hardwareMap.get(DcMotor.class, "yellow");
        right = hardwareMap.get(DcMotor.class, "orange");
        right.setDirection(DcMotorSimple.Direction.REVERSE);

        left.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        right.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);

        spaghettinator = hardwareMap.get(Servo.class, "yes");
        pizzaspinner = hardwareMap.get(CRServo.class, "no");


    }


    @Override
    public void start() {
        left.setPower(0);
        right.setPower(0);

        spaghettinator.setPosition(0);
        pizzaspinner.setPower(0);

        left.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        right.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
    }

    @Override
    public void loop() {
        double drive = gamepad1.left_stick_y;
        double turn = gamepad1.right_stick_x;

        left.setPower(drive + turn);
        right.setPower(drive - turn);

        if (gamepad2.y) {
            spaghettinator.setPosition(1);

        } else if (gamepad2.x) {
            spaghettinator.setPosition(0);
        }

        if (gamepad2.left_trigger > 0) {
            pizzaspinner.setPower(gamepad2.left_trigger);
        } else if (gamepad2.right_trigger > 0) {
            pizzaspinner.setPower(-gamepad2.right_trigger);

        }

        if (gamepad1.right_bumper) {
            left.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
            right.setMode(DcMotor.RunMode.RUN_USING_ENCODER);

            left.setTargetPosition(7);
            right.setTargetPosition(3000000);

            left.setMode(DcMotor.RunMode.RUN_TO_POSITION);
            right.setMode(DcMotor.RunMode.RUN_TO_POSITION);

            left.setPower(.32);
            right.setPower(.8889);

            if (!left.isBusy() || !right.isBusy()) {
                left.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
                right.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
            }
        }
    }


}
