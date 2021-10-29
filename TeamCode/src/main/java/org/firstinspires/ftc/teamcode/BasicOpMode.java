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
package org.firstinspires.ftc.teamcode;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.util.ElapsedTime;
import com.qualcomm.robotcore.util.Range;

@TeleOp(name="Basic: Linear OpMode.v1.4", group="Linear Opmode")
public class BasicOpMode extends LinearOpMode {
    // initialize telemetry
    private ElapsedTime runtime = new ElapsedTime();

    @Override
    public void runOpMode() {

            // initialize the hardware map
            RobotHardware robot = new RobotHardware();
            robot.init(hardwareMap);
            //Magic piece of code does something important
            robot.LFDrive.setDirection(DcMotor.Direction.FORWARD);
            robot.RFDrive.setDirection(DcMotor.Direction.REVERSE);
            robot.LBDrive.setDirection(DcMotor.Direction.FORWARD);
            robot.RBDrive.setDirection(DcMotor.Direction.REVERSE);

            // Wait for start
            waitForStart();
            runtime.reset();

            while (opModeIsActive()) {
                // power
                double leftPower;
                double rightPower;
                double strafePower;
                // multiplier for slow mode
                double multiplier = 1;
                // check for controller inputs
                double drive = -gamepad1.left_stick_y;
                double strafe = -gamepad1.left_stick_x;
                double turn  =  gamepad1.right_stick_x;
                // process inputs
                leftPower = Range.clip(drive + turn, -1.0, 1.0) ;
                strafePower = Range.clip(strafe, -1.0, 1.0);
                rightPower = Range.clip(drive - turn, -1.0, 1.0) ;
                if(gamepad1.right_bumper){
                    multiplier = 0.5;
                }
                // set power
                robot.LFDrive.setPower((leftPower - strafePower)*multiplier);
                robot.RFDrive.setPower((rightPower + strafePower)*multiplier);
                robot.LBDrive.setPower((leftPower + strafePower)*multiplier);
                robot.RBDrive.setPower((rightPower - strafePower)*multiplier);
                robot.Carousel.setPower(gamepad1.b ? 1 : 0); // this also may not work

                // Show the elapsed game time and wheel power. Telemetry
                telemetry.addData("Status", "Run Time: " + runtime.toString());
                telemetry.addData("Motors", "left (%.2f), right (%.2f), strafe (%.2f), carouselpower (%.2b)", leftPower, rightPower, strafePower, gamepad1.b);
                telemetry.update();
            }
        }
    }