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

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.robot.Robot;
import com.qualcomm.robotcore.util.ElapsedTime;


@Autonomous(name="Pushbot: Auto Drive By Encoder", group="Pushbot")
// @Disabled
public class BasicAuto extends LinearOpMode {

    /* Declare OpMode members. */
    RobotHardware robot = new RobotHardware();
    private ElapsedTime     runtime = new ElapsedTime();

    static final double     COUNTS_PER_MOTOR_REV    = 1440 ;    // eg: TETRIX Motor Encoder
    static final double     DRIVE_GEAR_REDUCTION    = 2.0 ;     // This is < 1.0 if geared UP
    static final double     WHEEL_DIAMETER_INCHES   = 4.0 ;     // For figuring circumference
    static final double     COUNTS_PER_INCH         = (COUNTS_PER_MOTOR_REV * DRIVE_GEAR_REDUCTION) /
                                                      (WHEEL_DIAMETER_INCHES * 3.1415);
    static final double     DRIVE_SPEED             = 0.6;
    static final double     TURN_SPEED              = 0.5;

    @Override
    public void runOpMode() {

        /*
         * Initialize the drive system variables.
         * The init() method of the hardware class does all the work here
         */

        // Send telemetry message to signify robot waiting;
        telemetry.addData("Status", "Resetting Encoders");    //
        telemetry.update();
        RobotHardware robot = new RobotHardware();
        robot.LFDrive.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        robot.RFDrive.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        robot.LBDrive.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        robot.RBDrive.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);

        robot.LFDrive.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        robot.RFDrive.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        robot.LBDrive.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        robot.RBDrive.setMode(DcMotor.RunMode.RUN_USING_ENCODER);

        // Send telemetry message to indicate successful Encoder reset
        telemetry.addData("Path0",  "Starting at %7d :%7d",
                robot.LFDrive.getCurrentPosition(),
                robot.RFDrive.getCurrentPosition(),
                robot.LBDrive.getCurrentPosition(),
                robot.RBDrive.getCurrentPosition());
        telemetry.update();

        // Wait for the game to start (driver presses PLAY)
        waitForStart();

        // Step through each leg of the path,
        // Note: Reverse movement is obtained by setting a negative distance (not speed)
        encoderDrive(DRIVE_SPEED,  48,  48, 5.0);  // S1: Forward 47 Inches with 5 Sec timeout
        encoderDrive(TURN_SPEED,   12, -12, 4.0);  // S2: Turn Right 12 Inches with 4 Sec timeout
        encoderDrive(DRIVE_SPEED, -24, -24, 4.0);  // S3: Reverse 24 Inches with 4 Sec timeout

        sleep(1000);     // pause for servos to move

        telemetry.addData("Path", "Complete");
        telemetry.update();
    }

    /*
     *  Method to perform a relative move, based on encoder counts.
     *  Encoders are not reset as the move is based on the current position.
     *  Move will stop if any of three conditions occur:
     *  1) Move gets to the desired position
     *  2) Move runs out of time
     *  3) Driver stops the opmode running.
     */
    public void encoderDrive(double speed,
                             double leftInches, double rightInches,
                             double timeoutS) {
        int LFTarget;
        int RFTarget;
        int LBTarget;
        int RBTarget;

        // Ensure that the opmode is still active
        if (opModeIsActive()) {

            // Determine new target position, and pass to motor controller
            LFTarget = robot.LFDrive.getCurrentPosition() + (int)(leftInches * COUNTS_PER_INCH);
            RFTarget = robot.RFDrive.getCurrentPosition() + (int)(rightInches * COUNTS_PER_INCH);
            LBTarget = robot.LBDrive.getCurrentPosition() + (int)(leftInches * COUNTS_PER_INCH);
            RBTarget = robot.RBDrive.getCurrentPosition() + (int)(rightInches * COUNTS_PER_INCH);

            robot.LFDrive.setTargetPosition(LFTarget);
            robot.RFDrive.setTargetPosition(RFTarget);
            robot.LBDrive.setTargetPosition(LBTarget);
            robot.RBDrive.setTargetPosition(RBTarget);

            // Turn On RUN_TO_POSITION
            robot.LFDrive.setMode(DcMotor.RunMode.RUN_TO_POSITION);
            robot.RFDrive.setMode(DcMotor.RunMode.RUN_TO_POSITION);
            robot.LBDrive.setMode(DcMotor.RunMode.RUN_TO_POSITION);
            robot.RBDrive.setMode(DcMotor.RunMode.RUN_TO_POSITION);

            // reset the timeout time and start motion.
            runtime.reset();
            robot.LFDrive.setPower(Math.abs(speed));
            robot.RFDrive.setPower(Math.abs(speed));
            robot.LBDrive.setPower(Math.abs(speed));
            robot.RBDrive.setPower(Math.abs(speed));

            // keep looping while we are still active, and there is time left, and both motors are running.
            // Note: We use (isBusy() && isBusy()) in the loop test, which means that when EITHER motor hits
            // its target position, the motion will stop.  This is "safer" in the event that the robot will
            // always end the motion as soon as possible.
            // However, if you require that BOTH motors have finished their moves before the robot continues
            // onto the next step, use (isBusy() || isBusy()) in the loop test.
            while (opModeIsActive() &&
                   (runtime.seconds() < timeoutS) &&
                   (robot.LFDrive.isBusy() && robot.RFDrive.isBusy() && robot.LBDrive.isBusy() && robot.RBDrive.isBusy())) {

                // Display it for the driver.
                telemetry.addData("Path1",  "Running to %7d :%7d", LFTarget,  RFTarget, LBTarget, RBTarget);
                telemetry.addData("Path2",  "Running at %7d :%7d",
                                            robot.LFDrive.getCurrentPosition(),
                                            robot.RFDrive.getCurrentPosition());
                telemetry.update();
            }

            // Stop all motion;
            robot.LFDrive.setPower(0);
            robot.RFDrive.setPower(0);
            robot.LBDrive.setPower(0);
            robot.RBDrive.setPower(0);

            // Turn off RUN_TO_POSITION
            robot.LFDrive.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
            robot.RFDrive.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
            robot.LBDrive.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
            robot.RBDrive.setMode(DcMotor.RunMode.RUN_USING_ENCODER);

            //  sleep(250);   // optional pause after each move
        }
    }
}
