package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.util.ElapsedTime;
import java.lang.Math;

@Autonomous(name="R1x1.3", group="Pushbot")
// @Disabled
public class BasicAuto extends LinearOpMode {

    /* Declare OpMode members. */
    RobotHardware robot = new RobotHardware();
    private ElapsedTime     runtime = new ElapsedTime();

    static final double     COUNTS_PER_MOTOR_REV    = 537.6 ;
    static final double     DRIVE_GEAR_REDUCTION    = 1.0 ;     // This is < 1.0 if geared UP
    static final double     WHEEL_DIAMETER_INCHES   = 4.0 ;     // For figuring circumference
    static final double     COUNTS_PER_INCH         = (COUNTS_PER_MOTOR_REV * DRIVE_GEAR_REDUCTION) /
            (WHEEL_DIAMETER_INCHES * 3.1415);
    static final double     DRIVE_SPEED             = 0.01;
    static final double     TURN_SPEED              = 0.5;

    @Override
    public void runOpMode() {
        robot.init(hardwareMap);

        // Send telemetry message to signify robot waiting;
        telemetry.addData("Status", "Resetting Encoders");    //
        telemetry.update();

        robot.autoinit(hardwareMap); //[TODO] Test this
        robot.LFDrive.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        robot.RFDrive.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        robot.LBDrive.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        robot.RBDrive.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);

        robot.LFDrive.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        robot.RFDrive.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        robot.LBDrive.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        robot.RBDrive.setMode(DcMotor.RunMode.RUN_USING_ENCODER);

        robot.LFDrive.setDirection(DcMotor.Direction.FORWARD);
        robot.RFDrive.setDirection(DcMotor.Direction.REVERSE);
        robot.LBDrive.setDirection(DcMotor.Direction.FORWARD);
        robot.RBDrive.setDirection(DcMotor.Direction.REVERSE);
        // Send telemetry message to indicate successful Encoder reset

        telemetry.addData("Path0",  "Starting at %7d :%7d",
                robot.LFDrive.getCurrentPosition(),
                robot.RFDrive.getCurrentPosition(),
                robot.LBDrive.getCurrentPosition(),
                robot.RBDrive.getCurrentPosition());
        telemetry.update();

        // Wait for the game to start (driver presses PLAY)
        waitForStart();

        turn(TURN_SPEED, 45, 3);
        strafe(DRIVE_SPEED, 8, 3);
        sleep(1000);     // pause for servos to move

        telemetry.addData("Path", "Complete");
        telemetry.update();
    }

    public void forward(double speed, double inches, double timeout) {
        encoderDrive(DRIVE_SPEED, inches, inches, timeout);
    }
    public void turn(double speed, double angle, double timeout) { //[TODO] Test
        double realangle = angle - 30;
        if (Math.abs(90 - angle) <= Math.abs(270 - angle)) {
            robot.RFDrive.setDirection(DcMotor.Direction.REVERSE);
            robot.RBDrive.setDirection(DcMotor.Direction.REVERSE);
            robot.LFDrive.setDirection(DcMotor.Direction.FORWARD);
            robot.LBDrive.setDirection(DcMotor.Direction.FORWARD);
            encoderDrive(speed, realangle, -realangle, timeout);
        } else {
            robot.RFDrive.setDirection(DcMotor.Direction.FORWARD);
            robot.RBDrive.setDirection(DcMotor.Direction.FORWARD);
            robot.LFDrive.setDirection(DcMotor.Direction.REVERSE);
            robot.LBDrive.setDirection(DcMotor.Direction.REVERSE);
            encoderDrive(speed, -realangle, realangle, timeout);
            }

    }

    public void strafe(double speed, double inches, double timeout) {
        int LFTarget;
        int RFTarget;
        int LBTarget;
        int RBTarget;

        // Ensure that the opmode is still active
        if (opModeIsActive()) {
            robot.LFDrive.setDirection(DcMotor.Direction.REVERSE);
            robot.RFDrive.setDirection(DcMotor.Direction.FORWARD);
            robot.LBDrive.setDirection(DcMotor.Direction.FORWARD);
            robot.RBDrive.setDirection(DcMotor.Direction.REVERSE);

            // reset the timeout time and start motion.
            // runtime.reset();
            robot.LFDrive.setPower(Math.abs(speed));
            robot.RFDrive.setPower(Math.abs(speed));
            robot.LBDrive.setPower(Math.abs(speed));
            robot.RBDrive.setPower(Math.abs(speed));

            while (opModeIsActive() &&
                    (runtime.seconds() < timeout) &&
                    (robot.LFDrive.isBusy() && robot.RFDrive.isBusy() && robot.LBDrive.isBusy() && robot.RBDrive.isBusy())) {

                // Display it for the driver.
                telemetry.addData("Path2", "Running at %7d :%7d :%7d :%7d",
                        robot.LFDrive.getCurrentPosition(),
                        robot.RFDrive.getCurrentPosition(),
                        robot.LBDrive.getCurrentPosition(),
                        robot.RBDrive.getCurrentPosition());
                telemetry.update();
            }

            // Stop all motion;
            robot.LFDrive.setPower(0);
            robot.RFDrive.setPower(0);
            robot.LBDrive.setPower(0);
            robot.RBDrive.setPower(0);

            sleep(250);
        }

    }
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
            // runtime.reset();
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
                        robot.RFDrive.getCurrentPosition(),
                        robot.LBDrive.getCurrentPosition(),
                        robot.RBDrive.getCurrentPosition());
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
