package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.util.ElapsedTime;
import java.lang.Math;

@Autonomous(name="TestAuto", group="Testing")
public class TestAuto extends LinearOpMode {

    /* Declare OpMode members. */
    RobotHardware robot = new RobotHardware();
    private final ElapsedTime runtime = new ElapsedTime();

    static final double COUNTS_PER_MOTOR_REV = 537.6;
    static final double DRIVE_GEAR_REDUCTION = 1.0;     // This is < 1.0 if geared UP
    static final double WHEEL_DIAMETER_INCHES = 4.0;     // For figuring circumference
    static final double COUNTS_PER_INCH = (COUNTS_PER_MOTOR_REV * DRIVE_GEAR_REDUCTION) /
            (WHEEL_DIAMETER_INCHES * Math.PI);

    static final double COUNTS_PER_MOTOR_REV2 = 103.8; // 28 PPR at encoder shaft, 103.8 PPR at gearbox output shaft
    static final double DRIVE_GEAR_REDUCTION2 = 2.0;     // This is < 1.0 if geared UP
    static final double WHEEL_DIAMETER_INCHES2 = 0.8425;     // For figuring circumference
    static final double COUNTS_PER_INCH2 = (COUNTS_PER_MOTOR_REV2 * DRIVE_GEAR_REDUCTION2) /
            (WHEEL_DIAMETER_INCHES2 * Math.PI);

    static final double DRIVE_SPEED = 0.75;
    static final double PRECISION_DRIVE_SPEED = 0.5;
    static final double TURN_SPEED = 0.5;

    @Override
    public void runOpMode() {
        robot.init(hardwareMap);

        // Send telemetry message to signify robot waiting;
        telemetry.addData("Status", "Resetting Encoders");
        telemetry.update();

        // Send telemetry message to indicate successful Encoder reset
        telemetry.addData("Path0", "Starting at %7d :%7d",
                robot.LFDrive.getCurrentPosition(),
                robot.RFDrive.getCurrentPosition(),
                robot.LBDrive.getCurrentPosition(),
                robot.RBDrive.getCurrentPosition());
        telemetry.update();
        // wait for Play
        waitForStart();

        //-------------------------------------------------//
        /*sleep(1000);
        robot.ClawSide.setPosition(0.3);
        sleep(1000);
        runSlides(robot, 1, (int)(-30.0 * COUNTS_PER_INCH2), 2);
        sleep(1000);
        robot.Intake.setPower(-0.5);
        sleep(3000);
        robot.Intake.setPower(0);
        sleep(1000);
        runSlides(robot, 0.5, (int)(0.0 * COUNTS_PER_INCH2), 2);
        sleep(1000);
        robot.ClawSide.setPosition(0.8);*/

        sleep(1000);
        forward(1, 30);
        // strafe(1, 10, 2);
        //-------------------------------------------------//

        telemetry.addData("Path", "Complete");
        telemetry.update();
    }

    public void forward(double speed, double timeout) {
        speed = -speed;

        if (opModeIsActive()) {
            runtime.reset();
            robot.LFDrive.setPower(speed);
            robot.RFDrive.setPower(speed);
            robot.LBDrive.setPower(speed);
            robot.RBDrive.setPower(speed);

            while (opModeIsActive() &&
                    (runtime.seconds() < timeout)) {
                telemetry.update();
            }

            robot.LFDrive.setPower(0);
            robot.RFDrive.setPower(0);
            robot.LBDrive.setPower(0);
            robot.RBDrive.setPower(0);
        }
    }

    public void turn(double speed, double angle, double timeout) { //Angles are not accurate
        double realangle = (136 * Math.PI) * (angle / 360);
        if (Math.abs(90 - angle) <= Math.abs(270 - angle)) {
            encoderDrive(speed, realangle, -realangle, timeout);
        } else {
            encoderDrive(speed, -realangle, realangle, timeout);
        }

    }

    public void duck(double timeoutS) {

        double spin = 0;

        if (opModeIsActive()) {
            while (opModeIsActive() && runtime.seconds() < timeoutS) {
                spin += 0.0025;
                robot.Carousel.setPower(spin);
                telemetry.addData("Spinning speed", spin);
                telemetry.update();
            }

            robot.Carousel.setPower(0);
        }
    }

    public void strafe(double speed, double timeout) {

        /*int LFTarget;
        int RFTarget;
        int LBTarget;
        int RBTarget;*/

        // Ensure that the opmode is still active
        if (opModeIsActive()) {

            /*LFTarget = robot.LFDrive.getCurrentPosition() - (int) (inches * COUNTS_PER_INCH);
            RFTarget = robot.RFDrive.getCurrentPosition() + (int) (inches * COUNTS_PER_INCH);
            LBTarget = robot.LBDrive.getCurrentPosition() + (int) (inches * COUNTS_PER_INCH);
            RBTarget = robot.RBDrive.getCurrentPosition() - (int) (inches * COUNTS_PER_INCH);

            robot.LFDrive.setTargetPosition(LFTarget);
            robot.RFDrive.setTargetPosition(RFTarget);
            robot.LBDrive.setTargetPosition(LBTarget);
            robot.RBDrive.setTargetPosition(RBTarget);*/

            // reset the timeout time and start motion.
            runtime.reset();
            robot.LFDrive.setPower(speed);
            robot.RFDrive.setPower(-speed);
            robot.LBDrive.setPower(-speed);
            robot.RBDrive.setPower(speed);

            // Turn On RUN_TO_POSITION
            /*robot.LFDrive.setMode(DcMotor.RunMode.RUN_TO_POSITION);
            robot.RFDrive.setMode(DcMotor.RunMode.RUN_TO_POSITION);
            robot.LBDrive.setMode(DcMotor.RunMode.RUN_TO_POSITION);
            robot.RBDrive.setMode(DcMotor.RunMode.RUN_TO_POSITION);*/

            while (opModeIsActive() &&
                    (runtime.seconds() < timeout)) {
                telemetry.update();
            }

            robot.LFDrive.setPower(0);
            robot.RFDrive.setPower(0);
            robot.LBDrive.setPower(0);
            robot.RBDrive.setPower(0);

        }
    }

    public void encoderDrive ( double speed, double leftInches, double rightInches, double timeoutS){

        int LFTarget;
        int RFTarget;
        int LBTarget;
        int RBTarget;

        // Ensure that the opmode is still active
        if (opModeIsActive()) {

            // Determine new target position, and pass to motor controller
            LFTarget = robot.LFDrive.getCurrentPosition() + (int) (leftInches * COUNTS_PER_INCH);
            RFTarget = robot.RFDrive.getCurrentPosition() + (int) (rightInches * COUNTS_PER_INCH);
            LBTarget = robot.LBDrive.getCurrentPosition() + (int) (leftInches * COUNTS_PER_INCH);
            RBTarget = robot.RBDrive.getCurrentPosition() + (int) (rightInches * COUNTS_PER_INCH);

            robot.LFDrive.setTargetPosition(LFTarget);
            robot.RFDrive.setTargetPosition(RFTarget);
            robot.LBDrive.setTargetPosition(LBTarget);
            robot.RBDrive.setTargetPosition(RBTarget);


            // reset the timeout time and start motion.
            runtime.reset();
            robot.LFDrive.setPower(Math.abs(speed));
            robot.RFDrive.setPower(Math.abs(speed));
            robot.LBDrive.setPower(Math.abs(speed));
            robot.RBDrive.setPower(Math.abs(speed));

            // Turn On RUN_TO_POSITION
            robot.LFDrive.setMode(DcMotor.RunMode.RUN_TO_POSITION);
            robot.RFDrive.setMode(DcMotor.RunMode.RUN_TO_POSITION);
            robot.LBDrive.setMode(DcMotor.RunMode.RUN_TO_POSITION);
            robot.RBDrive.setMode(DcMotor.RunMode.RUN_TO_POSITION);

            while (opModeIsActive() &&
                    (runtime.seconds() < timeoutS) &&
                    (robot.LBDrive.isBusy() || robot.RBDrive.isBusy() || robot.RFDrive.isBusy() || robot.LFDrive.isBusy())) {

                // Display it for the driver.
                telemetry.addData("Time", timeoutS);
                telemetry.addData("Front Left ", robot.LFDrive.getCurrentPosition());
                telemetry.addData("TARGET:", robot.LFDrive.getTargetPosition());
                telemetry.addData("Front Right ", robot.RFDrive.getCurrentPosition());
                telemetry.addData("TARGET:", robot.RFDrive.getTargetPosition());
                telemetry.addData("Back Left ", robot.LBDrive.getCurrentPosition());
                telemetry.addData("TARGET:", robot.LBDrive.getTargetPosition());
                telemetry.addData("Back Right ", robot.RBDrive.getCurrentPosition());
                telemetry.addData("TARGET:", robot.RBDrive.getTargetPosition());
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

    public void runSlides(RobotHardware robot, double speed, int slideTarget, double timeoutS) {

        // Ensure that the opmode is still active
        if (opModeIsActive()) {

            robot.Slide.setTargetPosition(slideTarget);
            runtime.reset();
            robot.Slide.setPower(Math.abs(speed));
            robot.Slide.setMode(DcMotor.RunMode.RUN_TO_POSITION);

            while (opModeIsActive() &&
                    (runtime.seconds() < timeoutS) &&
                    (Math.abs(slideTarget - robot.Slide.getCurrentPosition()) > 10)) {

                // Display it for the driver.
                telemetry.addData("Path1", "Running to %7d", slideTarget);
                telemetry.addData("Path2", "Running at %7d", robot.Slide.getCurrentPosition());
                telemetry.update();
            }

            robot.Slide.setPower(0);
        }
    }
}