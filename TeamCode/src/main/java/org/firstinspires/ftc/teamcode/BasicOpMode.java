package org.firstinspires.ftc.teamcode;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.util.ElapsedTime;
import java.lang.Math;

@Autonomous(name="R1", group="Red")
public class R1 extends LinearOpMode {

    /* Declare OpMode members. */
    RobotHardware robot = new RobotHardware();
    private final ElapsedTime runtime = new ElapsedTime();

    static final double COUNTS_PER_MOTOR_REV = 537.6;
    static final double DRIVE_GEAR_REDUCTION = 1.0;     // This is < 1.0 if geared UP
    static final double WHEEL_DIAMETER_INCHES = 4.0;     // For figuring circumference
    static final double COUNTS_PER_INCH = (COUNTS_PER_MOTOR_REV * DRIVE_GEAR_REDUCTION) /
            (WHEEL_DIAMETER_INCHES * Math.PI);
    static final double DRIVE_SPEED = 0.75;
    static final double PRECISION_DRIVE_SPEED = 0.05;
    static final double TURN_SPEED = 0.5;

    @Override
    public void runOpMode() {
        robot.init(hardwareMap);

        // Send telemetry message to signify robot waiting;
        telemetry.addData("Status", "Resetting Encoders");
        telemetry.update();

        robot.autoinit(hardwareMap);

//        // Send telemetry message to indicate successful Encoder reset
        telemetry.addData("Path0", "Starting at %7d :%7d",
                robot.LFDrive.getCurrentPosition(),
                robot.RFDrive.getCurrentPosition(),
                robot.LBDrive.getCurrentPosition(),
                robot.RBDrive.getCurrentPosition());
        telemetry.update();
        // wait for Play
        waitForStart();

        //-------------------------------------------------//
        sleep(1000);
        strafe(DRIVE_SPEED, 23, 6.5);
        sleep(5000);
        robot.Carousel.setPower(0);
        // duck(5);
        //-------------------------------------------------//

        telemetry.addData("Path", "Complete");
        telemetry.update();
    }

    public void forward(double speed, double inches, double timeout) {
        encoderDrive(speed, -inches, -inches, timeout);
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

    public void strafe(double speed, double inches, double timeoutS) {

        int LFTarget;
        int RFTarget;
        int LBTarget;
        int RBTarget;

        // Ensure that the opmode is still active
        if (opModeIsActive()) {

            LFTarget = robot.LFDrive.getCurrentPosition() - (int) (inches * COUNTS_PER_INCH);
            RFTarget = robot.RFDrive.getCurrentPosition() + (int) (inches * COUNTS_PER_INCH);
            LBTarget = robot.LBDrive.getCurrentPosition() + (int) (inches * COUNTS_PER_INCH);
            RBTarget = robot.RBDrive.getCurrentPosition() - (int) (inches * COUNTS_PER_INCH);

            robot.LFDrive.setTargetPosition(LFTarget);
            robot.RFDrive.setTargetPosition(RFTarget);
            robot.LBDrive.setTargetPosition(LBTarget);
            robot.RBDrive.setTargetPosition(RBTarget);

            // reset the timeout time and start motion.
            runtime.reset();
            double leftPower;
            double rightPower;
            double strafePower;
            double slidePower = 0;
            double carouselPower = 0.1;
            double servoCenterAngle = 1;
            double servoSideAngle = 1;

            while (opModeIsActive()) {

                // multiplier for slow mode
                double multiplier = 1;

                // check for controller inputs
                double drive = gamepad1.left_stick_y;
                double strafe = gamepad1.left_stick_x;
                double claw = gamepad1.right_stick_y;
                double turn  =  -gamepad1.right_stick_x;

                // process inputs
                servoSideAngle = Range.clip(claw, -0.5, 0.5) + 0.5;
                if (Math.abs(0.5 - servoSideAngle) > 0.25) {
                    turn = 0; // Ignore turning if moving claw up or down
                }

                leftPower = Range.clip(drive + turn, -1.0, 1.0);
                strafePower = Range.clip(strafe, -1.0, 1.0);
                rightPower = Range.clip(drive - turn, -1.0, 1.0);
                if (gamepad1.right_bumper) multiplier = 0.5;
                // slowly increase motor speed to not send ducks flying
                if (gamepad1.b || gamepad1.y) {
                    carouselPower += 0.0025;
                }
                else {
                    carouselPower = 0;
                }

                if (gamepad1.a && servoCenterAngle < 1) {
                    servoCenterAngle += 0.0005;
                }
                if (gamepad1.x && servoCenterAngle > 0) {
                    servoCenterAngle -= 0.0005;
                }

                if (gamepad1.dpad_up) {
                    slidePower = 0.25;
                }
                if (gamepad1.dpad_down) {
                    slidePower = -0.25;
                }

                // set power
                robot.LFDrive.setPower((leftPower - strafePower)*multiplier);
                robot.RFDrive.setPower((rightPower + strafePower)*multiplier);
                robot.LBDrive.setPower((leftPower + strafePower)*multiplier);
                robot.RBDrive.setPower((rightPower - strafePower)*multiplier);
                if(gamepad1.b) {
                    robot.Carousel.setDirection(DcMotor.Direction.FORWARD);
                }

                public void strafe(double speed, double inches, double timeoutS) {

                    int LFTarget;
                    int RFTarget;
                    int LBTarget;
                    int RBTarget;

                    // Ensure that the opmode is still active
                    if (opModeIsActive()) {

                        LFTarget = robot.LFDrive.getCurrentPosition() - (int) (inches * COUNTS_PER_INCH);
                        RFTarget = robot.RFDrive.getCurrentPosition() + (int) (inches * COUNTS_PER_INCH);
                        LBTarget = robot.LBDrive.getCurrentPosition() + (int) (inches * COUNTS_PER_INCH);
                        RBTarget = robot.RBDrive.getCurrentPosition() - (int) (inches * COUNTS_PER_INCH);

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
                                (robot.LFDrive.isBusy() && robot.RFDrive.isBusy() && robot.LBDrive.isBusy() && robot.RBDrive.isBusy())) {

                            // Display it for the driver.
                            telemetry.addData("Path1", "Running to %7d :%7d", LFTarget, RFTarget);
                            telemetry.addData("Path2", "Running at %7d :%7d",
                                    robot.RFDrive.getCurrentPosition(),
                                    robot.RBDrive.getCurrentPosition());
                            telemetry.update();
                        }


                        robot.LFDrive.setPower(0);
                        robot.RFDrive.setPower(0);
                        robot.LBDrive.setPower(0);
                        robot.RBDrive.setPower(0);

                    }
                }
                robot.Carousel.setPower(((gamepad2.b || gamepad2.y) ? carouselPower : 0)*multiplier);
                robot.ClawCenter.setPosition(servoCenterAngle);
                robot.ClawLeft.setPosition(servoSideAngle);
                // robot.ClawRight.setPosition(1 - servoSideAngle);
                robot.Slide.setPower(slidePower*multiplier);

                // Show the elapsed game time and wheel power. Telemetry
                telemetry.addData("Status", "Run Time: " + runtime.toString());
                // telemetry.addData("Motors", "left (%.2f), right (%.2f), strafe (%.2f), b-button (%.2b), carouselPower(%.2d)", leftPower, rightPower, strafePower, gamepad1.b, carouselPower);
                // telemetry.addData("Servos","servoCenterAngle(%.2d), servoSideAngle(%.2d)", servoCenterAngle, servoSideAngle);
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

