package org.firstinspires.ftc.teamcode;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.util.ElapsedTime;
import com.qualcomm.robotcore.util.Range;

@TeleOp(name="Intake OpMode", group="Linear Opmode")
public class IntakeOpMode extends LinearOpMode {
    // initialize telemetry
    private final ElapsedTime runtime = new ElapsedTime();

    static final double COUNTS_PER_MOTOR_REV = 103.8; // 28 PPR at encoder shaft, 103.8 PPR at gearbox output shaft
    static final double DRIVE_GEAR_REDUCTION = 2.0;     // This is < 1.0 if geared UP
    static final double WHEEL_DIAMETER_INCHES = 0.8425;     // For figuring circumference
    static final double COUNTS_PER_INCH = (COUNTS_PER_MOTOR_REV * DRIVE_GEAR_REDUCTION) /
            (WHEEL_DIAMETER_INCHES * Math.PI);

    @Override
    public void runOpMode() {

        // initialize the hardware map
        RobotHardware robot = new RobotHardware();
        robot.init(hardwareMap);

        // Wait for start
        waitForStart();
        runtime.reset();
        double leftPower;
        double rightPower;
        double strafePower;
        double slidePower = 0;
        double carouselPower = 0.1;
        double intakePower;

        int clawSideIdx = 0;
        double[] clawSidePos = new double[]{0.03, 0.15, 0.30};
        boolean xPressed = false;
        boolean aPressed = false;

        int powerIdx = 1;
        int slideIdx = 0;
        double[] slidePow = new double[]{-1.0, 0.0, 1.0};
        double[] slidePos = new double[]{0.0, -15.0, -30.0};
        boolean upPressed = false;
        boolean sidePressed = false;
        boolean downPressed = false;
        boolean shouldRun = true;

        while (opModeIsActive()) {

            // multiplier for slow mode
            double multiplier = 1;

            // check for controller inputs
            double drive = gamepad1.left_stick_y;
            double strafe = gamepad1.left_stick_x;
            // double claw = gamepad1.right_stick_y;
            double turn = -gamepad1.right_stick_x;

            rightPower = Range.clip(drive - turn, -1.0, 1.0);
            leftPower = Range.clip(drive + turn, -1.0, 1.0);
            strafePower = Range.clip(strafe, -1.0, 1.0);
            intakePower = 0;

            //--------------------------------------------------------------------------------------

            if (gamepad1.right_bumper) {
                multiplier = 0.5;
            }

            if (gamepad1.b || gamepad1.y) {
                carouselPower += 0.0035;
            } else {
                carouselPower = 0;
            }

            //--------------------------------------------------------------------------------------

            if (gamepad1.left_trigger > 0) {
                intakePower = 1;
            }

            if (gamepad1.right_trigger > 0) {
                intakePower = -0.5;
            }

            //--------------------------------------------------------------------------------------

            if (!xPressed && gamepad1.x) {
                if (clawSideIdx != 2) {
                    clawSideIdx = 2;
                } else {
                    clawSideIdx = 0;
                }
            }
            xPressed = gamepad1.x;

            if (!aPressed && gamepad1.a) {
                if (clawSideIdx != 1) {
                    clawSideIdx = 1;
                } else {
                    clawSideIdx = 0;
                }
            }
            aPressed = gamepad1.a;

            //--------------------------------------------------------------------------------------

            if (!upPressed && gamepad1.dpad_up) {
                slideIdx = 2;
                shouldRun = true;
            }
            upPressed = gamepad1.dpad_up;

            if (!sidePressed && gamepad1.dpad_right) {
                slideIdx = 1;
                shouldRun = true;
            }
            sidePressed = gamepad1.dpad_right;

            if (!downPressed && gamepad1.dpad_down) {
                slideIdx = 0;
                shouldRun = true;
            }
            downPressed = gamepad1.dpad_down;

            //--------------------------------------------------------------------------------------

            // set power
            robot.LFDrive.setPower((leftPower - strafePower) * multiplier);
            robot.RFDrive.setPower((rightPower + strafePower) * multiplier);
            robot.LBDrive.setPower((leftPower + strafePower) * multiplier);
            robot.RBDrive.setPower((rightPower - strafePower) * multiplier);
            if (gamepad1.b) {
                robot.Carousel.setDirection(DcMotor.Direction.FORWARD);
            }
            if (gamepad1.y) {
                robot.Carousel.setDirection(DcMotor.Direction.REVERSE);
            }
            robot.Carousel.setPower(((gamepad1.b || gamepad1.y) ? carouselPower : 0) * multiplier);

            // robot.ClawCenter.setPosition(clawCenterPos[clawCenterIdx]);
            robot.ClawSide.setPosition(clawSidePos[clawSideIdx]);
            // robot.ClawRight.setPosition(1 - clawSidePos[clawSideIdx]);

            int slideTarget = (int) (slidePos[slideIdx] * COUNTS_PER_INCH);

            if (shouldRun) {
                runSlides(robot, 2, slideTarget, 1);
            }

            if (Math.abs(slideTarget - robot.Slide.getCurrentPosition()) < 30) {
                shouldRun = false;
            }
            robot.Intake.setPower(intakePower * multiplier);

            // Show the elapsed game time and wheel power. Telemetry
            telemetry.addData("Status", "Run Time: " + runtime.toString());
            telemetry.addData("Intake motor", "Left trigger (%f), right trigger (%f)",
                    gamepad1.left_trigger, gamepad1.right_trigger);
            telemetry.addData("Claw side", "xPressed (%b), x (%b), sideIdx (%d), setPosition (%f)",
                    xPressed, gamepad1.x, clawSideIdx, clawSidePos[clawSideIdx]);
            telemetry.addData("Slide motor", "powerIdx (%d), power (%f), position (%7d)",
                    powerIdx, slidePow[powerIdx], robot.Slide.getCurrentPosition());
            telemetry.addData("dpad arrows", "dpadU (%b), dpadR (%b), dpadD (%b)",
                    gamepad1.dpad_up, gamepad1.dpad_right, gamepad1.dpad_down);
            telemetry.update();
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
