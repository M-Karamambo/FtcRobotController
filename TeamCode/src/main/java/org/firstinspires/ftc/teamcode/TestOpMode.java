package org.firstinspires.ftc.teamcode;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.util.ElapsedTime;
import com.qualcomm.robotcore.util.Range;

@TeleOp(name="Test OpMode", group="Linear Opmode")
public class TestOpMode extends LinearOpMode {
    // initialize telemetry
    private final ElapsedTime runtime = new ElapsedTime();

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
        double servoCenterAngle = 0.0;
        double servoSideAngle = 0.0;
        double testAngle = 0.0;

        while (opModeIsActive()) {

            // multiplier for slow mode
            double multiplier = 1;

            // check for controller inputs
            double drive = gamepad1.left_stick_y;
            double strafe = gamepad1.left_stick_x;
            // double claw = gamepad1.right_stick_y;
            double turn  =  -gamepad1.right_stick_x;

            // process inputs
            /*
            servoSideAngle = Range.clip(claw, -1.0, 1.0) * 0.5 + 0.5; // Shrink and offset range for servo
            if (Math.abs(0.5 - servoSideAngle) > 0.25) {
                turn = 0; // Ignore turning if moving claw up or down
            }
            */

            rightPower = Range.clip(drive - turn, -1.0, 1.0);
            leftPower = Range.clip(drive + turn, -1.0, 1.0);
            strafePower = Range.clip(strafe, -1.0, 1.0);
            if (gamepad1.right_bumper) multiplier = 0.5;
            // slowly increase motor speed to not send ducks flying
            if (gamepad1.b || gamepad1.y) {
                carouselPower += 0.0025;
            }
            else {
                carouselPower = 0;
            }

            if (servoSideAngle <= 1.0 && gamepad1.a) {
                servoSideAngle += 0.001;
            }
            if (servoSideAngle >= 0.0 && gamepad1.x) {
                servoSideAngle -= 0.001;
            }

            if (servoCenterAngle <= 1.0 && gamepad1.right_trigger > 0) {
                servoCenterAngle += 0.001;
            }
            if (servoCenterAngle >= 0.0 && gamepad1.left_trigger > 0) {
                servoCenterAngle -= 0.001;
            }

            /*
            if (gamepad1.right_trigger > 0) {
                testAngle += 0.001;
            }
            if (gamepad1.left_trigger > 0) {
                testAngle -= 0.001;
            }
            */

            // servoCenterAngle = gamepad1.right_trigger;
            // servoCenterAngle = 0.5;

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
            if(gamepad1.y) {
                robot.Carousel.setDirection(DcMotor.Direction.REVERSE);
            }
            robot.Carousel.setPower(((gamepad1.b || gamepad1.y) ? carouselPower : 0)*multiplier);
            robot.ClawCenter.setPosition(servoCenterAngle);
            robot.ClawLeft.setPosition(servoSideAngle);
            // robot.ClawRight.setPosition(1 - servoSideAngle);
            // robot.TestServo.setPosition(testAngle);
            robot.Slide.setPower(slidePower*multiplier);

            // Show the elapsed game time and wheel power. Telemetry
            telemetry.addData("Status", "Run Time: " + runtime.toString());
            telemetry.addData("Test angle", "%.2f", testAngle);
            telemetry.addData("Claw center", "%.2f", servoCenterAngle);
            telemetry.addData("Claw center", "%.2f", servoSideAngle);
            telemetry.update();
        }
    }
}
