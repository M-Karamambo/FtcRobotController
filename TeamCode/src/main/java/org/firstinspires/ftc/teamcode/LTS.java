package org.firstinspires.ftc.teamcode;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.util.ElapsedTime;
import com.qualcomm.robotcore.util.Range;

@TeleOp(name="LTS", group="Linear Opmode")
public class LTS extends LinearOpMode {
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
            if(gamepad1.y) {
                robot.Carousel.setDirection(DcMotor.Direction.REVERSE);
            }
            robot.Carousel.setPower(((gamepad1.b || gamepad1.y) ? carouselPower : 0)*multiplier);
            robot.ClawCenter.setPosition(servoCenterAngle);
            robot.ClawSide.setPosition(servoSideAngle);
            // robot.ClawRight.setPosition(1 - servoSideAngle);
            robot.Slide.setPower(slidePower*multiplier);

            // Show the elapsed game time and wheel power. Telemetry
            telemetry.addData("Status", "Run Time: " + runtime.toString());
            // telemetry.addData("Motors", "left (%.2f), right (%.2f), strafe (%.2f), b-button (%.2b), carouselPower(%.2d)", leftPower, rightPower, strafePower, gamepad1.b, carouselPower);
            // telemetry.addData("Servos","servoCenterAngle(%.2d), servoSideAngle(%.2d)", servoCenterAngle, servoSideAngle);
            telemetry.update();
        }
    }
}