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

        int clawCenterIdx = 0;
        double[] clawCenterPos = new double[]{0.0, 1.0};
        boolean aPressed = false;
        int clawSideIdx = 0;
        double[] clawSidePos = new double[]{0.0, 0.5};
        boolean xPressed = false;

        int slideIdx = 0;
        double[] slidePos = new double[]{0.0, 0.5, 1.0};

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

            if (gamepad1.right_bumper) {
                multiplier = 0.5;
            }

            if (gamepad1.b || gamepad1.y) {
                carouselPower += 0.0025;
            }
            else {
                carouselPower = 0;
            }

            if (!aPressed && gamepad1.a) {
                if (clawCenterIdx == 0) {
                    clawCenterIdx = 1;
                }
                else if (clawCenterIdx == 1) {
                    clawCenterIdx = 0;
                }
                /*switch(clawCenterIdx) {
                    case 0:
                        clawCenterIdx = 1;
                        break;
                    case 1:
                        clawCenterIdx = 0;
                        break;
                }*/
            }
            aPressed = gamepad1.a;

            if (!xPressed && gamepad1.x) {
                if (clawSideIdx == 0) {
                    clawSideIdx = 1;
                }
                else if (clawSideIdx == 1) {
                    clawSideIdx = 0;
                }
                /*switch(clawSideIdx) {
                    case 0:
                        clawSideIdx = 1;
                        break;
                    case 1:
                        clawSideIdx = 0;
                        break;
                }*/
            }
            xPressed = gamepad1.x;

            if (gamepad1.dpad_up && slideIdx <= 2) {
                slideIdx++;
            }
            if (gamepad1.dpad_down && slideIdx >= 0) {
                slideIdx--;
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
            robot.Carousel.setPower(((gamepad1.b || gamepad1.y) ? carouselPower : 0) * multiplier);

            robot.ClawCenter.setPosition(clawCenterPos[clawCenterIdx]);
            robot.ClawLeft.setPosition(clawSidePos[clawSideIdx]);
            // robot.ClawRight.setPosition(1 - clawSidePos[clawSideIdx]);

            // Show the elapsed game time and wheel power. Telemetry
            telemetry.addData("Status", "Run Time: " + runtime.toString());
            telemetry.addData("Claw center", "aPressed (%b), a (%b), centerIdx (%d), setPosition (%f)",
                    aPressed, gamepad1.a, clawCenterIdx, clawCenterPos[clawCenterIdx]);
            telemetry.addData("Claw side", "xPressed (%b), x (%b), sideIdx (%d), setPosition (%f)",
                    xPressed, gamepad1.x, clawSideIdx, clawSidePos[clawSideIdx]);
            telemetry.update();
        }
    }
}
