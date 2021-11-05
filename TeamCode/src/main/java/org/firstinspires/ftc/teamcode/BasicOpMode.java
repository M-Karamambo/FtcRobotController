package org.firstinspires.ftc.teamcode;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.util.ElapsedTime;
import com.qualcomm.robotcore.util.Range;

@TeleOp(name="Basic: Linear OpMode.v1.6.1", group="Linear Opmode")
public class BasicOpMode extends LinearOpMode {
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
            double carouselPower = 0.1;
            while (opModeIsActive()) {

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
                if (gamepad1.right_bumper) multiplier = 0.5;
                // slowly increase motor speed to not send ducks flying
                if (gamepad1.b || gamepad1.y) {
                    carouselPower += 0.0025;
                }
                else {
                    carouselPower = 0;
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

                // Show the elapsed game time and wheel power. Telemetry
                telemetry.addData("Status", "Run Time: " + runtime.toString());
                telemetry.addData("Motors", "left (%.2f), right (%.2f), strafe (%.2f), b-button (%.2b), carouselPower(%.2f)",
                        leftPower, rightPower, strafePower, gamepad1.b, carouselPower);
                telemetry.update();
            }
        }
    }