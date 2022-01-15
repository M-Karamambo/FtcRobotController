package org.firstinspires.ftc.teamcode;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.Servo;

public class RobotHardware {
    public final String VER = "1.6.4";
    public RobotHardware() {}
    public HardwareMap hmap;
    public DcMotor LFDrive;
    public DcMotor RFDrive;
    public DcMotor LBDrive;
    public DcMotor RBDrive;
    public DcMotor Carousel;

    public Servo ClawCenter;
    public Servo ClawSide;
    public Servo ClawRight;

    public DcMotor Slide;
    public DcMotor Intake;

    public void init(HardwareMap myHmap){
        hmap = myHmap;

        LFDrive = hmap.get(DcMotor.class, "left_front_drive"); //[TODO] rename these
        RFDrive = hmap.get(DcMotor.class, "right_front_drive");
        LBDrive = hmap.get(DcMotor.class, "left_back_drive");
        RBDrive = hmap.get(DcMotor.class, "right_back_drive");
        Carousel = hmap.get(DcMotor.class, "carousel_spinner");

        ClawCenter = hmap.get(Servo.class, "claw_center");
        ClawSide = hmap.get(Servo.class, "claw_side");

        Slide = hmap.get(DcMotor.class, "slide_motor");
        Intake = hmap.get(DcMotor.class, "intake_motor");

        LFDrive.setDirection(DcMotor.Direction.FORWARD);
        RFDrive.setDirection(DcMotor.Direction.REVERSE);
        LBDrive.setDirection(DcMotor.Direction.FORWARD);
        RBDrive.setDirection(DcMotor.Direction.REVERSE);

        ClawCenter.setPosition(0.0);
        ClawSide.setPosition(0.0);

        Slide.setDirection(DcMotor.Direction.FORWARD);
        Intake.setDirection(DcMotor.Direction.FORWARD);
    }

    public void autoinit(HardwareMap myHmap){
        hmap = myHmap;

        LFDrive = hmap.get(DcMotor.class, "left_front_drive"); //[TODO] rename these
        RFDrive = hmap.get(DcMotor.class, "right_front_drive");
        LBDrive = hmap.get(DcMotor.class, "left_back_drive");
        RBDrive = hmap.get(DcMotor.class, "right_back_drive");
        Carousel = hmap.get(DcMotor.class, "carousel_spinner");

        ClawCenter = hmap.get(Servo.class, "claw_center");
        ClawSide = hmap.get(Servo.class, "claw_side");

        Slide = hmap.get(DcMotor.class, "slide_motor");
        Intake = hmap.get(DcMotor.class, "intake_motor");

        LFDrive.setDirection(DcMotor.Direction.FORWARD);
        RFDrive.setDirection(DcMotor.Direction.REVERSE);
        LBDrive.setDirection(DcMotor.Direction.FORWARD);
        RBDrive.setDirection(DcMotor.Direction.REVERSE);

        ClawCenter.setPosition(0.0);
        // ClawSide.setPosition(0.0);

        Slide.setDirection(DcMotor.Direction.FORWARD);
        Intake.setDirection(DcMotor.Direction.FORWARD);
    }
}

