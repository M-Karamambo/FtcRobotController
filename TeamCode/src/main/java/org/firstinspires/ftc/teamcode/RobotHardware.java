package org.firstinspires.ftc.teamcode;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
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
    public Servo ClawLeft;
    public double fakeUnit = 5/20; // 20 is arbitrary
    // the succses of this implementation depends on the accuracy of the forward and starfe methods working correctley
    // public Servo ClawRight;

    public DcMotor Slide;

    public void init(HardwareMap myHmap){
        hmap = myHmap;
        LFDrive = hmap.get(DcMotor.class, "left_front_drive");
        RFDrive = hmap.get(DcMotor.class, "right_front_drive");
        LBDrive = hmap.get(DcMotor.class, "left_back_drive");
        RBDrive = hmap.get(DcMotor.class, "right_back_drive");
        Carousel = hmap.get(DcMotor.class, "carousel_spinner");

        ClawCenter = hmap.get(Servo.class, "claw_center");
        ClawLeft = hmap.get(Servo.class, "claw_left");
        // ClawRight = hmap.get(Servo.class, "claw_right");
        Slide = hmap.get(DcMotor.class, "slide_motor");

        LFDrive.setDirection(DcMotor.Direction.FORWARD);
        RFDrive.setDirection(DcMotor.Direction.REVERSE);
        LBDrive.setDirection(DcMotor.Direction.FORWARD);
        RBDrive.setDirection(DcMotor.Direction.REVERSE);
        Slide.setDirection(DcMotor.Direction.REVERSE);
    }
    
    public void autoinit(HardwareMap myHmap){

        LFDrive.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        RFDrive.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        LBDrive.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        RBDrive.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);

        LFDrive.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        RFDrive.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        LBDrive.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        RBDrive.setMode(DcMotor.RunMode.RUN_USING_ENCODER);

        LFDrive.setDirection(DcMotor.Direction.FORWARD);
        RFDrive.setDirection(DcMotor.Direction.REVERSE);
        LBDrive.setDirection(DcMotor.Direction.FORWARD);
        RBDrive.setDirection(DcMotor.Direction.REVERSE);
    }

}

