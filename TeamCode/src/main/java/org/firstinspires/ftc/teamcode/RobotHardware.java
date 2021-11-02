package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.HardwareMap;

public class RobotHardware {
    public RobotHardware() {}
    public HardwareMap hmap;
    public DcMotor LFDrive;
    public DcMotor RFDrive;
    public DcMotor LBDrive;
    public DcMotor RBDrive;
    public DcMotor Carousel;



    public void init(HardwareMap myHmap){
        hmap = myHmap;
        LFDrive = hmap.get(DcMotor.class, "left_front_drive"); //[TODO] rename these
        RFDrive = hmap.get(DcMotor.class, "right_front_drive");
        LBDrive = hmap.get(DcMotor.class, "left_back_drive");
        RBDrive = hmap.get(DcMotor.class, "right_back_drive");
        Carousel = hmap.get(DcMotor.class, "carousel_spinner");

        LFDrive.setDirection(DcMotor.Direction.FORWARD);
        RFDrive.setDirection(DcMotor.Direction.REVERSE);
        LBDrive.setDirection(DcMotor.Direction.FORWARD);
        RBDrive.setDirection(DcMotor.Direction.REVERSE);
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