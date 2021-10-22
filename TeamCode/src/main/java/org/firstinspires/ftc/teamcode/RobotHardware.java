package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.util.ElapsedTime;

public class RobotHardware {
    public RobotHardware() {}
    public HardwareMap hmap;
    public DcMotor LFDrive = hmap.get(DcMotor.class, "left_back_drive");
    public DcMotor RFDrive = hmap.get(DcMotor.class, "left_back_drive");
    public DcMotor LBDrive = hmap.get(DcMotor.class, "right_front_drive");
    public DcMotor RBDrive = hmap.get(DcMotor.class, "left_front_drive");



    public void init(){
    }
}
