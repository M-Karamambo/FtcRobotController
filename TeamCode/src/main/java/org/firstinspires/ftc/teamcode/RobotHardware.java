
package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.hardware.DcMotor;
// import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.util.ElapsedTime;

public class RobotHardware {

    public static HardwareMap hMap = null;
    public static DcMotor LFDrive = hMap.get(DcMotor.class, "left_front_drive");
    public static DcMotor RFDrive = hMap.get(DcMotor.class, "right_front_drive");
    public static DcMotor LBDrive = hMap.get(DcMotor.class, "left_back_drive");
    public static DcMotor RBDrive = hMap.get(DcMotor.class, "right_back_drive");
    // public static DcMotor Carousel = hMap.get(DcMotor.class, "carousel_spinner");

    public void init(HardwareMap hMap) {
        LFDrive.setDirection(DcMotor.Direction.FORWARD);
        RFDrive.setDirection(DcMotor.Direction.REVERSE);
        LBDrive.setDirection(DcMotor.Direction.FORWARD);
        RBDrive.setDirection(DcMotor.Direction.REVERSE);

        LFDrive.setPower(0);
        RFDrive.setPower(0);
        LBDrive.setPower(0);
        RBDrive.setPower(0);
    }
}
