package org.firstinspires.ftc.teamcode;

/**
 * Created by dina.brustein on 9/27/2017.
 */
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.HardwareMap;

public class DropCenterDrive {

    HardwareMap hMap;

    DcMotor fr = hMap.dcMotor.get("front_right");
    DcMotor fl = hMap.dcMotor.get("front_left");
    DcMotor bl = hMap.dcMotor.get("back_left");
    DcMotor br = hMap.dcMotor.get("back_right");

}
