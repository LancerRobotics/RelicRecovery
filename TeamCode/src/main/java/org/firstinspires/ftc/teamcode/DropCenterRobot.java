package org.firstinspires.ftc.teamcode;

/**
 * Created by dina.brustein on 9/27/2017.
 */
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.HardwareMap;

public class DropCenterRobot {

    HardwareMap hMap;

    public DcMotor fr = hMap.dcMotor.get("front_right");
    public DcMotor fl = hMap.dcMotor.get("front_left");
    public DcMotor bl = hMap.dcMotor.get("back_left");
    public DcMotor br = hMap.dcMotor.get("back_right");

    public DropCenterRobot(){

    }



}
