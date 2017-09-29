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

    public void moveStraight(double power, boolean backwards){
        if(backwards == false) {
            fr.setPower(0.8);
            fl.setPower(0.8);
            br.setPower(0.8);
            bl.setPower(0.8);
        }

        else {
            fr.setPower(-0.8);
            fl.setPower(-0.8);
            br.setPower(-0.8);
            bl.setPower(-0.8);
        }
    }

    public void turn (double power, boolean left) {
        if (left == false) {
            fr.setPower(0.8);
            br.setPower(0.8);
        }
        else {
            fl.setPower(0.8);
            bl.setPower(0.8);
        }
    }

}
