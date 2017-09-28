package org.firstinspires.ftc.teamcode;

/**
 * Created by dina.brustein on 9/27/2017.
 */
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;

import static android.R.attr.left;

@Autonomous(name = "Mechanum", group = "TeleOp")
public class MechanumRobot {

    HardwareMap hMap;
    LinearOpMode opMode;
    DcMotor fl = null;
    DcMotor fr = null;
    DcMotor bl = null;
    DcMotor br = null;

    public MechanumRobot(){

    }


    public void strafe(double power, boolean left){

        if(!left){
            fl.setPower(-power);
            fr.setPower(power);
            bl.setPower(-power);
            br.setPower(power);
        }

        else {
            fl.setPower(power);
            fr.setPower(-power);
            bl.setPower(power);
            br.setPower(-power);
        }
    }

    public void setDrivePower(double power, boolean backwards){
        if(!backwards){
            fl.setPower(power);
            fr.setPower(power);
            bl.setPower(power);
            br.setPower(power);
        }
        else{
            fl.setPower(-power);
            fr.setPower(-power);
            bl.setPower(-power);
            br.setPower(-power);
        }
    }

    public void init(HardwareMap hMap){

        DcMotor fr = hMap.dcMotor.get("front_right");
        DcMotor fl = hMap.dcMotor.get("front_left");
        DcMotor br = hMap.dcMotor.get("back_right");
        DcMotor bl = hMap.dcMotor.get("back_left");

    }

    public void turn(double power, boolean left){

        if(!left){
            fl.setPower(power);
            bl.setPower(power);
        }
        else {
            fr.setPower(power);
            br.setPower(power);
        }
    }
}
