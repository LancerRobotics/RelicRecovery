package org.firstinspires.ftc.teamcode;


import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.util.ElapsedTime;
import com.qualcomm.robotcore.util.Range;

/**
 * Created by jake.wiseberg on 9/29/2017.
 */

@Autonomous(name="Omni Teleop", group="TeleOp")
public class TeleOpOmni extends OpMode {

    //values, using global values for faster runtime
    HardwareMap hMap;
    public DcMotor fr;
    public DcMotor fl;
    public DcMotor bl;
    public DcMotor br;
    private ElapsedTime runtime = new ElapsedTime();
    double[] motorPwr = new double[4];
    double x,y,z;

    @Override
    public void init() {
        telemetry.addData("Status", "In progress...");

        //set motors to hardware
        fr = hMap.dcMotor.get("front_right");
        fl = hMap.dcMotor.get("front_left");
        bl = hMap.dcMotor.get("back_left");
        br = hMap.dcMotor.get("back_right");

        telemetry.addData("Status", "Initialized");
    }

    @Override
    public void start() {
        runtime.reset();
    }

    @Override
    public void loop() {
        //getting joystick values
        y = gamepad1.left_stick_y; //left joystick y axis
        x = gamepad1.left_stick_x; //left joystick x axis
        z = gamepad1.right_stick_x; //right joystick x axis

        //calculating the motor power
        motorPwr[0] = Range.clip(-y+x-z, -1.0, 1.0); //front right
        motorPwr[1] = Range.clip(y+x-z, -1.0, 1.0); //front left
        motorPwr[2] = Range.clip(-y-x-z, -1.0, 1.0); //back right
        motorPwr[3] = Range.clip(y-x-z, -1.0, 1.0); //back left

        //setting the motor powers
        fr.setPower(motorPwr[0]);
        fl.setPower(motorPwr[1]);
        br.setPower(motorPwr[2]);
        bl.setPower(motorPwr[3]);

        telemetry.addData("Status", "Run Time: " + runtime.toString());
    }
}
