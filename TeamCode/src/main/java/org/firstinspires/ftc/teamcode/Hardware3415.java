package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.HardwareMap;

/**
 * Created by mervyn.mathew on 10/12/2017.
 */

public class Hardware3415 {

    public DcMotor br = null;
    public DcMotor bl = null;
    public DcMotor fr = null;
    public DcMotor fl = null;
    HardwareMap hwMap = null;

    public static final String brName = "back_right";
    public static final String blName = "back_left";
    public static final String frName = "front_right";
    public static final String flName = "front_left";

    

    // Constructor
    public Hardware3415() {
    }

    public void init(HardwareMap ahwMap, boolean autonomous) {

    }

    public void setDrivePower (double power) {
        br.setPower(power);
        bl.setPower(power);
        fr.setPower(power);
        fl.setPower(power);
    }

    public void changeDriveMode (DcMotor.RunMode mode) {
        br.setMode(mode);
        bl.setMode(mode);
        fr.setMode(mode);
        fl.setMode(mode);
    }

    public boolean moveStraight (double inches, LinearOpMode opMode) {
        changeDriveMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER); // Sets encoder position to 0.

        while (br.getCurrentPosition() != 0 && opMode.opModeIsActive())
            opMode.telemetry.addLine("Resettting Encoders");

        changeDriveMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        opMode.telemetry.addLine("Beginning to Move");
        opMode.sleep(500);

        int targetTick = (int) (inches * 1140.0 / (4.0 * Math.PI * 2.0)); // The value here is from last year.
        br.setTargetPosition(targetTick);

        if (inches > 0) {
            setDrivePower(0.35); // The value here is from last year.
            while (br.getCurrentPosition() < targetTick && opMode.opModeIsActive() && !opMode.isStopRequested()) {
                opMode.telemetry.addData("Encoder Pos: ", br.getCurrentPosition());
                opMode.telemetry.addData("Target Pos: ", targetTick);
            }
        }
        else {
            setDrivePower(-0.35); // This value here is from last year.
            while (br.getCurrentPosition() > targetTick && opMode.opModeIsActive() && !opMode.isStopRequested()) {
                opMode.telemetry.addData("Encoder Pos: ", br.getCurrentPosition());
                opMode.telemetry.addData("Target Pos: ", targetTick);
            }
        }

        setDrivePower(0);
        return true;
    }
}
