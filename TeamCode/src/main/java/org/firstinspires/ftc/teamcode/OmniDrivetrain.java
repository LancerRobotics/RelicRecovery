package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.util.ElapsedTime;

/**
 * Created by jake.wiseberg on 9/29/2017.
 */

public class OmniDrivetrain extends OpMode {

    //values
    HardwareMap hMap;
    public DcMotor fr;
    public DcMotor fl;
    public DcMotor bl;
    public DcMotor br;
    private ElapsedTime runtime = new ElapsedTime();
    double[] motorVal = new double[4];
    double x,y,z;

    @Override
    public void init() {
        telemetry.addData("Status", "In progress...");
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
        x = gamepad1.left_stick_x;
        y = gamepad1.left_stick_y;

        telemetry.addData("Status", "Run Time: " + runtime.toString());
    }

/*
    //joystick values
    int y= joystick.joy1_y1;
    int x= joystick.joy1_x1;
    int z= joystick.joy1_x2;

    //motor values using the variables above
//note the negatives
    motorTrigger(frontRight,x*.75,-y*.75,z);
    motorTrigger(backRight,-x*.75,-y*.75,z);
    motorTrigger(frontLeft,x*.75,y*.75,z);

//other stuff maybe gotto look more into this
    motor[backRight]=(-y-x-z)*.75;



    //joystick values (translated)
    int x= gamepad1.left_stick_x; //side to side
    int y= gamepad1.left_stick_y; //forward/backward
    int z= gamepad1.right_stick_x; //sideways

    //motor values using the variables above
    motor[frontLeft]=(y+x-z)*.75;
    motor[backLeft]= (y-x-z)*.75; // -y-x||x+y
    motor[frontRight]=(-y+x-z)*.75;//-y+x||-x+y

    //__________________________
    flPower = Range.scale( (y+x-z), -1, 1, -MAX_SPEED, MAXSPEED  ;

    //note the negatives
    motorTrigger(frontRight,x*.75,-y*.75,z);
    motorTrigger(backRight,-x*.75,-y*.75,z);
    motorTrigger(frontLeft,x*.75,y*.75,z);

//other stuff maybe gotto look more into this
    motor[frontLeft]=(y+x-z)*.75;
    motor[backLeft]= (y-x-z)*.75; // -y-x||x+y
    motor[frontRight]=(-y+x-z)*.75;//-y+x||-x+y
    motor[backRight]=(-y-x-z)*.75;

    */
}
