package org.firstinspires.ftc.teamcode;

/**
 * Created by yibin.long on 9/26/2017.
 */

public class OmniDrivetrain {
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
*/


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

}
