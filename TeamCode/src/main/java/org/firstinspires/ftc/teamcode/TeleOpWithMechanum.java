package org.firstinspires.ftc.teamcode;

/**
 * Created by dina.brustein on 9/27/2017.
 */
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.util.Range;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;

public class TeleOpWithMechanum {

    MechanumRobot robot = new MechanumRobot();

    public void runOpMode(){

        //if the joystick x value is negative
        robot.strafe(0.86,true);

        //if the joystick x value is positive
        robot.strafe(0.86,false);

        //if the joystick y value is negative
        robot.strafe(0.86,true);

        //if the joystick y value is positive
        robot.strafe(0.86,false);
    }

}
