package org.firstinspires.ftc.teamcode;

/**
 * Created by dina.brustein on 9/27/2017.
 */

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.Gamepad;


public class TeleOpWithMechanum_WithoutPerspective extends LinearOpMode {
    HardwareMechanumRobot robot = new HardwareMechanumRobot();
    public static double x, y, z;

    public void setup(){

    }

    public void runOpMode(){

        Gamepad gamepad1 = new Gamepad();
        Gamepad gamepad2 = new Gamepad();
        
        robot.init(hardwareMap, false);

        x = gamepad1.left_stick_x; //moving left/right
        y = gamepad1.left_stick_y; //moving forwards/backwards
        x = gamepad1.right_stick_x; //turning

        if(x < -0.15){
            robot.strafe(0.86,true);
        }

        if(x > 0.15){
            robot.strafe(0.86,false);
        }

        if(y > 0.15){
            robot.setDrivePower(0.8,false);
        }

        if(y < -0.15){
            robot.setDrivePower(0.8,true);
        }

        if(z > 0.15){
            robot.turn(0.35,false);
        }

        if(z < -0.15){
            robot.turn(0.35, true);
        }
    }

}
