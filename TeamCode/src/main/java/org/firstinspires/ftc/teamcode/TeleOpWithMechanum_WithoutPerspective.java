package org.firstinspires.ftc.teamcode;

/**
 * Created by dina.brustein on 9/27/2017.
 */

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.Gamepad;
import com.qualcomm.robotcore.util.Range;

@TeleOp

public class TeleOpWithMechanum_WithoutPerspective extends LinearOpMode {
    HardwareMechanumRobot robot = new HardwareMechanumRobot();
    public static double x, y, z, flPower, frPower, brPower, blPower;

    public void setup(){

    }

    public void runOpMode(){
        
        robot.init(hardwareMap, false);

        waitForStart();

        while(opModeIsActive()) {

            //Sets the gamepad values to x, y, and z
            z = gamepad1.right_stick_x; //sideways
            y = gamepad1.left_stick_y; //forward and backward
            x = gamepad1.left_stick_x; //rotation

            //Sets the motor powers of the wheels to the correct power based on all three of the above gyro values and
            //scales them accordingly
            flPower = Range.scale((-x - y - z), -1, 1, -robot.MAX_MOTOR_SPEED, robot.MAX_MOTOR_SPEED);
            frPower = Range.scale((-x + y - z), -1, 1, -robot.MAX_MOTOR_SPEED, robot.MAX_MOTOR_SPEED);
            blPower = Range.scale((x - y - z), -1, 1, -robot.MAX_MOTOR_SPEED, robot.MAX_MOTOR_SPEED);
            brPower = Range.scale((x + y - z), -1, 1, -robot.MAX_MOTOR_SPEED, robot.MAX_MOTOR_SPEED);

            //Sets each motor power to the correct power
            robot.fl.setPower(flPower);
            robot.fr.setPower(frPower);
            robot.bl.setPower(blPower);
            robot.br.setPower(brPower);

            if(gamepad1.a) {
                robot.arm1.setPosition(robot.armDOWN);
                robot.arm2.setPosition(robot.armDOWN);
            }
            if(gamepad1.b) {
                robot.arm1.setPosition(robot.armHALFWAY);
                robot.arm2.setPosition(robot.armHALFWAY);
            }
            if(gamepad1.y){
                robot.arm1.setPosition(robot.armDROP);
                robot.arm2.setPosition(robot.armDROP);
            }
        }
    }

}
