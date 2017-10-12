package org.firstinspires.ftc.teamcode;

/**
 * Created by dina.brustein on 9/27/2017.
 */

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.Gamepad;

import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.robotcore.external.navigation.AxesOrder;
import org.firstinspires.ftc.robotcore.external.navigation.AxesReference;

import com.qualcomm.hardware.bosch.BNO055IMU;
import com.qualcomm.hardware.bosch.JustLoggingAccelerationIntegrator;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.util.ElapsedTime;
import com.qualcomm.robotcore.util.Range;

import org.firstinspires.ftc.robotcore.external.navigation.Acceleration;
import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.robotcore.external.navigation.AxesOrder;
import org.firstinspires.ftc.robotcore.external.navigation.AxesReference;
import org.firstinspires.ftc.robotcore.external.navigation.Orientation;
import org.firstinspires.ftc.robotcore.external.navigation.Position;
import org.firstinspires.ftc.robotcore.external.navigation.Velocity;


public class TeleOpWithMechanum_WithPerspective extends LinearOpMode {
    HardwareMechanumRobot robot = new HardwareMechanumRobot();
    public static double x, y, z, trueX, trueY;

    public void setup(){

    }

    public void runOpMode(){

        Gamepad gamepad1 = new Gamepad();
        Gamepad gamepad2 = new Gamepad();
        
        robot.init(hardwareMap, false);

        z = gamepad1.left_stick_x; //moving left/right
        y = gamepad1.left_stick_y; //moving forwards/backwards
        x = gamepad1.right_stick_x; //turning
/*
        trueX = ((Math.cos(Math.toRadians(360 - robot.imu.angleUnit.formatAngle(angles.angleUnit,angles.secondAngle )   ))) * x) - ((Math.sin(Math.toRadians(360 - Artemis.convertYaw(Artemis.navx_device.getYaw())))) * y); //sets trueX to rotated value
        trueY = ((Math.sin(Math.toRadians(360 - Artemis.convertYaw(Artemis.navx_device.getYaw())))) * x) + ((Math.cos(Math.toRadians(360 - Artemis.convertYaw(Artemis.navx_device.getYaw())))) * y);
*/

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
