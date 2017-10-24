package org.firstinspires.ftc.teamcode;

/**
 * Created by dina.brustein on 9/27/2017.
 */

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.Gamepad;
import com.qualcomm.robotcore.util.Range;

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
    public double frpower, flpower, brpower, blpower;

    public void setup(){

    }

    public void runOpMode(){

        Gamepad gamepad1 = new Gamepad();
        Gamepad gamepad2 = new Gamepad();
        
        robot.init(hardwareMap, false);

        z = gamepad1.left_stick_x; //moving left/right
        y = gamepad1.left_stick_y; //moving forwards/backwards
        x = gamepad1.right_stick_x; //turning

        //Sets the motor powers of the wheels to the correct power based on all three of the above gyro values and
        //scales them accordingly
        flpower = Range.scale((-x + y - z), -1, 1, -robot.MAX_MOTOR_SPEED, robot.MAX_MOTOR_SPEED);
        frpower = Range.scale((-x - y - z), -1, 1, -robot.MAX_MOTOR_SPEED, robot.MAX_MOTOR_SPEED);
        blpower = Range.scale((x + y - z), -1, 1, -robot.MAX_MOTOR_SPEED, robot.MAX_MOTOR_SPEED);
        brpower = Range.scale((x - y - z), -1, 1, -robot.MAX_MOTOR_SPEED, robot.MAX_MOTOR_SPEED);

        //Sets each motor power to the correct power
        robot.fl.setPower(flpower);
        robot.fr.setPower(frpower);
        robot.bl.setPower(blpower);
        robot.br.setPower(brpower);

        //Gamepad controls
        if (x == 0 && y == 0 && z == 0) {
            if (gamepad1.dpad_right) {
                robot.bl.setPower(robot.MAX_MOTOR_SPEED);
                robot.fl.setPower(robot.MAX_MOTOR_SPEED);
            } else if (gamepad1.dpad_up) {
                robot.bl.setPower(-robot.MAX_MOTOR_SPEED);
                robot.fl.setPower(-robot.MAX_MOTOR_SPEED);
            } else if (gamepad1.dpad_down) {
                robot.br.setPower(robot.MAX_MOTOR_SPEED);
                robot.fr.setPower(robot.MAX_MOTOR_SPEED);
            } else if (gamepad1.dpad_left) {
                robot.br.setPower(-robot.MAX_MOTOR_SPEED);
                robot.fr.setPower(-robot.MAX_MOTOR_SPEED);
                
            }
        }

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

        //Important data to the driver
        telemetry.addData("FR Power", robot.fr.getPower());
        telemetry.addData("FL Power", robot.fl.getPower());
        telemetry.addData("BR Power", robot.br.getPower());
        telemetry.addData("BL Power", robot.bl.getPower());

    }

}
