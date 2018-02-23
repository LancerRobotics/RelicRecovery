package org.firstinspires.ftc.teamcode;

/**
 * Created by yibin.long on 2/19/2018.
 */

import com.qualcomm.hardware.bosch.BNO055IMU;
import com.qualcomm.hardware.bosch.JustLoggingAccelerationIntegrator;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.ColorSensor;

import org.firstinspires.ftc.robotcore.external.ClassFactory;
import org.firstinspires.ftc.robotcore.external.Func;
import org.firstinspires.ftc.robotcore.external.matrices.OpenGLMatrix;
import org.firstinspires.ftc.robotcore.external.matrices.VectorF;
import org.firstinspires.ftc.robotcore.external.navigation.Acceleration;
import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.robotcore.external.navigation.AxesOrder;
import org.firstinspires.ftc.robotcore.external.navigation.AxesReference;
import org.firstinspires.ftc.robotcore.external.navigation.Orientation;
import org.firstinspires.ftc.robotcore.external.navigation.Position;
import org.firstinspires.ftc.robotcore.external.navigation.RelicRecoveryVuMark;
import org.firstinspires.ftc.robotcore.external.navigation.Velocity;

import com.qualcomm.hardware.bosch.BNO055IMU;
import com.qualcomm.hardware.bosch.JustLoggingAccelerationIntegrator;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.AnalogInput;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.util.ElapsedTime;
import com.qualcomm.robotcore.util.Range;

import org.firstinspires.ftc.robotcore.external.navigation.Acceleration;
import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.robotcore.external.navigation.AxesOrder;
import org.firstinspires.ftc.robotcore.external.navigation.AxesReference;
import org.firstinspires.ftc.robotcore.external.navigation.Orientation;
import org.firstinspires.ftc.robotcore.external.navigation.Position;
import org.firstinspires.ftc.robotcore.external.navigation.Velocity;
import org.firstinspires.ftc.teamcode.HardwareMechanumRobot;

@TeleOp(name="Teleop No Persepctive", group="plswork")
public class MecanumTestNEWPerspective extends LinearOpMode{
    HardwareMechanumRobot robot = new HardwareMechanumRobot();
    // Declare OpMode members.
    private ElapsedTime runtime = new ElapsedTime();
    public static double frPower, flPower, brPower, blPower;
    public static boolean Button1_b, Button1_a, Button2_a, Button2_b, Button2_x, Button2_y;

    //values for motor
    double[] motorPwr = new double[4];
    double x,y,z;


    //potentiometer
    AnalogInput potentiometer;


    @Override
    public void runOpMode() {
        telemetry.addData("Status", "In Progress...");
        telemetry.update();

        robot.init(hardwareMap, false);

        telemetry.addData("Status", "Initialized");
        telemetry.update();
        // Wait for the game to start (driver presses PLAY)
        waitForStart();
        runtime.reset();

        //Testing to see if all telemetry is kept
        telemetry.setAutoClear(false);

        // run until the end of the match (driver presses STOP)
        while (opModeIsActive() && !isStopRequested()) {

            //Sets the gamepad values to x, y, and z
            z = gamepad1.right_stick_x; //rotation
            y = gamepad1.left_stick_y; //side to side
            x = gamepad1.left_stick_x; //forward and backward

            //deadzones
            if(Math.abs(x) < .1) {
                x = 0;
            }
            if(Math.abs(y) < .1) {
                y = 0;
            }
            if(Math.abs(z) < .1) {
                z = 0;
            }

            //B driver = tilt glyph down
            if(gamepad1.b) {
                robot.trayL.setPosition(0);
                robot.trayR.setPosition(0);
            }

            //Y driver = tilt glyph up
            if(gamepad1.y) {
                robot.trayL.setPosition(0.5);
                robot.trayR.setPosition(0.5);
            }

            //Right trigger driver = Collect
            if(gamepad1.right_trigger>0.15) {
                robot.glyphCollectorL.setPower(0.8);
                robot.glyphCollectorR.setPower(0.8);
            }

            //Left trigger driver = Reverse collector
            if(gamepad1.left_trigger>0.15) {
                robot.glyphCollectorL.setPower(-0.8);
                robot.glyphCollectorR.setPower(-0.8);
            }
            robot.glyphCollectorL.setPower(0);
            robot.glyphCollectorR.setPower(0);

            //Gunner x = Open relic claw
            if(gamepad2.x) {
                robot.relicClaw.setPosition(0.5);
            }
            //Gunner b = Close relic claw
            if(gamepad2.b) {
                robot.relicClaw.setPosition(0);
            }
            //Gunner left joystick = glyft
            if(gamepad2.left_stick_y > 0.15) {
                robot.glyftL.setPower(0.8);
                robot.glyftR.setPower(0.8);
            }
            if(gamepad2.left_stick_y < -0.15) {
                robot.glyftL.setPower(-0.8);
                robot.glyftR.setPower(-0.8);
            }
            robot.glyftL.setPower(0);
            robot.glyftR.setPower(0);

            //Gunner right joystick = relic scorer
            if(gamepad2.right_stick_y > 0.15) {
                robot.relicScorer.setPower(0.8);
            }
            if(gamepad2.right_stick_y < -0.15) {
                robot.relicScorer.setPower(-0.8);
            }
            robot.relicScorer.setPower(0);

            //telemetry.addData("Calibrate value: ", calibrate);
            //C&P


            flPower = Range.scale((-x + y - z), -1, 1, -robot.MAX_MOTOR_SPEED, robot.MAX_MOTOR_SPEED);
            frPower = Range.scale((-x - y - z), -1, 1, -robot.MAX_MOTOR_SPEED, robot.MAX_MOTOR_SPEED);
            blPower = Range.scale((x + y - z), -1, 1, -robot.MAX_MOTOR_SPEED, robot.MAX_MOTOR_SPEED);
            brPower = Range.scale((x - y - z), -1, 1, -robot.MAX_MOTOR_SPEED, robot.MAX_MOTOR_SPEED);

            robot.fl.setPower(flPower);
            robot.fr.setPower(frPower);
            robot.bl.setPower(blPower);
            robot.br.setPower(brPower);

            telemetry.addData("FR Power", robot.fr.getPower());
            telemetry.addData("FL Power", robot.fl.getPower());
            telemetry.addData("BR Power", robot.br.getPower());
            telemetry.addData("BL Power", robot.bl.getPower());
            telemetry.addData("Old X on Gamepad: ", x);
            telemetry.addData("Old Y on Gamepad: ", y);
            telemetry.update();
        }
    }
}
