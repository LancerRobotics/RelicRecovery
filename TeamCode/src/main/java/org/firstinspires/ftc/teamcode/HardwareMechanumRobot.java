package org.firstinspires.ftc.teamcode;

/**
 * Created by dina.brustein on 9/27/2017.
 */
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.hardware.bosch.BNO055IMU;
import com.qualcomm.hardware.bosch.JustLoggingAccelerationIntegrator;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;

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

public class HardwareMechanumRobot {

    // Motors
    public DcMotor fl = null;
    public DcMotor fr = null;
    public DcMotor bl = null;
    public DcMotor br = null;

    //Servos
    public Servo arm1 = null; //relic lifter right
    public Servo arm2 = null; //relic lifter left
    public Servo arm3 = null; //relic clamper
    public Servo arm4 = null; //glyph grabber left servo
    public Servo arm5 = null; //glyph grabber right servo
  //  public Servo arm6 = null; //glyph grabber hook

    //Gyro
    public BNO055IMU imu = null;
    public Orientation angles;
    public Acceleration gravity;
    float theta;

    //HardwareMap
    HardwareMap hwMap = null;

    public static final double MAX_MOTOR_SPEED = .86;
    public static final double armUP = .25;
    public static final double armDOWN = 1;
    public static final double armINITIAL = .6;
    public static final double armDROP = 0.4;
    public static final double armHALFWAY = 0.75;
    public static final double ARM_4_POSITION = 0.4;
    public static final double ARM_3_POSITION = 0.4;

    public HardwareMechanumRobot(){
    }

    public void init(HardwareMap ahwMap, boolean autonomous){
        hwMap = ahwMap;
        fr = hwMap.dcMotor.get("front_right");
        fl = hwMap.dcMotor.get("front_left");
        br = hwMap.dcMotor.get("back_right");
        bl = hwMap.dcMotor.get("back_left");
        arm1 = hwMap.servo.get("relic_lifter_right");
        arm2 = hwMap.servo.get("relic_lifter_left");
        arm3 = hwMap.servo.get("relic_clamper");
        arm4 = hwMap.servo.get("glyph_left");
        arm5 = hwMap.servo.get("glyph_right");
   //     arm6 = hwMap.servo.get("glyph_holder");

        BNO055IMU.Parameters parameters = new BNO055IMU.Parameters();
        parameters.angleUnit = BNO055IMU.AngleUnit.DEGREES;
        parameters.accelUnit = BNO055IMU.AccelUnit.METERS_PERSEC_PERSEC;
        parameters.calibrationDataFile = "BNO055IMUCalibration.json";
        parameters.loggingEnabled = true;
        parameters.loggingTag = "IMU";
        parameters.accelerationIntegrationAlgorithm = new JustLoggingAccelerationIntegrator();

        imu = hwMap.get(BNO055IMU.class, "imu");
        imu.initialize(parameters);
        arm1.setPosition(armINITIAL);
        arm2.setPosition(armINITIAL);
    //    arm3.setPosition(armINITIAL);
        arm4.setPosition(0.4);
        arm5.setPosition(0.6);
    //    arm6.setPosition(armINITIAL);
    }



    public void strafe(double power, boolean left){
        //fixed strafe to these values: tinyurl.com/mecanum
        if(!left) {
            fl.setPower(power);
            fr.setPower(-power);
            bl.setPower(-power);
            br.setPower(power);
        }

        else {
            fl.setPower(-power);
            fr.setPower(power);
            bl.setPower(power);
            br.setPower(-power);
        }
    }

    public void setDrivePower(double power, boolean backwards){
        if(!backwards){
            fl.setPower(power);
            fr.setPower(power);
            bl.setPower(power);
            br.setPower(power);
        }
        else{
            fl.setPower(-power);
            fr.setPower(-power);
            bl.setPower(-power);
            br.setPower(-power);
        }
    }

    public void turn(double power, boolean left){

        if(!left){
            fl.setPower(power);
            bl.setPower(power);
        }
        else {
            fr.setPower(power);
            br.setPower(power);
        }
    }
}