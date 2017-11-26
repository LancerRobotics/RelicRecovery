package org.firstinspires.ftc.teamcode;

/**
 * Created by dina.brustein on 9/27/2017.
 */
import android.media.midi.MidiDeviceInfo;

import com.qualcomm.robotcore.hardware.ColorSensor;
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
    public static double frPower, flPower, brPower, blPower;
    // Motors
    public DcMotor fl = null;
    public DcMotor fr = null;
    public DcMotor bl = null;
    public DcMotor br = null;
    public DcMotor relic = null;
    public DcMotor glyph = null;

    //Servos
    public Servo arm1 = null; //relic lifter right
    public Servo arm2 = null; //relic lifter left
    public Servo arm3 = null; //relic clamper
    public Servo arm4 = null; //glyph grabber left servo
    public Servo arm5 = null; //glyph grabber right servo
    public Servo jewel0 = null;
    public Servo jewel1 = null;

//    public Servo arm6 = null; //glyph grabber hook

    ColorSensor color;

    //Gyro
    public BNO055IMU imu = null;
    public Orientation angles;
    public Acceleration gravity;
    float theta;

    //HardwareMap
    HardwareMap hwMap = null;

    public static final double MAX_MOTOR_SPEED = .86;

    public static final double ARM_1_DOWN = 0.95;
    public static final double ARM_1_MIDDLE = 0.5;
    public static final double ARM_1_UP = 0.3;
    public static final double ARM_2_DOWN = 0.95;
    public static final double ARM_2_MIDDLE = 0.5;
    public static final double ARM_2_UP = 0.35;
    public static final double ARM_3_CLAMP = 0.9;
    public static final double ARM_3_UNCLAMP = 0.2;
    public static final double ARM_4_OPEN = 0.5;
    public static final double ARM_4_CLOSED = 0.7;
    public static final double ARM_5_OPEN = 0.5;
    public static final double ARM_5_CLOSED = 0.2;


    public HardwareMechanumRobot(){
    }

    public void init(HardwareMap ahwMap, boolean autonomous){
        hwMap = ahwMap;
        fr = hwMap.dcMotor.get("front_right");
        fl = hwMap.dcMotor.get("front_left");
        br = hwMap.dcMotor.get("back_right");
        bl = hwMap.dcMotor.get("back_left");
        relic = hwMap.dcMotor.get("relic");
        arm1 = hwMap.servo.get("glyph_top_right");
        arm2 = hwMap.servo.get("glyph_top_left");
        arm3 = hwMap.servo.get("jewel_hitter");
        arm4 = hwMap.servo.get("glyph_bottom_left");
        arm5 = hwMap.servo.get("glyph_bottom_right");
        jewel0 = hwMap.servo.get("jewel_lifter_0");
        jewel1 = hwMap.servo.get("jewel_lifter_1");

        //arm6 = hwMap.servo.get("glyph_holder");

        //color = hwMap.colorSensor.get("color_sensor");

        BNO055IMU.Parameters parameters = new BNO055IMU.Parameters();
        parameters.angleUnit = BNO055IMU.AngleUnit.DEGREES;
        parameters.accelUnit = BNO055IMU.AccelUnit.METERS_PERSEC_PERSEC;
        parameters.calibrationDataFile = "BNO055IMUCalibration.json";
        parameters.loggingEnabled = true;
        parameters.loggingTag = "IMU";
        parameters.accelerationIntegrationAlgorithm = new JustLoggingAccelerationIntegrator();

        imu = hwMap.get(BNO055IMU.class, "imu");
        imu.initialize(parameters);
        arm1.setPosition(ARM_1_DOWN);
        arm2.setPosition(ARM_2_DOWN);
        arm3.setPosition(ARM_3_UNCLAMP);
        arm4.setPosition(ARM_4_OPEN);
        arm5.setPosition(ARM_5_OPEN);
     //   arm6.setPosition(armINITIAL);

        arm1.scaleRange(0,1);
        arm2.scaleRange(0,1);
        arm3.scaleRange(0,1);
        arm4.scaleRange(0,1);
        arm5.scaleRange(0,1);

        fr.setDirection(DcMotor.Direction.REVERSE);
        br.setDirection(DcMotor.Direction.REVERSE);
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
            fr.setPower(-power);
            br.setPower(-power);
        }
        else {
            fr.setPower(power);
            br.setPower(power);
            fl.setPower(-power);
            bl.setPower(-power);
        }
    }
}