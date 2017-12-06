package org.firstinspires.ftc.teamcode;

/**
 * Created by dina.brustein on 9/27/2017.
 */
import com.qualcomm.hardware.bosch.BNO055IMU;
import com.qualcomm.hardware.bosch.JustLoggingAccelerationIntegrator;
import com.qualcomm.robotcore.hardware.ColorSensor;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.Servo;

import org.firstinspires.ftc.robotcore.external.navigation.Acceleration;
import org.firstinspires.ftc.robotcore.external.navigation.Orientation;

public class HardwareMechanumRobot {
    public static double frPower, flPower, brPower, blPower;
    // Motors
    public DcMotor fl = null;
    public DcMotor fr = null;
    public DcMotor bl = null;
    public DcMotor br = null;
    public DcMotor glyph = null;
    public DcMotor relic = null;
    public DcMotor extender = null;

    //Servos
    public Servo arm0 = null;
    public Servo arm1 = null; //glyph top right
    public Servo arm2 = null; //glyph top left
    public Servo arm4 = null; //glyph bottom left
    public Servo arm5 = null; //glyph bottom right

//    public Servo arm6 = null; //glyph grabber hook

    //Gyro
    public BNO055IMU imu = null;
    public Orientation angles;
    public Acceleration gravity;
    float theta;

    //HardwareMap
    HardwareMap hwMap = null;

    public static final double MAX_MOTOR_SPEED = .86;

    public static final double ARM_0_UP = 0.2;
    public static final double ARM_0_DOWN = 0.85;
    public static final double ARM_1_OPEN = 0.3;
    public static final double ARM_1_CLOSED = 0.15;
    public static final double ARM_2_OPEN = 0.55;
    public static final double ARM_2_CLOSED = 0.7;
    public static final double ARM_4_OPEN = 0.55;
    public static final double ARM_4_CLOSED_AUTON = 0.7;
    public static final double ARM_4_CLOSED = 0.75;
    public static final double ARM_5_OPEN = 0.7;
    public static final double ARM_5_CLOSED = 0.3;
    public static final double ARM_5_CLOSED_AUTON = 0.35;

    public HardwareMechanumRobot(){
    }

    public void init(HardwareMap ahwMap, boolean autonomous){
        hwMap = ahwMap;
        fr = hwMap.dcMotor.get("front_right");
        fl = hwMap.dcMotor.get("front_left");
        br = hwMap.dcMotor.get("back_right");
        bl = hwMap.dcMotor.get("back_left");
        glyph = hwMap.dcMotor.get("glyph");
        relic = hwMap.dcMotor.get("relic");
        extender = hwMap.dcMotor.get("relic_extender");

        arm0 = hwMap.servo.get("relic_clamper");
        arm1 = hwMap.servo.get("glyph_top_right");
        arm2 = hwMap.servo.get("glyph_top_left");
        arm4 = hwMap.servo.get("glyph_bottom_left");
        arm5 = hwMap.servo.get("glyph_bottom_right");

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
        arm4.setPosition(ARM_4_OPEN);
        arm5.setPosition(ARM_5_OPEN);
     //   arm6.setPosition(armINITIAL);

        arm0.scaleRange(0, 1);
        arm1.scaleRange(0,1);
        arm2.scaleRange(0,1);
        arm4.scaleRange(0,1);
        arm5.scaleRange(0,1);

        fr.setDirection(DcMotor.Direction.REVERSE);
        br.setDirection(DcMotor.Direction.REVERSE);

        if(!autonomous){
            arm1.setPosition(ARM_1_OPEN);
            arm2.setPosition(ARM_2_OPEN);
            arm4.setPosition(ARM_4_CLOSED);
            arm5.setPosition(ARM_5_CLOSED);
        }

        else {
            arm4.setPosition(ARM_4_OPEN);
            arm5.setPosition(ARM_5_OPEN);
        }

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