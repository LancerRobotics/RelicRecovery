package org.firstinspires.ftc.teamcode;

/**
 * Created by dina.brustein on 9/27/2017.
 */
import com.qualcomm.hardware.bosch.BNO055IMU;
import com.qualcomm.hardware.bosch.JustLoggingAccelerationIntegrator;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.CRServo;
import com.qualcomm.robotcore.hardware.ColorSensor;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.Servo;

import org.firstinspires.ftc.robotcontroller.external.samples.ConceptVuforiaNavigation;
import org.firstinspires.ftc.robotcore.external.ClassFactory;
import org.firstinspires.ftc.robotcore.external.matrices.OpenGLMatrix;
import org.firstinspires.ftc.robotcore.external.matrices.VectorF;
import org.firstinspires.ftc.robotcore.external.navigation.Acceleration;
import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.robotcore.external.navigation.AxesOrder;
import org.firstinspires.ftc.robotcore.external.navigation.AxesReference;
import org.firstinspires.ftc.robotcore.external.navigation.Orientation;
import org.firstinspires.ftc.robotcore.external.navigation.RelicRecoveryVuMark;
import org.firstinspires.ftc.robotcore.external.navigation.VuMarkInstanceId;
import org.firstinspires.ftc.robotcore.external.navigation.VuforiaLocalizer;
import org.firstinspires.ftc.robotcore.external.navigation.VuforiaTrackable;
import org.firstinspires.ftc.robotcore.external.navigation.VuforiaTrackableDefaultListener;
import org.firstinspires.ftc.robotcore.external.navigation.VuforiaTrackables;

public class HardwareMechanumRobot {
    public static double frPower, flPower, brPower, blPower;
    // Motors
    public DcMotor fl = null;
    public DcMotor fr = null;
    public DcMotor bl = null;
    public DcMotor br = null;
    public DcMotor glyftL = null;
    public DcMotor glyftR = null;
    public DcMotor relicScorer = null;

    //Servos
    public CRServo glyphPusher = null;
    public Servo gcSecure = null;
    public Servo autonGlyphL = null;
    public Servo autonGlyphR = null;
    public Servo relicTurner = null;
    public Servo relicClaw = null;
    public Servo jewelLift = null;
    public Servo jewelHitter = null;
    public Servo trayL = null;
    public Servo trayR = null;
    public CRServo glyphCollectorR = null;
    public CRServo glyphCollectorL = null;

    //Color Sensor
    public ColorSensor color = null;

    //Gyro
    public BNO055IMU imu = null;
    public Orientation angles;
    public Acceleration gravity;
    float theta;

    //Vuforia
    public static final String TAG = "Vuforia VuMark Sample";

    //HardwareMap
    HardwareMap hwMap = null;

    public static final double MAX_MOTOR_SPEED = .86;

    public static final double ARM_0_UP = 0.3;
    public static final double ARM_0_DOWN = 0.95;
    public static final double ARM_1_OPEN = 0.35;
    public static final double ARM_1_CLOSED = 0.15;
    public static final double ARM_2_OPEN = 0.5;
    public static final double ARM_2_CLOSED = 0.7;
    public static final double ARM_4_OPEN = 0.60;
    public static final double ARM_4_CLOSED_AUTON = 0.7;
    public static final double ARM_4_CLOSED = 0.80;
    public static final double ARM_5_OPEN = 0.45;
    public static final double ARM_5_CLOSED = 0.3;
    public static final double ARM_5_CLOSED_AUTON = 0.35;

    public HardwareMechanumRobot() {
    }

    public void init(HardwareMap ahwMap, boolean autonomous) {
        hwMap = ahwMap;
        fr = hwMap.dcMotor.get("front_right");
        fl = hwMap.dcMotor.get("front_left");
        br = hwMap.dcMotor.get("back_right");
        bl = hwMap.dcMotor.get("back_left");
        glyftL = hwMap.dcMotor.get("glyft_left");
        glyftR = hwMap.dcMotor.get("glyft_right");
        relicScorer = hwMap.dcMotor.get("relic_scorer");

        gcSecure = hwMap.servo.get("glyph_collector_securer");
        glyphPusher = hwMap.crservo.get("glyph_pusher");
        autonGlyphL = hwMap.servo.get("auton_glyph_left");
        autonGlyphR = hwMap.servo.get("auton_glyph_right");
        relicTurner = hwMap.servo.get("relic_turner");
        relicClaw = hwMap.servo.get("relic_claw");
        jewelHitter = hwMap.servo.get("jewel_hitter");
        jewelLift = hwMap.servo.get("jewel_lift");
        trayL = hwMap.servo.get("tray_left");
        trayR = hwMap.servo.get("tray_right");
        glyphCollectorL = hwMap.crservo.get("glyph_collector_left");
        glyphCollectorR = hwMap.crservo.get("glyph_collector_right");

        //Scaling
        gcSecure.scaleRange(0,1);
        autonGlyphL.scaleRange(0,1);
        autonGlyphR.scaleRange(0,1);
        relicTurner.scaleRange(0,1);
        relicClaw.scaleRange(0,1);
        jewelHitter.scaleRange(0,1);
        jewelLift.scaleRange(0,1);
        trayL.scaleRange(0,1);
        trayR.scaleRange(0,1);

        //Initialize Positions
        gcSecure.setPosition(0.5);
        autonGlyphL.setPosition(0);
        autonGlyphR.setPosition(1);

        color = hwMap.colorSensor.get("color_sensor");

        fl.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        fr.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        bl.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        br.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);

        if(autonomous) {
            bl.setDirection(DcMotor.Direction.REVERSE);
            fl.setDirection(DcMotor.Direction.REVERSE);
        }
    }

    //To test David's servo jk
    public void init2(HardwareMap ahwMap, boolean autonomous) {
        hwMap = ahwMap;
    }

    public void strafe(double power, boolean left) {
        //fixed strafe to these values: tinyurl.com/mecanum
        if (!left) {
            fl.setPower(power);
            fr.setPower(-power);
            bl.setPower(-power);
            br.setPower(power);
        } else {
            fl.setPower(-power);
            fr.setPower(power);
            bl.setPower(power);
            br.setPower(-power);
        }
    }

    public void setDrivePower(double power, boolean backwards) {
        if (!backwards) {
            fl.setPower(power);
            fr.setPower(power);
            bl.setPower(power);
            br.setPower(power);
        } else {
            fl.setPower(-power);
            fr.setPower(-power);
            bl.setPower(-power);
            br.setPower(-power);
        }
    }

    public void turn(double power, boolean left) {

        if (!left) {
            fl.setPower(power);
            bl.setPower(power);
            fr.setPower(-power);
            br.setPower(-power);
        } else {
            fr.setPower(power);
            br.setPower(power);
            fl.setPower(-power);
            bl.setPower(-power);
        }
    }

    public void gyroTurn(double power, double angle) {

    }

    public boolean resetEncoders() {
        br.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        return true;
    }

    public boolean setUpEncoders() {
        br.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        return true;
    }

    public boolean runEncoders() {
        br.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        return true;
    }

    public boolean powerAll(double power) {
        fl.setPower(power);
        fr.setPower(power);
        bl.setPower(power);
        br.setPower(power);
        return true;
    }

    public void encoderDrive(int ticks, double power, LinearOpMode opMode) {
        br.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        br.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        int target = br.getCurrentPosition() + (ticks);
        br.setTargetPosition((int)target);
        br.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        fl.setPower(power);
        fr.setPower(power);
        bl.setPower(power);
        br.setPower(power);
        while(fl.isBusy()) {
            opMode.telemetry.addData("Current tick: " , br.getCurrentPosition());
            opMode.telemetry.update();
        }
        fl.setPower(0);
        fr.setPower(0);
        bl.setPower(0);
        br.setPower(0);
    }

    public boolean checkEncoders() {
        return fl.isBusy();
    }

    public boolean encoderPlz (double ticks, double power, LinearOpMode opMode) {
        resetEncoders();
        setUpEncoders();
        int target = br.getCurrentPosition() + (int)(ticks);
        br.setTargetPosition((int)target);
        runEncoders();
        powerAll(power);
        while(checkEncoders()) {
            opMode.telemetry.addData("Current tick: " , br.getCurrentPosition());
            opMode.telemetry.update();
        }
        powerAll(0);
        return setUpEncoders();
        /*
        fl.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        fl.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        int target = fl.getCurrentPosition() + (ticks);
        fl.setTargetPosition((int)target);
        fl.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        fl.setPower(power);
        */
        /*
        while(fl.isBusy()) {
            opMode.telemetry.addData("Current tick: " , fl.getCurrentPosition());
            opMode.telemetry.update();
        }
        setDrivePower(0, false);
        */
    }

    public void encoderDrive(double inches, double power, LinearOpMode opMode) {
        fl.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        fl.setTargetPosition((int)(inches * 1140.0 / (4.0 * Math.PI * 2.0)));
        fl.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        setDrivePower(power, false);
        opMode.telemetry.addLine("Check to see if telemetry works ");
        opMode.telemetry.update();
        opMode.sleep(1000);
        while(fl.isBusy()) {
            opMode.telemetry.addData("Current tick: " , fl.getCurrentPosition());
            opMode.telemetry.update();
        }
        while(fl.getCurrentPosition() <= fl.getTargetPosition() && opMode.opModeIsActive()) {
            setDrivePower(0.2, false);
            opMode.telemetry.addData("Current position: ", fl.getCurrentPosition());
            opMode.telemetry.update();
        }
        setDrivePower(0, false);
    }

    public void encoderTurn(double inches, double power, boolean left, LinearOpMode opMode) {
        fl.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        fl.setTargetPosition((int)(inches * 1140.0 / (4.0 * Math.PI * 2.0)));
        fl.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        if(left) {
            turn(power, true);
            while(fl.isBusy()) {
                opMode.telemetry.addData("Current tick: ", fl.getCurrentPosition());
                opMode.telemetry.update();
            }
            setDrivePower(0, false);
        }
        else {
            turn(power, false);
            while(fl.isBusy()) {
                opMode.telemetry.addData("Current tick: ", fl.getCurrentPosition());
            }
            setDrivePower(0, false);
        }
    }
    public void pleaseEncodedMove(double inches, LinearOpMode opMode) {
        fl.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);

        while (fl.getCurrentPosition() != 0 && opMode.opModeIsActive()) {
            opMode.telemetry.addLine("Resetting Encoders");
            opMode.telemetry.update();
        }

        fl.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        opMode.telemetry.addLine("Beginning to Move");
        opMode.sleep(500);
        int targetTick = (int) (inches * 1140.0 / (4.0 * Math.PI * 2.0));

        if (inches > 0) {
            fr.setPower(0.2);
            opMode.telemetry.addData("Encoder Pos: ", fr.getCurrentPosition());
            opMode.telemetry.addData("Target Pos: ", targetTick);
            opMode.telemetry.update();
            while (fr.getCurrentPosition() < targetTick && opMode.opModeIsActive() && !opMode.isStopRequested()){
                opMode.telemetry.addData("Encoder Pos: ", fr.getCurrentPosition());
                opMode.telemetry.addData("Target Pos: ", targetTick);
                opMode.telemetry.update();
            }
            setDrivePower(0, false);
        } else {
            setDrivePower(.35, true);
            while (fl.getCurrentPosition() > targetTick && opMode.opModeIsActive() && !opMode.isStopRequested()) {
                opMode.telemetry.addData("Encoder Pos: ", fl.getCurrentPosition());
                opMode.telemetry.addData("Target Pos: ", targetTick);
                opMode.telemetry.update();
            }
            setDrivePower(0, true);
        }
    }
}