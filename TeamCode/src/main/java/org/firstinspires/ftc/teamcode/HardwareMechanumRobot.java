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
    public DcMotor glyph = null;
    public DcMotor relic = null;
    public DcMotor extender = null;

    //Servos
    public Servo arm0 = null;
    public Servo arm1 = null; //glyph top right
    public Servo arm2 = null; //glyph top left
    public Servo arm4 = null; //glyph bottom left
    public Servo arm5 = null; //glyph bottom right

    public CRServo jewel0 = null; //servo jewel0 and jewel1 bring the arm down
    public Servo jewel1 = null;
    //This is a Continuous Rotation servo rn
    public CRServo jewel_hitter = null;
    //public Servo arm6 = null; //glyph grabber hook

    //David's servo test
    public CRServo vexMotor = null;

    //Color Sensor
    public ColorSensor color_sensor = null;

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
//        glyph = hwMap.dcMotor.get("glyph");
//        relic = hwMap.dcMotor.get("relic");
//        extender = hwMap.dcMotor.get("relic_extender");
/*
        arm0 = hwMap.servo.get("relic_clamper");
        arm1 = hwMap.servo.get("glyph_top_right");
        arm2 = hwMap.servo.get("glyph_top_left");
        arm4 = hwMap.servo.get("glyph_bottom_left");
        arm5 = hwMap.servo.get("glyph_bottom_right");

        jewel0 = hwMap.crservo.get("jewel_extender_1");
        jewel1 = hwMap.servo.get("jewel_extender_2");
        jewel_hitter = hwMap.crservo.get("jewel_hitter");

        color_sensor = hwMap.colorSensor.get("color_sensor");
        */
        //arm6 = hwMap.servo.get("glyph_holder");

        fl.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        fr.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        bl.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        br.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);

        //color = hwMap.colorSensor.get("color_sensor");
/*
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
        //arm6.setPosition(armINITIAL);

        arm0.scaleRange(0,1);
        arm1.scaleRange(0,1);
        arm2.scaleRange(0,1);
        arm4.scaleRange(0,1);
        arm5.scaleRange(0,1);

        jewel1.scaleRange(0,1);
        //jewel_hitter.scaleRange(0,1);
*/
        fr.setDirection(DcMotor.Direction.REVERSE);
        br.setDirection(DcMotor.Direction.REVERSE);
/*
        //always set the relic clamper down
        arm0.setPosition(ARM_0_DOWN);
        //set the init positions of servos
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
*/
    }

    //To test David's servo jk
    public void init2(HardwareMap ahwMap, boolean autonomous) {
        hwMap = ahwMap;
        vexMotor = hwMap.crservo.get("vexMotor");
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

    public boolean resetEncoders() {
        fl.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        return true;
    }

    public boolean setUpEncoders() {
        fl.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        return true;
    }

    public boolean runEncoders() {
        fl.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        return true;
    }

    public boolean powerAll(double power) {
        fl.setPower(power);
        return true;
    }

    public boolean checkEncoders() {
        return fl.isBusy();
    }

    public boolean encoderPlz (int ticks, double power, LinearOpMode opMode) {
        resetEncoders();
        setUpEncoders();
        int target = fl.getCurrentPosition() + (ticks);
        fl.setTargetPosition((int)target);
        runEncoders();
        powerAll(power);
        while(checkEncoders()) {
            opMode.telemetry.addData("Current tick: " , fl.getCurrentPosition());
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
//        while(fl.getCurrentPosition() <= fl.getTargetPosition()) {
//            setDrivePower(0.2, false);
//            opMode.telemetry.addData("Current position: ", fl.getCurrentPosition());
//            opMode.telemetry.update();
//        }
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
        /*
        fl.setTargetPosition(targetTick);
        fl.setPower(0.3);
        opMode.telemetry.addData("TargetTick:" , targetTick);
        opMode.telemetry.addData("CurrentPos: ", fl.getCurrentPosition());
        opMode.telemetry.update();
        */
        if (inches > 0) {
            setDrivePower(.35, false);
            opMode.telemetry.addData("Encoder Pos: ", fl.getCurrentPosition());
            opMode.telemetry.addData("Target Pos: ", targetTick);
            opMode.telemetry.update();
            while (fl.getCurrentPosition() < targetTick && opMode.opModeIsActive() && !opMode.isStopRequested()) {
                opMode.telemetry.addData("Encoder Pos: ", fl.getCurrentPosition());
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