package org.firstinspires.ftc.teamcode;


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

import java.util.Locale;

//imu imports
//harware imports

/**
 * Created by jake.wiseberg on 9/29/2017.
 */

//note for later
//The IMU is on I2C port 0 so you would need to go into the robot configuration and name it. Then follow the "SensorBNO055IMU" example under external.samples in the SDK. Make sure you are on the newest version of sdk 3.1 .


@TeleOp(name="Omni Perspective Op", group="TeleOp")
public class TeleOpOmniWithPerspective extends OpMode {

    //values, using global values for faster runtime
    public DcMotor fr;
    public DcMotor fl;
    public DcMotor bl;
    public DcMotor br;
    private ElapsedTime runtime = new ElapsedTime();

    //values for motor
    double[] motorPwr = new double[4];
    double x,y,z;
    double joyX, joyY;

    //imu object
    BNO055IMU imu;
    //imu values
    Orientation angles;
    Acceleration gravity;
    float theta;

    @Override
    public void init() {
        telemetry.addData("Status", "In progress...");

        //FIRST comments yay...
        // Set up the parameters with which we will use our IMU. Note that integration
        // algorithm here just reports accelerations to the logcat log; it doesn't actually
        // provide positional information.
        BNO055IMU.Parameters parameters = new BNO055IMU.Parameters();
        parameters.angleUnit           = BNO055IMU.AngleUnit.DEGREES;
        parameters.accelUnit           = BNO055IMU.AccelUnit.METERS_PERSEC_PERSEC;
        parameters.calibrationDataFile = "BNO055IMUCalibration.json"; // see the calibration sample opmode
        parameters.loggingEnabled      = true;
        parameters.loggingTag          = "IMU";
        parameters.accelerationIntegrationAlgorithm = new JustLoggingAccelerationIntegrator();

        //set imu to hardware and initialize
        imu = hardwareMap.get(BNO055IMU.class, "imu");
        imu.initialize(parameters);

        //set motors to hardware
        fr = hardwareMap.dcMotor.get("front_right");
        fl = hardwareMap.dcMotor.get("front_left");
        bl = hardwareMap.dcMotor.get("back_left");
        br = hardwareMap.dcMotor.get("back_right");

        telemetry.addData("Status", "Initialized");
    }

    @Override
    public void start() {
        //timer
        runtime.reset();
        //imu start
        imu.startAccelerationIntegration(new Position(), new Velocity(), 1000);
    }

    @Override
    public void loop() {
        angles   = imu.getAngularOrientation(AxesReference.INTRINSIC, AxesOrder.XYZ, AngleUnit.RADIANS);
        theta = angles.firstAngle;

        //getting joystick values and rotate axis for perspective drive
        joyY = gamepad1.left_stick_y; //left joystick y axis
        joyX = gamepad1.left_stick_x; //left joystick x axis
        z = gamepad1.right_stick_x; //right joystick x axis (for turning)

        y = joyX*Math.cos(theta) - joyY*Math.sin(theta);
        x = joyX*Math.sin(theta) - joyY*Math.cos(theta);

        motorPwr[0] = Range.clip(-y+x-z, -1.0, 1.0); //front right
        motorPwr[1] = Range.clip(y+x-z, -1.0, 1.0); //front left
        motorPwr[2] = Range.clip(-y-x-z, -1.0, 1.0); //back right
        motorPwr[3] = Range.clip(y-x-z, -1.0, 1.0); //back left

        fr.setPower(motorPwr[0]);
        fl.setPower(motorPwr[0]);
        br.setPower(motorPwr[0]);
        bl.setPower(motorPwr[0]);

        telemetry.addData("Status", "Run Time: " + runtime.toString());
        telemetry.addData("Angle", theta);
    }
}
