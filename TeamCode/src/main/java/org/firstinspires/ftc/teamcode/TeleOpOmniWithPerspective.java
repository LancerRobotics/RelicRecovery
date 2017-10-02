package org.firstinspires.ftc.teamcode;


import com.qualcomm.hardware.bosch.BNO055IMU;
import com.qualcomm.hardware.bosch.JustLoggingAccelerationIntegrator;
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


@TeleOp(name="Template: Linear OpMode", group="Linear Opmode")
public class TeleOpOmniWithPerspective extends OpMode {

    //values, using global values for faster runtime
    HardwareMap hMap;
    public DcMotor fr;
    public DcMotor fl;
    public DcMotor bl;
    public DcMotor br;
    private ElapsedTime runtime = new ElapsedTime();

    //values for motor
    double[] motorPwr = new double[4];
    double x,y,z;

    //imu object
    BNO055IMU imu;
    //imu values
    Orientation angles;
    Acceleration gravity;

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
        fr = hMap.dcMotor.get("front_right");
        fl = hMap.dcMotor.get("front_left");
        bl = hMap.dcMotor.get("back_left");
        br = hMap.dcMotor.get("back_right");

        //imu hardware
        imu = hardwareMap.get(BNO055IMU.class, "imu");
        imu.initialize(parameters);

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
        //get imu data TODO this might make things slow so I probs shud fix this later
        angles   = imu.getAngularOrientation(AxesReference.INTRINSIC, AxesOrder.ZYX, AngleUnit.DEGREES); //TODO figure out whether angles should be in degrees or radians
        gravity  = imu.getGravity(); //there has to be a use for it if not I'll delete it for faster runtime


        //getting joystick values
        y = gamepad1.left_stick_y; //left joystick y axis
        x = gamepad1.left_stick_x; //left joystick x axis
        z = gamepad1.right_stick_x; //right joystick x axis

        //calculating the motor power TODO incorperate perspective drive
        motorPwr[0] = Range.clip(-y+x-z, -1.0, 1.0); //front right
        motorPwr[1] = Range.clip(y+x-z, -1.0, 1.0); //front left
        motorPwr[2] = Range.clip(-y-x-z, -1.0, 1.0); //back right
        motorPwr[3] = Range.clip(y-x-z, -1.0, 1.0); //back left

        //setting the motor powers TODO incorperate perspective drive
        fr.setPower(motorPwr[0]);
        fl.setPower(motorPwr[0]);
        br.setPower(motorPwr[0]);
        bl.setPower(motorPwr[0]);

        telemetry.addData("Status", "Run Time: " + runtime.toString());
    }

    //honestly we probs won't need these but if we need it for testing the imu w telementry it's here I guess
    String formatAngle(AngleUnit angleUnit, double angle) {
        return formatDegrees(AngleUnit.DEGREES.fromUnit(angleUnit, angle));
    }

    String formatDegrees(double degrees){
        return String.format(Locale.getDefault(), "%.1f", AngleUnit.DEGREES.normalize(degrees));
    }
}
