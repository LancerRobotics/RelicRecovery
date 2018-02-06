package org.firstinspires.ftc.teamcode.Autonomous;

/**
 * Created by yibin.long on 2/4/2018.
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
import org.firstinspires.ftc.robotcore.external.navigation.VuMarkInstanceId;
import org.firstinspires.ftc.robotcore.external.navigation.VuforiaLocalizer;
import org.firstinspires.ftc.robotcore.external.navigation.VuforiaTrackable;
import org.firstinspires.ftc.robotcore.external.navigation.VuforiaTrackableDefaultListener;
import org.firstinspires.ftc.robotcore.external.navigation.VuforiaTrackables;
import org.firstinspires.ftc.teamcode.HardwareMechanumRobot;
import org.firstinspires.ftc.teamcode.Vuforia;

@Autonomous (name = "Blue Auton with Gyro TEST- USE THIS", group = "Linear OpMode")
//@Disabled
public class BlueAutonGyroTest extends LinearOpMode{
    HardwareMechanumRobot robot = new HardwareMechanumRobot();
    BNO055IMU imu;

    Orientation angles;
    Acceleration gravity;

    public void setup(){
    }

    public void runOpMode(){
        BNO055IMU.Parameters parameters = new BNO055IMU.Parameters();
        parameters.angleUnit           = BNO055IMU.AngleUnit.DEGREES;
        parameters.accelUnit           = BNO055IMU.AccelUnit.METERS_PERSEC_PERSEC;
        parameters.calibrationDataFile = "BNO055IMUCalibration.json"; // see the calibration sample opmode
        parameters.loggingEnabled      = true;
        parameters.loggingTag          = "IMU";
        parameters.accelerationIntegrationAlgorithm = new JustLoggingAccelerationIntegrator();

        // Retrieve and initialize the IMU. We expect the IMU to be attached to an I2C port
        // on a Core Device Interface Module, configured to be a sensor of type "AdaFruit IMU",
        // and named "imu".
        imu = hardwareMap.get(BNO055IMU.class, "imu");
        imu.initialize(parameters);
        //wait
        //uncomment this if it breaks stuff waitForStart();

        // Start the logging of measured acceleration
        imu.startAccelerationIntegration(new Position(), new Velocity(), 1000);

        robot.init(hardwareMap, true);

        waitForStart();

        //init the angles value to 0
        angles = imu.getAngularOrientation(AxesReference.INTRINSIC, AxesOrder.ZYX, AngleUnit.DEGREES);

        //Move to and face cryptobox
        sleep(500);
        //move forwards
        robot.setDrivePower(0.5, false);
        sleep(1000);
        robot.setDrivePower(0, true);
        //turn left, change the angle value until it goes to right, mid, or left box
        while(angles.firstAngle < 135 && opModeIsActive()){
            robot.turn(.2, true);
            angles = imu.getAngularOrientation(AxesReference.INTRINSIC, AxesOrder.ZYX, AngleUnit.DEGREES);
        }
        robot.turn(0, false);

        //Move into the cryptobox
        robot.setDrivePower(0, false);
        //move forwards
        robot.setDrivePower(0.5, false);
        sleep(1000);
        robot.setDrivePower(0, false);

        //Move backwards from cryptobox
        sleep(500);
        robot.arm4.setPosition(.40);
        robot.arm5.setPosition(.60);
        sleep(1500);
        //move backwards
        robot.setDrivePower(0.5, true);
        sleep(250);
        robot.setDrivePower(0, true);

        /*while(angles.firstAngle < 90 && opModeIsActive()){
            robot.turn(0.2, true);
            angles = imu.getAngularOrientation(AxesReference.INTRINSIC, AxesOrder.ZYX, AngleUnit.DEGREES);
            telemetry.addData("Gyro angle: ", angles.firstAngle);
            telemetry.update();
        }*/
        sleep(500);

        robot.setDrivePower(0, false);

    }
}

