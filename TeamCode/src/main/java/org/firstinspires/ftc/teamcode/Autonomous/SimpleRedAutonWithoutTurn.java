package org.firstinspires.ftc.teamcode.Autonomous;

import com.qualcomm.hardware.bosch.BNO055IMU;
import com.qualcomm.hardware.bosch.JustLoggingAccelerationIntegrator;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;

import org.firstinspires.ftc.robotcore.external.navigation.Acceleration;
import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.robotcore.external.navigation.AxesOrder;
import org.firstinspires.ftc.robotcore.external.navigation.AxesReference;
import org.firstinspires.ftc.robotcore.external.navigation.Orientation;
import org.firstinspires.ftc.teamcode.HardwareMechanumRobot;

/**
 * Created by dina.brustein on 11/11/2017.
 */

//@Autonomous
public class SimpleRedAutonWithoutTurn extends LinearOpMode {
    HardwareMechanumRobot robot = new HardwareMechanumRobot();

    BNO055IMU imu;
    //imu values
    Orientation angles;
    Acceleration gravity;
    float theta;
    float calibrate;

    public void runOpMode(){

        robot.init(hardwareMap, true);

        BNO055IMU.Parameters parameters = new BNO055IMU.Parameters();
        parameters.angleUnit           = BNO055IMU.AngleUnit.DEGREES;
        parameters.accelUnit           = BNO055IMU.AccelUnit.METERS_PERSEC_PERSEC;
        parameters.calibrationDataFile = "BNO055IMUCalibration.json"; // see the calibration sample opmode
        parameters.loggingEnabled      = true;
        parameters.loggingTag          = "IMU";
        parameters.accelerationIntegrationAlgorithm = new JustLoggingAccelerationIntegrator();

        imu = hardwareMap.get(BNO055IMU.class, "imu");
        imu.initialize(parameters);

        angles = imu.getAngularOrientation(AxesReference.INTRINSIC, AxesOrder.XYZ, AngleUnit.DEGREES);

        waitForStart();

        robot.arm4.setPosition(robot.ARM_4_CLOSED_AUTON);
        robot.arm5.setPosition(robot.ARM_5_CLOSED_AUTON);

        sleep(500);

        robot.setDrivePower(0.6, false);
        sleep(1000);
        robot.setDrivePower(0, true);

        sleep(500);
        robot.arm4.setPosition(robot.ARM_4_CLOSED_AUTON);
        robot.arm5.setPosition(robot.ARM_5_CLOSED_AUTON);
        sleep(500);

        robot.setDrivePower(0.5, true);
        sleep(250);
        robot.setDrivePower(0, true);

    }
}
