package org.firstinspires.ftc.teamcode;

import com.qualcomm.hardware.bosch.BNO055IMU;
import com.qualcomm.hardware.bosch.JustLoggingAccelerationIntegrator;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;

import org.firstinspires.ftc.robotcore.external.navigation.Acceleration;
import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.robotcore.external.navigation.AxesOrder;
import org.firstinspires.ftc.robotcore.external.navigation.AxesReference;
import org.firstinspires.ftc.robotcore.external.navigation.Orientation;

/**
 * Created by david.lin on 11/6/2017.
 */

@Autonomous

public class SimpleRedAutonMeet extends LinearOpMode{
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

        robot.arm4.setPosition(0.5);
        robot.arm5.setPosition(0.5);

        robot.setDrivePower(0.5, false);
        sleep(1000);
        robot.setDrivePower(0, true);

        while(angles.firstAngle > -2 && opModeIsActive() && !(isStopRequested())){
            robot.turn(0.25, false);
            angles = imu.getAngularOrientation(AxesReference.INTRINSIC, AxesOrder.XYZ, AngleUnit.DEGREES);

            telemetry.addData("Gyro value", angles.firstAngle);
            telemetry.update();
        }

        robot.setDrivePower(0, false);

        robot.setDrivePower(0.3, false);
        sleep(1000);
        robot.setDrivePower(0, false);

        sleep(500);

        robot.setDrivePower(0.3, true);
        sleep(250);
        robot.setDrivePower(0, true);

    }
}
