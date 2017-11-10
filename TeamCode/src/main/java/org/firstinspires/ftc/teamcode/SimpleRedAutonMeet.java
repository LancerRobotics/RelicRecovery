package org.firstinspires.ftc.teamcode;

import com.qualcomm.hardware.bosch.BNO055IMU;
import com.qualcomm.hardware.bosch.JustLoggingAccelerationIntegrator;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;

import org.firstinspires.ftc.robotcore.external.navigation.Acceleration;
import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.robotcore.external.navigation.AxesOrder;
import org.firstinspires.ftc.robotcore.external.navigation.AxesReference;
import org.firstinspires.ftc.robotcore.external.navigation.Orientation;

/**
 * Created by david.lin on 11/6/2017.
 */

public class SimpleRedAutonMeet extends LinearOpMode{
    HardwareMechanumRobot robot = new HardwareMechanumRobot();

    BNO055IMU imu;
    //imu values
    Orientation angles;
    Acceleration gravity;
    float theta;
    float calibrate;
    public void setup(){

    }

    public void runOpMode(){
        waitForStart();

        BNO055IMU.Parameters parameters = new BNO055IMU.Parameters();
        parameters.angleUnit           = BNO055IMU.AngleUnit.DEGREES;
        parameters.accelUnit           = BNO055IMU.AccelUnit.METERS_PERSEC_PERSEC;
        parameters.calibrationDataFile = "BNO055IMUCalibration.json"; // see the calibration sample opmode
        parameters.loggingEnabled      = true;
        parameters.loggingTag          = "IMU";
        parameters.accelerationIntegrationAlgorithm = new JustLoggingAccelerationIntegrator();

        robot.init(hardwareMap, true);


        imu = hardwareMap.get(BNO055IMU.class, "imu");
        imu.initialize(parameters);

        robot.arm4.setPosition(0);
        robot.arm5.setPosition(1);

        robot.setDrivePower(0.5, false);
        sleep(500);
        robot.setDrivePower(0, true);

        while(angles.firstAngle < 90){
            robot.turn(0.3, false);
        }

        robot.arm4.setPosition(1);
        robot.arm5.setPosition(0);

        robot.setDrivePower(0.5, false);
        sleep(500);
        robot.setDrivePower(0, true);

        robot.setDrivePower(0.3, true);
        sleep(250);
        robot.setDrivePower(0, true);

    }
}
