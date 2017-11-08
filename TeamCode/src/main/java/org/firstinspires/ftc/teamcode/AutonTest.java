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

        robot.arm4.setPosition(robot.ARM_4_POSITION);
        robot.arm3.setPosition(robot.ARM_3_POSITION);

        robot.init(hardwareMap, true);

        robot.setDrivePower(0.86, false);
        sleep(500);
        robot.setDrivePower(0, true);

        BNO055IMU.Parameters parameters = new BNO055IMU.Parameters();
        parameters.angleUnit           = BNO055IMU.AngleUnit.DEGREES;
        parameters.accelUnit           = BNO055IMU.AccelUnit.METERS_PERSEC_PERSEC;
        parameters.calibrationDataFile = "BNO055IMUCalibration.json"; // see the calibration sample opmode
        parameters.loggingEnabled      = true;
        parameters.loggingTag          = "IMU";
        parameters.accelerationIntegrationAlgorithm = new JustLoggingAccelerationIntegrator();

        imu = hardwareMap.get(BNO055IMU.class, "imu");
        imu.initialize(parameters);


        while(angles.firstAngle < 90){
            robot.turn(0.3, false);
        }

        sleep(500);
        
        robot.setDrivePower(0.5, false);
        sleep(1000);
        robot.setDrivePower(0, true);

        robot.setDrivePower(0.5, false);
        sleep(500);
        robot.setDrivePower(0, true);

        }

    }
}
