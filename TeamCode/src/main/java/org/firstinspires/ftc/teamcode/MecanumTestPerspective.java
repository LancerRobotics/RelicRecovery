package org.firstinspires.ftc.teamcode;

import com.qualcomm.hardware.bosch.BNO055IMU;
import com.qualcomm.hardware.bosch.JustLoggingAccelerationIntegrator;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.AnalogInput;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.util.ElapsedTime;
import com.qualcomm.robotcore.util.Range;

import org.firstinspires.ftc.robotcore.external.navigation.Acceleration;
import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.robotcore.external.navigation.AxesOrder;
import org.firstinspires.ftc.robotcore.external.navigation.AxesReference;
import org.firstinspires.ftc.robotcore.external.navigation.Orientation;
import org.firstinspires.ftc.robotcore.external.navigation.Position;
import org.firstinspires.ftc.robotcore.external.navigation.Velocity;
import org.firstinspires.ftc.teamcode.HardwareMechanumRobot;

@TeleOp(name="Teleop Perspective", group="plswork")
//@Disabled
public class MecanumTestPerspective extends LinearOpMode {
    HardwareMechanumRobot robot = new HardwareMechanumRobot();
    // Declare OpMode members.
    private ElapsedTime runtime = new ElapsedTime();
    public static double frPower, flPower, brPower, blPower;
    public static boolean Button1_b, Button1_a, Button2_a, Button2_b, Button2_x, Button2_y;

    //values for motor
    double[] motorPwr = new double[4];
    double x,y,z;
    double trueX, trueY;

    //imu object
    BNO055IMU imu;
    //imu values
    Orientation angles;
    Acceleration gravity;
    float theta;
    float calibrate;

    //potentiometer
    AnalogInput potentiometer;


    @Override
    public void runOpMode() {
        telemetry.addData("Status", "In Progress...");
        telemetry.update();

        BNO055IMU.Parameters parameters = new BNO055IMU.Parameters();
        parameters.angleUnit           = BNO055IMU.AngleUnit.DEGREES;
        parameters.accelUnit           = BNO055IMU.AccelUnit.METERS_PERSEC_PERSEC;
        parameters.calibrationDataFile = "BNO055IMUCalibration.json"; // see the calibration sample opmode
        parameters.loggingEnabled      = true;
        parameters.loggingTag          = "IMU";
        parameters.accelerationIntegrationAlgorithm = new JustLoggingAccelerationIntegrator();

        imu = hardwareMap.get(BNO055IMU.class, "imu");
        imu.initialize(parameters);

        //potentiometer
        //potentiometer = hardwareMap.analogInput.get("potentiometer");

        //Start logging acceleration
        imu.startAccelerationIntegration(new Position(), new Velocity(), 1000);
        robot.init(hardwareMap, false);

        telemetry.addData("Status", "PLSWORK");
        telemetry.update();
        // Wait for the game to start (driver presses PLAY)
        waitForStart();
        runtime.reset();

        //Testing to see if all telemetry is kept
        telemetry.setAutoClear(false);

        // run until the end of the match (driver presses STOP)
        while (opModeIsActive() && !isStopRequested()) {

            //Sets the gamepad values to x, y, and z
            z = gamepad1.right_stick_x; //rotation
            y = gamepad1.left_stick_y; //side to side
            x = gamepad1.left_stick_x; //forward and backward

            //deadzones
            if(Math.abs(x) < .1) {
                x = 0;
            }
            if(Math.abs(y) < .1) {
                y = 0;
            }
            if(Math.abs(z) < .1) {
                z = 0;
            }


            //telemetry.addData("Calibrate value: ", calibrate);
            //C&P
            angles = imu.getAngularOrientation(AxesReference.INTRINSIC, AxesOrder.ZYX, AngleUnit.DEGREES);
            theta = angles.firstAngle;

            //the convert method
            if (theta <= 0) {
                theta = 360 + theta; //if yaw is negative, make it positive (makes the turn easier to visualize)
            }

            if(gamepad1.right_stick_button && gamepad1.left_stick_button) {
                calibrate = 360 - theta;
            }

            trueX = (Math.cos(Math.toRadians(theta + calibrate)) * x) - (Math.sin(Math.toRadians(theta + calibrate)) * y);
            trueY = (Math.sin(Math.toRadians(theta + calibrate)) * x) + (Math.cos(Math.toRadians(theta + calibrate)) * y);

            x = trueX;
            y = trueY;

            flPower = Range.scale((-x + y - z), -1, 1, -robot.MAX_MOTOR_SPEED, robot.MAX_MOTOR_SPEED);
            frPower = Range.scale((-x - y - z), -1, 1, -robot.MAX_MOTOR_SPEED, robot.MAX_MOTOR_SPEED);
            blPower = Range.scale((x + y - z), -1, 1, -robot.MAX_MOTOR_SPEED, robot.MAX_MOTOR_SPEED);
            brPower = Range.scale((x - y - z), -1, 1, -robot.MAX_MOTOR_SPEED, robot.MAX_MOTOR_SPEED);

            robot.fl.setPower(flPower);
            robot.fr.setPower(frPower);
            robot.bl.setPower(blPower);
            robot.br.setPower(brPower);

            telemetry.clearAll();
            telemetry.addData("Converted Gyro Value", theta);
            telemetry.addData("Calibrate", calibrate);
            telemetry.addData("FR Power", robot.fr.getPower());
            telemetry.addData("FL Power", robot.fl.getPower());
            telemetry.addData("BR Power", robot.br.getPower());
            telemetry.addData("BL Power", robot.bl.getPower());
            telemetry.addData("Old X on Gamepad", x);
            telemetry.addData("Old Y on Gamepad", y);
            telemetry.update();
        }
    }
}
//AJ SAVES THE DAY