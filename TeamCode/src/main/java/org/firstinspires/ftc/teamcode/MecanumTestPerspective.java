package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.util.ElapsedTime;
import com.qualcomm.robotcore.util.Range;

import com.qualcomm.hardware.bosch.BNO055IMU;
import com.qualcomm.hardware.bosch.JustLoggingAccelerationIntegrator;

import org.firstinspires.ftc.robotcore.external.navigation.Acceleration;
import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.robotcore.external.navigation.AxesOrder;
import org.firstinspires.ftc.robotcore.external.navigation.AxesReference;
import org.firstinspires.ftc.robotcore.external.navigation.Orientation;
import org.firstinspires.ftc.robotcore.external.navigation.Position;
import org.firstinspires.ftc.robotcore.external.navigation.Velocity;


/**
 * This file contains an minimal example of a Linear "OpMode". An OpMode is a 'program' that runs in either
 * the autonomous or the teleop period of an FTC match. The names of OpModes appear on the menu
 * of the FTC Driver Station. When an selection is made from the menu, the corresponding OpMode
 * class is instantiated on the Robot Controller and executed.
 *
 * This particular OpMode just executes a basic Tank Drive Teleop for a two wheeled robot
 * It includes all the skeletal structure that all linear OpModes contain.
 *
 * Use Android Studios to Copy this Class, and Paste it into your team's code folder with a new name.
 * Remove or comment out the @Disabled line to add this opmode to the Driver Station OpMode list
 */

@TeleOp(name="MecanumTestPerspective", group="Linear Opmode")
//@Disabled
public class MecanumTestPerspective extends LinearOpMode {

    // Declare OpMode members.
    private ElapsedTime runtime = new ElapsedTime();
    public DcMotor fr; //= null;7
    public DcMotor fl; //= null;
    public DcMotor bl; //= null;
    public DcMotor br; //= null;
    public static double frPower, flPower, brPower, blPower;

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

        // Initialize the hardware variables. Note that the strings used here as parameters
        // to 'get' must correspond to the names assigned during the robot configuration
        // step (using the FTC Robot Controller app on the phone).
        fr = hardwareMap.dcMotor.get("front_right");
        fl = hardwareMap.dcMotor.get("front_left");
        bl = hardwareMap.dcMotor.get("back_left");
        br = hardwareMap.dcMotor.get("back_right");

        // Most robots need the motor on one side to be reversed to drive forward
        // Reverse the motor that runs backwards when connected directly to the battery
        //leftDrive.setDirection(DcMotor.Direction.FORWARD);
        //rightDrive.setDirection(DcMotor.Direction.REVERSE);


        telemetry.addData("Status", "Initialized");
        telemetry.update();
        // Wait for the game to start (driver presses PLAY)
        waitForStart();
        runtime.reset();
        imu.startAccelerationIntegration(new Position(), new Velocity(), 1000);

        //zero yaw work / calibration


        // run until the end of the match (driver presses STOP)
        while (opModeIsActive()) {

            angles = imu.getAngularOrientation(AxesReference.INTRINSIC, AxesOrder.XYZ, AngleUnit.DEGREES);
            //CHANGED ANGLE UNIT TO DEGREES

            //telemetry.addData("angles (same as theta,in rad): ", angles);
            theta = angles.firstAngle;
            //if(gamepad1.right_stick_button && gamepad1.left_stick_button) {
            //    calibrate = 360 - theta;//angles.firstAngle; //.zeroYaw
            //}
            //Sets the gamepad values to x, y, and z
            z = -gamepad1.right_stick_x; //rotation
            y = -gamepad1.left_stick_y; //forward and backward
            x = gamepad1.left_stick_x; //side to side
            telemetry.addData("Theta(in rads): ", theta);
            telemetry.addData("first x:", x);
            telemetry.addData("first y:", y);


            //Converts x and y to a different value based on the gyro value
            //trueX = ((Math.cos(Math.toDegrees(360-theta- calibrate)) * x) - ((Math.sin(Math.toDegrees(360-theta- calibrate))) * y)); //sets trueX to rotated value
            //trueY = ((Math.sin(Math.toDegrees(360-theta- calibrate))) * x) + ((Math.cos(Math.toDegrees(360-theta- calibrate))) * y);
            //((Math.cos(Math.toRadians(360 - Artemis.convertYaw(Artemis.navx_device.getYaw())))) * x)
            trueX = ((Math.cos(Math.toRadians(360 - theta)))*x) - ((Math.sin(Math.toRadians(360 - theta)))*y); //sets trueX to rotated value
            trueY = ((Math.sin(Math.toRadians(360 - theta)))*x) - ((Math.cos(Math.toRadians(360 - theta)))*y);

            //Sets trueX and trueY to its respective value
            x = trueX;
            y = trueY;

            telemetry.addData("true x:", x);
            telemetry.addData("true y:", y);

            //deadzone
  /*          if(x > -0.05 || x < 0.05) {
                x = 0;
            }
            if(y > -.05 || y < .05){
                y = 0;
            }
*/

            //Sets the motor powers of the wheels to the correct power based on all three of the above gyro values and
            //scales them accordingly
            flPower = Range.clip((-x + y - z), -1, 1);
            frPower = Range.clip((-x - y - z), -1, 1);
            blPower = Range.clip((x + y - z), -1, 1);
            brPower = Range.clip((x - y - z), -1, 1);


            //Sets each motor power to the correct power
            fl.setPower(flPower);
            fr.setPower(frPower);
            bl.setPower(blPower);
            br.setPower(brPower);

            // Show the elapsed game time and wheel power.
            telemetry.addData("Status", "Run Time: " + runtime.toString());
            telemetry.update();
        }
    }
}
