package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.util.ElapsedTime;
import com.qualcomm.robotcore.util.Range;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.hardware.bosch.BNO055IMU;
import com.qualcomm.hardware.bosch.JustLoggingAccelerationIntegrator;

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

@TeleOp(name="MecanumTestPerspective-USE THIS", group="Linear Opmode")
//@Disabled
public class MecanumTestPerspective extends LinearOpMode {
    HardwareMechanumRobot robot = new HardwareMechanumRobot();
    // Declare OpMode members.
    private ElapsedTime runtime = new ElapsedTime();
    public DcMotor fr; //= null;
    public DcMotor fl; //= null;
    public DcMotor bl; //= null;
    public DcMotor br; //= null;
    public Servo arm1 = null; //relic lifter right
    public Servo arm2 = null; //relic lifter left
    public Servo arm3 = null; //relic clamper
    public Servo arm4 = null; //glyph grabber left servo
    public Servo arm5 = null; //glyph grabber right servo
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

        robot.init(hardwareMap, false);

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
            if(gamepad1.right_stick_button && gamepad1.left_stick_button) {
                calibrate = 360 - theta;//angles.firstAngle; //.zeroYaw
            }

            //Sets the gamepad values to x, y, and z
            z = -gamepad1.right_stick_x; //rotation
            y = -gamepad1.left_stick_y; //forward and backward
            x = gamepad1.left_stick_x; //side to side
            telemetry.addData("Theta(in degrees): ", theta);
            telemetry.addData("direct joystick x:", x);
            telemetry.addData("direct joystick y:", y);
            telemetry.addData("direct joystick z:", z);

            //Converts x and y to a different value based on the gyro value
            //trueX = ((Math.cos(Math.toDegrees(360-theta- calibrate)) * x) - ((Math.sin(Math.toDegrees(360-theta- calibrate))) * y)); //sets trueX to rotated value
            //trueY = ((Math.sin(Math.toDegrees(360-theta- calibrate))) * x) + ((Math.cos(Math.toDegrees(360-theta- calibrate))) * y);


            z = gamepad1.left_stick_x; //moving left/right
            y = gamepad1.left_stick_y; //moving forwards/backwards
            x = gamepad1.right_stick_x; //turning
            Button1_a = gamepad1.a; //open glyph grabber
            Button2_a = gamepad2.a; //lift relic partially
            Button2_b = gamepad2.b; //lift relic parallel to ground
            Button2_x = gamepad2.x; //clamp over relic
            Button2_y = gamepad2.y; //place relic perpendicular to ground




            //arm4 is left glyph grabber arm, arm5 is right glyph grabber arm

            if (Button1_a==true){ //open and close glyph grabber
               int pos = 0; //0 close or 1 is open
                if (pos ==0){ //close
                    robot.arm4.setPosition(robot.ARM_4_CLOSED);
                    robot.arm5.setPosition(robot.ARM_5_CLOSED);
                    pos = 1;
                }
                else{ //open
                    robot.arm4.setPosition(robot.ARM_4_OPEN);
                    robot.arm5.setPosition(robot.ARM_5_OPEN);
                    pos = 0;
                }
            }
            if (Button2_a){ //relic grabber move up partially
                robot.arm1.setPosition(robot.ARM_1_DOWN);
                robot.arm2.setPosition(robot.ARM_2_DOWN);
            }

            if (Button2_b){ //relic grabber move up parallel to ground, 0 is up
                robot.arm1.setPosition(robot.ARM_1_MIDDLE);
                robot.arm2.setPosition(robot.ARM_2_MIDDLE);
            }

            if (Button2_x){ //clamp or unclamp over relic
                int pos = 0; //clamp or unclamp
                if(pos == 0){ //clamp
                    robot.arm3.setPosition(robot.ARM_3_CLAMP);
                    pos = 1;
                }
                else{ //unclamp
                    robot.arm3.setPosition(robot.ARM_3_UNCLAMP);
                    pos = 0;
                }
            }

            if (Button2_y){ //move relic perpendicular to ground, 1 is down
                robot.arm1.setPosition(robot.ARM_1_UP);
                robot.arm2.setPosition(robot.ARM_2_UP);
            }

            trueX = (Math.cos(theta+calibrate)*x) - (Math.sin(theta+calibrate)*y); //sets trueX to rotated value
            trueY = (Math.sin(theta+calibrate)*x) + (Math.cos(theta+calibrate)*y);

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


            // Show the elapsed game time and wheel power.
            telemetry.addData("Status", "Run Time: " + runtime.toString());
            telemetry.update();
        }
    }
}
