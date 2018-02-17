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
        potentiometer = hardwareMap.analogInput.get("potentiometer");

        //Start logging acceleration
        imu.startAccelerationIntegration(new Position(), new Velocity(), 1000);

        robot.init(hardwareMap, false);
        //robot.arm1.setPosition(robot.ARM_1_CLOSED);
        //robot.arm2.setPosition(robot.ARM_2_CLOSED);

        // Most robots need the motor on one side to be reversed to drive forward
        // Reverse the motor that runs backwards when connected directly to the battery
        //leftDrive.setDirection(DcMotor.Direction.FORWARD);
        //rightDrive.setDirection(DcMotor.Direction.REVERSE);


        telemetry.addData("Status", "Initialized");
        telemetry.update();
        robot.fr.setDirection(DcMotorSimple.Direction.FORWARD);
        robot.br.setDirection(DcMotorSimple.Direction.FORWARD);
        // Wait for the game to start (driver presses PLAY)
        waitForStart();
        runtime.reset();


        // run until the end of the match (driver presses STOP)
        while (opModeIsActive() && !isStopRequested()) {

            angles = imu.getAngularOrientation(AxesReference.INTRINSIC, AxesOrder.ZYX, AngleUnit.DEGREES);
            //CHANGED ANGLE UNIT TO DEGREES
            telemetry.addData("Gyro angle: ", angles.firstAngle);
            //telemetry.addData("angles (same as theta,in rad): ", angles);
            theta = angles.firstAngle;
            if(gamepad1.right_stick_button && gamepad1.left_stick_button) {
                calibrate = 360 - theta;//goes from 180 to 540, start at 360//.zeroYaw

            }

            //Sets the gamepad values to x, y, and z
            z = gamepad1.right_stick_x; //rotation
            x = gamepad1.left_stick_x; //forward and backward
            y = gamepad1.left_stick_y; //side to side
            //deadzones
            if(Math.abs(x) < .15) {
                x = 0;
            }
            if(Math.abs(y) < .15) {
                y = 0;
            }
            if(Math.abs(z) < .15) {
                z = 0;
            }
            telemetry.addData("Theta(in degrees): ", 360-theta+calibrate);
            telemetry.addData("direct joystick x:", x);
            telemetry.addData("direct joystick y:", y);
            telemetry.addData("direct joystick z:", z);
            telemetry.addData("Current Encoder Position: ", robot.fr.getCurrentPosition());

            //commenting out for now, these servos were stripped from the old robot (it'll crash if you use them)
            /*
            if (gamepad1.a){ //Grab glyph with bottom arms
                robot.arm1.setPosition(robot.ARM_1_CLOSED);
                robot.arm2.setPosition(robot.ARM_2_CLOSED);
                robot.arm4.setPosition(robot.ARM_4_CLOSED);
                robot.arm5.setPosition(robot.ARM_5_CLOSED);
                //gamepad2.a and .b -> the left doesnt move
            }

            if (gamepad1.b){ //Let go of glyph with bottom arms
                robot.arm1.setPosition(robot.ARM_1_OPEN);
                robot.arm2.setPosition(robot.ARM_2_OPEN);
                robot.arm4.setPosition(robot.ARM_4_OPEN);
                robot.arm5.setPosition(robot.ARM_5_OPEN);
            }

            if (gamepad1.x){ //Open top glyph grabbers
                robot.arm1.setPosition(robot.ARM_1_OPEN);
                robot.arm2.setPosition(robot.ARM_2_OPEN);
                robot.arm4.setPosition(robot.ARM_4_OPEN);
                robot.arm5.setPosition(robot.ARM_5_OPEN);
            }

            if (gamepad1.y){ //Close top glyph grabbers
                robot.arm1.setPosition(robot.ARM_1_CLOSED);
                robot.arm2.setPosition(robot.ARM_2_CLOSED);
                robot.arm4.setPosition(robot.ARM_4_CLOSED);
                robot.arm5.setPosition(robot.ARM_5_CLOSED);
            }
            */
            if(gamepad1.dpad_left){
                robot.flPower -= .1;
                robot.frPower -= .1;
                robot.blPower -= .1;
                robot.brPower-= .1;
            }

            if(gamepad1.dpad_right){
                robot.flPower -= .1;
                robot.frPower -= .1;
                robot.blPower -= .1;
                robot.brPower -= .1;
            }
        // commenting these out for now, these motors were stripped from the old robot
            /*
            if(gamepad2.a){
                robot.glyph.setPower(0.75);
            }

            else if(gamepad2.b){
                robot.glyph.setPower(-0.75);
            }

            else {
                robot.glyph.setPower(0);
            }

            if(gamepad2.y){
                robot.relic.setPower(0.8);
            }

            else if(gamepad2.x){
                robot.relic.setPower(-0.8);
            }

            else {
                robot.relic.setPower(0);
            }

            if(gamepad1.right_bumper){
                robot.extender.setPower(0.99);
            }

            else if(gamepad1.left_bumper){
                robot.extender.setPower(-0.99);
            }

            else {
                robot.extender.setPower(0);
            }

            if(gamepad2.left_bumper){
                robot.arm0.setPosition(robot.ARM_0_UP);
            }

            if(gamepad2.right_bumper){
                robot.arm0.setPosition(robot.ARM_0_DOWN);
            }
         */

            //((Math.cos(Math.toRadians(360 - Artemis.convertYaw(Artemis.navx_device.getYaw())))) * x)
            trueX = ((Math.cos(Math.toRadians(360 - theta + calibrate)))*x) -
                    ((Math.sin(Math.toRadians(360 - theta + calibrate)))*y);
            trueY = ((Math.sin(Math.toRadians(360 - theta + calibrate)))*x) -
                    ((Math.cos(Math.toRadians(360 - theta + calibrate)))*y);

            //Sets trueX and trueY to its respective value
            x = trueX;
            y = trueY;

            telemetry.addData("true x:", x);
            telemetry.addData("true y:", y);

            flPower = Range.scale((x + y + z), -1, 1, -robot.MAX_MOTOR_SPEED, robot.MAX_MOTOR_SPEED);
            frPower = Range.scale((x - y + z), -1, 1, -robot.MAX_MOTOR_SPEED, robot.MAX_MOTOR_SPEED);
            blPower = Range.scale((-x + y + z), -1, 1, -robot.MAX_MOTOR_SPEED, robot.MAX_MOTOR_SPEED);
            brPower = Range.scale((-x - y + z), -1, 1, -robot.MAX_MOTOR_SPEED, robot.MAX_MOTOR_SPEED);

            //Sets each motor power to the correct power
            robot.fl.setPower(flPower);
            robot.fr.setPower(frPower);
            robot.bl.setPower(blPower);
            robot.br.setPower(brPower);



                telemetry.addData("Front left encoder position: ", robot.fl.getCurrentPosition());
                telemetry.update();

            //TEMPORARY TO RESET ENCODER POSITION
            if(gamepad1.a) {
                robot.fl.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);

            }
            robot.fl.setMode(DcMotor.RunMode.RUN_TO_POSITION);
            // Show the elapsed game time and wheel power.
            telemetry.addData("Status", "Run Time: " + runtime.toString());
            telemetry.update();
        }
    }
}
