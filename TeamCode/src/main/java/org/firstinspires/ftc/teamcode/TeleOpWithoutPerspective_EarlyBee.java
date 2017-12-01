//package org.firstinspires.ftc.teamcode;
//
//import com.qualcomm.hardware.bosch.BNO055IMU;
//import com.qualcomm.hardware.bosch.JustLoggingAccelerationIntegrator;
//import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
//import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
//import com.qualcomm.robotcore.util.ElapsedTime;
//import com.qualcomm.robotcore.util.Range;
//
//import org.firstinspires.ftc.robotcore.external.navigation.Acceleration;
//import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
//import org.firstinspires.ftc.robotcore.external.navigation.AxesOrder;
//import org.firstinspires.ftc.robotcore.external.navigation.AxesReference;
//import org.firstinspires.ftc.robotcore.external.navigation.Orientation;
//import org.firstinspires.ftc.robotcore.external.navigation.Position;
//import org.firstinspires.ftc.robotcore.external.navigation.Velocity;
//
///**
// * Created by dina.brustein on 11/11/2017.
// */
//
//@TeleOp
//
//public class TeleOpWithoutPerspective_EarlyBee extends LinearOpMode {
//
//        HardwareMechanumRobot robot = new HardwareMechanumRobot();
//        // Declare OpMode members.
//        private ElapsedTime runtime = new ElapsedTime();
//        public static double frPower, flPower, brPower, blPower;
//        public static boolean Button1_b, Button1_a, Button2_a, Button2_b, Button2_x, Button2_y;
//
//        //values for motor
//        double[] motorPwr = new double[4];
//        double x,y,z;
//        double trueX, trueY;
//
//        //imu object
//        //imu values
//        Orientation angles;
//        Acceleration gravity;
//        float theta;
//        float calibrate;
//
//        @Override
//        public void runOpMode() {
//            telemetry.addData("Status", "In Progress...");
//            telemetry.update();
//
//            robot.init(hardwareMap, false);
//
//            // Most robots need the motor on one side to be reversed to drive forward
//            // Reverse the motor that runs backwards when connected directly to the battery
//            //leftDrive.setDirection(DcMotor.Direction.FORWARD);
//            //rightDrive.setDirection(DcMotor.Direction.REVERSE);
//
//
//            telemetry.addData("Status", "Initialized");
//            telemetry.update();
//            // Wait for the game to start (driver presses PLAY)
//            waitForStart();
//            runtime.reset();
//
//            //zero yaw work / calibration
//
//
//            // run until the end of the match (driver presses STOP)
//            while (opModeIsActive()) {
//
//                //CHANGED ANGLE UNIT TO DEGREES
//
//                //telemetry.addData("angles (same as theta,in rad): ", angles);
//                if(gamepad1.right_stick_button && gamepad1.left_stick_button) {
//                    calibrate = 360 - theta;//angles.firstAngle; //.zeroYaw
//                }
//
//                //Sets the gamepad values to x, y, and z
//                z = -gamepad1.right_stick_x; //rotation
//                y = -gamepad1.left_stick_y; //forward and backward
//                x = gamepad1.left_stick_x; //side to side
//                telemetry.addData("Theta(in degrees): ", theta);
//                telemetry.addData("direct joystick x:", x);
//                telemetry.addData("direct joystick y:", y);
//                telemetry.addData("direct joystick z:", z);
//
//                //Converts x and y to a different value based on the gyro value
//                //trueX = ((Math.cos(Math.toDegrees(360-theta- calibrate)) * x) - ((Math.sin(Math.toDegrees(360-theta- calibrate))) * y)); //sets trueX to rotated value
//                //trueY = ((Math.sin(Math.toDegrees(360-theta- calibrate))) * x) + ((Math.cos(Math.toDegrees(360-theta- calibrate))) * y);
//
//
//                z = gamepad1.left_stick_x; //moving left/right
//                y = gamepad1.left_stick_y; //moving forwards/backwards
//                x = gamepad1.right_stick_x; //turning
//                Button1_a = gamepad1.a; //open glyph grabber
//                Button2_a = gamepad2.a; //lift relic partially
//                Button2_b = gamepad2.b; //lift relic parallel to ground
//                Button2_x = gamepad2.x; //clamp over relic
//                Button2_y = gamepad2.y; //place relic perpendicular to ground
//
//
//
//
//                if (Button1_a){ //open and close glyph grabber; //0 close or 1 is open
//                    if(robot.arm4.getPosition() == robot.ARM_4_CLOSED && robot.arm5.getPosition() == robot.ARM_5_CLOSED){
//                        robot.arm4.setPosition(robot.ARM_4_OPEN);
//                        robot.arm5.setPosition(robot.ARM_5_OPEN);
//                    }
//                    else {
//                        robot.arm4.setPosition(robot.ARM_4_CLOSED);
//                        robot.arm5.setPosition(robot.ARM_5_CLOSED);
//                    }
//                }
//                if (Button2_a){ //relic grabber move up partially
//                    robot.arm1.setPosition(robot.ARM_1_DOWN);
//                    robot.arm2.setPosition(robot.ARM_2_DOWN);
//                }
//
//                if (Button2_b){ //relic grabber move up parallel to ground, 0 is up
//                    robot.arm1.setPosition(robot.ARM_1_MIDDLE);
//                    robot.arm2.setPosition(robot.ARM_2_MIDDLE);
//                }
//
//                if (Button2_x){ //clamp or unclamp over relic
//                    if(robot.arm3.getPosition() == robot.ARM_3_CLAMP){
//                        robot.arm3.setPosition(robot.ARM_3_UNCLAMP);
//                    }
//                    else {
//                        robot.arm3.setPosition(robot.ARM_3_CLAMP);
//                    }
//                }
//
//                if (Button2_y){ //move relic perpendicular to ground, 1 is down
//                    robot.arm1.setPosition(robot.ARM_1_UP);
//                    robot.arm2.setPosition(robot.ARM_2_UP);
//                }
//
//                //Sets the motor powers of the wheels to the correct power based on all three of the above gyro values and
//                //scales them accordingly
//                flPower = Range.clip((-x + y - z), -1, 1);
//                frPower = Range.clip((-x - y - z), -1, 1);
//                blPower = Range.clip((x + y - z), -1, 1);
//                brPower = Range.clip((x - y - z), -1, 1);
//
//                robot.fl.setPower(flPower);
//                robot.fr.setPower(frPower);
//                robot.bl.setPower(blPower);
//                robot.br.setPower(brPower);
//
//                // Show the elapsed game time and wheel power.
//                telemetry.addData("Status", "Run Time: " + runtime.toString());
//                telemetry.update();
//            }
//        }
//    }
