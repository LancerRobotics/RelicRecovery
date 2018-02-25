package org.firstinspires.ftc.teamcode.Autonomous;

/**
 * Created by yibin.long on 2/12/2018.
 */

import com.qualcomm.hardware.bosch.BNO055IMU;
import com.qualcomm.hardware.bosch.JustLoggingAccelerationIntegrator;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.ColorSensor;
import com.qualcomm.robotcore.util.ElapsedTime;

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


@Autonomous (name = "Full Blue Auton", group = "Linear OpMode")
public class BlueAutonVuforiaGyroCombinedTest extends LinearOpMode {
    HardwareMechanumRobot robot = new HardwareMechanumRobot();

    //ELAPSED TIME
    private ElapsedTime runtime = new ElapsedTime();

    //VUFORIA
    //probably unnecessary, but in vuforia sample class
    public static final String TAG = "Vuforia VuMark Sample";
    OpenGLMatrix lastLocation = null;
    //stores instance of Vuforia locally
    VuforiaLocalizer vuforia;

    //GYRO
    BNO055IMU imu;
    // State used for updating telemetry
    Orientation angles;
    Acceleration gravity;

    public void setup(){

    }
    @Override
    public void runOpMode() {
        //VUFORIA
        //startup Vuforia and tell it to use the camera
        int cameraMonitorViewId = hardwareMap.appContext.getResources().getIdentifier("cameraMonitorViewId", "id", hardwareMap.appContext.getPackageName());
        VuforiaLocalizer.Parameters parameters = new VuforiaLocalizer.Parameters(cameraMonitorViewId);
        parameters.vuforiaLicenseKey = "ATBUgQH/////AAAAGUzEvDOFgUX9qPZkEOHOXVQ5Oeih/sEYcCN1LGl3wn8D0liJKP3Ml/2T+ZFO4QSKfpFT0keCBLD1Z6wwjVRx3dzlJmC/a3J+J6A6fGfVhh1CFTDlFRAMvFsrP3b/vP6SHJ9Eo8NKhgxs0JUGgmcWsuvx2PieZcpfh2rPn8EyM+8HiVjw4Wm+PZIcTeDrp0TkDVfw6arGNQXlKXG1KOM/dWLTdj9eec02TDYb7l5A1inuFChJz2xs3spTKe3MixOmsqtPjjfNiln188WCIn4ag6AV72y0x7d/eFUjYmcXVlvSUufV6NbqXZDM4k10N06NwJnHs+nrVo6TV7v6OXPM75vyc4MsgRJ6+C5ofSJVJX00";
        //choose front or back facing camera
        parameters.cameraDirection = VuforiaLocalizer.CameraDirection.FRONT;
        this.vuforia = ClassFactory.createVuforiaLocalizer(parameters);
        //load VuMark images
        VuforiaTrackables relicTrackables = this.vuforia.loadTrackablesFromAsset("RelicVuMark");
        VuforiaTrackable relicTemplate = relicTrackables.get(0);
        relicTemplate.setName("relicVuMarkTemplate"); // can help in debugging; otherwise not necessary

        //GYRO
        //Note: I renamed the gyro's "parameters" variable (in the sample code) to gyroParameters
        // to avoid conflicts with Vuforia's parameters
        BNO055IMU.Parameters gyroParameters = new BNO055IMU.Parameters();
        gyroParameters.angleUnit           = BNO055IMU.AngleUnit.DEGREES;
        gyroParameters.accelUnit           = BNO055IMU.AccelUnit.METERS_PERSEC_PERSEC;
        gyroParameters.calibrationDataFile = "BNO055IMUCalibration.json"; // see the calibration sample opmode
        gyroParameters.loggingEnabled      = true;
        gyroParameters.loggingTag          = "IMU";
        gyroParameters.accelerationIntegrationAlgorithm = new JustLoggingAccelerationIntegrator();
        //initialize gyro imu
        imu = hardwareMap.get(BNO055IMU.class, "imu");
        imu.initialize(gyroParameters);

        imu.startAccelerationIntegration(new Position(), new Velocity(), 1000);
        robot.init(hardwareMap, true);

        telemetry.addData(">", "Press Play to start");
        telemetry.update();

        waitForStart();

        //RESET RUNTIME
        runtime.reset();

        robot.autonGlyphL.setPosition(0);
        robot.autonGlyphR.setPosition(1);

        sleep(500);
/*
        //Make arm go down
        robot.jewelLift.setPosition(0.5);

        sleep(500);

        robot.jewelHitter.setPosition(0.5);
        sleep(500);
        robot.jewelLift.setPosition(0);
        telemetry.addLine("Position is 0");
        telemetry.update();
        sleep(2500);

        //Read the color
        if(robot.color.red()-3 > robot.color.blue()){
            robot.jewelHitter.setPosition(0.4);
            telemetry.addLine("Will hit this jewel (Red > Blue)");
            telemetry.update();
        }
        else {
            robot.jewelHitter.setPosition(1);
            telemetry.addLine("Will hit other jewel (Blue > Red)");
            telemetry.update();
        }
        sleep(1000);

        robot.jewelHitter.setPosition(0.4);
        robot.jewelLift.setPosition(0.7);

        //Hitting jewel


        sleep(1000);
*/
        //IDENTIFY VUMARK AND **to be added** HIT JEWEL
        relicTrackables.activate();
        RelicRecoveryVuMark vuMark = RelicRecoveryVuMark.from(relicTemplate);
        int angleToTurn = 90; //the "middle" one, needs testing
        //If you CAN see it, identify which one, otherwise just go to the middle one - gotta add that 2nd part, make it time based
        double time = runtime.seconds();
        while(vuMark == RelicRecoveryVuMark.UNKNOWN && opModeIsActive() && (runtime.seconds()-time) < 5) {
            vuMark = RelicRecoveryVuMark.from(relicTemplate);
            telemetry.addLine("VuMark not found");
            telemetry.addData("VuMark", "%s visible", vuMark);
            telemetry.update();
        }
        //***It will give the stuck in stop error if you let a while loop run past the 30 second mark
        //We can add a timer that if it doesnt detect after x seconds, it goes on
        int distanceToMove = 0;
        long timeToMove = 0;
        int angle = 0;

        if(vuMark.toString().equals("LEFT")){
            distanceToMove = 31;
            timeToMove = 800;
            telemetry.addLine("Go to left column"); //Encoded move to left column
            angle = 120;
        }
        else if(vuMark.toString().equals("CENTER")){
            distanceToMove = 23;
            timeToMove = 1000;
            telemetry.addLine("Go to middle column");
            angle = 110;
        }
        else if(vuMark.toString().equals("RIGHT")) {
            distanceToMove = 15;
            timeToMove = 1200;
            angle = 100;
            telemetry.addLine("Go to right column"); //Encoded move to right column
        }
        else {
            distanceToMove = 23;
            timeToMove = 1000;
            angle = 110;
            telemetry.addLine("Go to middle column");
        }
        telemetry.update();


        //MOVE TO CRYPTOBOX
        //init the angles value to 0
        angles = imu.getAngularOrientation(AxesReference.INTRINSIC, AxesOrder.ZYX, AngleUnit.DEGREES);

        //Move to and face cryptobox
        sleep(500);
        //move forwards
        robot.setDrivePower(0.6, false);
        sleep(700);
        robot.setDrivePower(0, false);

        sleep(500);

        //SAME FROM BLUEAUTONGYROTEST
        //turn left, change the angle value until it goes to right, mid, or left box
        while(angles.firstAngle < angle && opModeIsActive()){
            robot.turn(.4, true);
            angles = imu.getAngularOrientation(AxesReference.INTRINSIC, AxesOrder.ZYX, AngleUnit.DEGREES);
            telemetry.addData("Gyro value: ", angles.firstAngle);
            telemetry.update();
        }
        robot.turn(0, false);
        telemetry.addLine("Turn to cryptobox degrees done!");
        telemetry.update();
        sleep(1000);

        robot.autonGlyphL.setPosition(1);
        robot.autonGlyphR.setPosition(0);
        sleep(500);

        //Move into the cryptobox
        robot.setDrivePower(.3, false);
        sleep(2000);
        robot.setDrivePower(0, false);

        //no arms right now
        //robot.arm4.setPosition(.40);
        //robot.arm5.setPosition(.60);
        sleep(1500);
        //move backwards
        robot.setDrivePower(0.3, true);
        sleep(250);
        robot.setDrivePower(0, true);

        //if we want to turn 180, change this method a little
        /*
        while(angles.firstAngle < 90 && opModeIsActive()) {
            robot.turn(0.4, true);
            angles = imu.getAngularOrientation(AxesReference.INTRINSIC, AxesOrder.ZYX, AngleUnit.DEGREES);
            telemetry.addData("Gyro Value: ", angles.firstAngle);
            telemetry.update();
        }
        robot.turn(0, false);
        telemetry.addLine("Turn 90 degrees done!");
        telemetry.update();
        sleep(1000);
        */

    }
}

