package org.firstinspires.ftc.teamcode.Autonomous;

/**
 * Created by yibin.long on 2/12/2018.
 */

import com.qualcomm.hardware.bosch.BNO055IMU;
import com.qualcomm.hardware.bosch.JustLoggingAccelerationIntegrator;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.ColorSensor;

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


@Autonomous (name = "Blue Vuforia/Gyro Auton TEST - USE THIS", group = "Linear OpMode")
public class BlueAutonVuforiaGyroCombinedTest extends LinearOpMode {
    HardwareMechanumRobot robot = new HardwareMechanumRobot();

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
    @Override public void runOpMode() {
        //VUFORIA
        //startup Vuforia and tell it to use the camera
        int cameraMonitorViewId = hardwareMap.appContext.getResources().getIdentifier("cameraMonitorViewId", "id", hardwareMap.appContext.getPackageName());
        VuforiaLocalizer.Parameters parameters = new VuforiaLocalizer.Parameters(cameraMonitorViewId);
        parameters.vuforiaLicenseKey = "ATBUgQH/////AAAAGUzEvDOFgUX9qPZkEOHOXVQ5Oeih/sEYcCN1LGl3wn8D0liJKP3Ml/2T+ZFO4QSKfpFT0keCBLD1Z6wwjVRx3dzlJmC/a3J+J6A6fGfVhh1CFTDlFRAMvFsrP3b/vP6SHJ9Eo8NKhgxs0JUGgmcWsuvx2PieZcpfh2rPn8EyM+8HiVjw4Wm+PZIcTeDrp0TkDVfw6arGNQXlKXG1KOM/dWLTdj9eec02TDYb7l5A1inuFChJz2xs3spTKe3MixOmsqtPjjfNiln188WCIn4ag6AV72y0x7d/eFUjYmcXVlvSUufV6NbqXZDM4k10N06NwJnHs+nrVo6TV7v6OXPM75vyc4MsgRJ6+C5ofSJVJX00";
        //choose front or back facing camera
        parameters.cameraDirection = VuforiaLocalizer.CameraDirection.BACK;
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

        relicTrackables.activate();
        //IDENTIFY VUMARK AND **to be added** HIT JEWEL
        RelicRecoveryVuMark vuMark = RelicRecoveryVuMark.from(relicTemplate);
        int cryptobox = 1; // 0 is left, 1 is center, 2 is right - just to testing, delete soon!
        int angleToTurn = 135; //the "middle" one, needs testing
        //If you CAN see it, identify which one, otherwise just go to the middle one
        while(vuMark == RelicRecoveryVuMark.UNKNOWN && opModeIsActive()) {
            vuMark = RelicRecoveryVuMark.from(relicTemplate);
            telemetry.addLine("VuMark not found");
            telemetry.addData("VuMark", "%s visible", vuMark);
            telemetry.update();
        }
        //***It will give the stuck in stop error if you let a while loop run past the 30 second mark
        //We can add a timer that if it doesnt detect after x seconds, it goes on

        if(vuMark.toString().equals("LEFT")){
            cryptobox = 0;
            angleToTurn = 120;
            telemetry.addLine("Go to left column"); //Encoded move to left column
        }
        if(vuMark.toString().equals("CENTER")){
            cryptobox = 1;
            angleToTurn = 135;
            telemetry.addLine("Go to middle column");
        }
        if(vuMark.toString().equals("RIGHT")) {
            cryptobox = 2;
            angleToTurn = 150;
            telemetry.addLine("Go to right column"); //Encoded move to right column
        }
        telemetry.update();


        //MOVE TO CRYPTOBOX
        //init the angles value to 0
        angles = imu.getAngularOrientation(AxesReference.INTRINSIC, AxesOrder.ZYX, AngleUnit.DEGREES);

        //Move to and face cryptobox
        sleep(500);
        //move forwards
        robot.setDrivePower(0.5, false);
        sleep(1000);
        robot.setDrivePower(0, true);

        //SAME FROM BLUEAUTONGYROTEST
        //turn left, change the angle value until it goes to right, mid, or left box
        while(angles.firstAngle < angleToTurn && opModeIsActive()){
            robot.turn(.4, true);
            angles = imu.getAngularOrientation(AxesReference.INTRINSIC, AxesOrder.ZYX, AngleUnit.DEGREES);
            telemetry.addData("Gyro value: ", angles.firstAngle);
            telemetry.update();
        }
        robot.turn(0, false);
        telemetry.addLine("Turn to cyrptobox degrees done!");
        telemetry.update();
        sleep(1000);

        //Move into the cryptobox
        robot.setDrivePower(.5, false);
        sleep(1000);
        robot.setDrivePower(0, false);

        sleep(500);
        //no arms right now
        //robot.arm4.setPosition(.40);
        //robot.arm5.setPosition(.60);
        sleep(1500);
        //move backwards
        robot.setDrivePower(0.5, true);
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

