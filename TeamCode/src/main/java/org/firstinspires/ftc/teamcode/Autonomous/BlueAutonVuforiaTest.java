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


//@Autonomous (name = "Blue Vuforia Auton TEST - USE THIS", group = "Linear OpMode")
public class BlueAutonVuforiaTest extends LinearOpMode {

    HardwareMechanumRobot robot = new HardwareMechanumRobot();

    //probably unnecessary, but in vuforia sample class
    public static final String TAG = "Vuforia VuMark Sample";
    OpenGLMatrix lastLocation = null;

    //stores instance of Vuforia locally
    VuforiaLocalizer vuforia;

    @Override public void runOpMode() {

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

        telemetry.addData(">", "Press Play to start");
        telemetry.update();
        waitForStart();

        relicTrackables.activate();

        while (opModeIsActive()) {
            RelicRecoveryVuMark vuMark = RelicRecoveryVuMark.from(relicTemplate);
            if (vuMark != RelicRecoveryVuMark.UNKNOWN) {
                telemetry.addData("VuMark", "%s visible", vuMark);
            }
            else {
                telemetry.addData("VuMark", "not visible");
            }


        }

    }
}
