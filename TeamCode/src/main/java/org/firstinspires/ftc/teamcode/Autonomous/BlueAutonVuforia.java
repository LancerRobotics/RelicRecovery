package org.firstinspires.ftc.teamcode.Autonomous;

import com.qualcomm.hardware.bosch.BNO055IMU;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.ColorSensor;

import org.firstinspires.ftc.robotcore.external.ClassFactory;
import org.firstinspires.ftc.robotcore.external.matrices.OpenGLMatrix;
import org.firstinspires.ftc.robotcore.external.matrices.VectorF;
import org.firstinspires.ftc.robotcore.external.navigation.Acceleration;
import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.robotcore.external.navigation.AxesOrder;
import org.firstinspires.ftc.robotcore.external.navigation.AxesReference;
import org.firstinspires.ftc.robotcore.external.navigation.Orientation;
import org.firstinspires.ftc.robotcore.external.navigation.RelicRecoveryVuMark;
import org.firstinspires.ftc.robotcore.external.navigation.VuMarkInstanceId;
import org.firstinspires.ftc.robotcore.external.navigation.VuforiaLocalizer;
import org.firstinspires.ftc.robotcore.external.navigation.VuforiaTrackable;
import org.firstinspires.ftc.robotcore.external.navigation.VuforiaTrackableDefaultListener;
import org.firstinspires.ftc.robotcore.external.navigation.VuforiaTrackables;
import org.firstinspires.ftc.teamcode.HardwareMechanumRobot;
import org.firstinspires.ftc.teamcode.Vuforia;

import static com.sun.tools.javac.util.Constants.format;

//@Autonomous (name = "Blue COLOR + Vuforia Auton - USE THIS", group = "Linear OpMode")
//@Disabled
public class BlueAutonVuforia extends LinearOpMode {
    HardwareMechanumRobot robot = new HardwareMechanumRobot();
    //ColorSensor color;
    public static final String TAG = "Vuforia VuMark Sample";

    OpenGLMatrix lastLocation = null;
    BNO055IMU imu;

    Orientation angles;
    Acceleration gravity;
    VuforiaLocalizer vuforia;

    public void setup(){

    }

    public void runOpMode() {
        robot.init(hardwareMap, true);
//
        //for a CR Servo, dont set the position to anything
        //robot.jewel_hitter.setPosition(.3);
        //robot.arm1.setPosition(robot.ARM_1_CLOSED);

        waitForStart();

        Vuforia vuforia = new Vuforia();

        int targetValue = 0;
 /*
   //     robot.jewel_hitter.setPower(-0.5);
   //     sleep(300);
   //     robot.jewel_hitter.setPower(0);

   //     sleep(500);

   //     robot.jewel0.setPower(-0.5);
   //     sleep(1000);
   //     robot.jewel0.setPower(0);

        //MAKE THE JEWEL HITTER MOVE FIRST, THEN THE OTHER 2 JEWEL SERVOS

        telemetry.addData("Blue: ", robot.color_sensor.blue());
        telemetry.addData("Red: ", robot.color_sensor.red());
        telemetry.update();

        telemetry.update();
        sleep(1000);

        //I added "-3" because the red is much stronger than blue
        if(robot.color_sensor.red()-3 > robot.color_sensor.blue()){
            telemetry.addLine("Will hit this jewel");
            telemetry.update();
            //MAKE RED AND BLUE AUTONS!!!
            robot.jewel_hitter.setPower(-.4);
            sleep(400);
            robot.jewel_hitter.setPower(0);
        }
        else {
            telemetry.addLine("Will hit other jewel");
            telemetry.update();
            robot.jewel_hitter.setPower(.4);
            sleep(400);
        }            robot.jewel_hitter.setPower(0);

        sleep(1000);
//        robot.jewel0.setPosition(.65);
        robot.jewel1.setPosition(.65);
        sleep(500);


        robot.jewel0.setPower(0.5);
        sleep(2000);
        robot.jewel0.setPower(0);

        sleep(500);

        robot.arm4.setPosition(robot.ARM_4_CLOSED_AUTON);
        robot.arm5.setPosition(robot.ARM_5_CLOSED_AUTON);

        sleep(500);
  */
        int cameraMonitorViewId = hardwareMap.appContext.getResources().getIdentifier("cameraMonitorViewId", "id", hardwareMap.appContext.getPackageName());
        VuforiaLocalizer.Parameters parameters = new VuforiaLocalizer.Parameters(cameraMonitorViewId);

        parameters.vuforiaLicenseKey = "ATBUgQH/////AAAAGUzEvDOFgUX9qPZkEOHOXVQ5Oeih/sEYcCN1LGl3wn8D0liJKP3Ml/2T+ZFO4QSKfpFT0keCBLD1Z6wwjVRx3dzlJmC/a3J+J6A6fGfVhh1CFTDlFRAMvFsrP3b/vP6SHJ9Eo8NKhgxs0JUGgmcWsuvx2PieZcpfh2rPn8EyM+8HiVjw4Wm+PZIcTeDrp0TkDVfw6arGNQXlKXG1KOM/dWLTdj9eec02TDYb7l5A1inuFChJz2xs3spTKe3MixOmsqtPjjfNiln188WCIn4ag6AV72y0x7d/eFUjYmcXVlvSUufV6NbqXZDM4k10N06NwJnHs+nrVo6TV7v6OXPM75vyc4MsgRJ6+C5ofSJVJX00";

        parameters.cameraDirection = VuforiaLocalizer.CameraDirection.FRONT;
        this.vuforia = ClassFactory.createVuforiaLocalizer(parameters);

        VuforiaTrackables relicTrackables = this.vuforia.loadTrackablesFromAsset("RelicVuMark");
        VuforiaTrackable relicTemplate = relicTrackables.get(0);
        relicTemplate.setName("relicVuMarkTemplate"); // can help in debugging; otherwise not necessary

        relicTrackables.activate();
        while (targetValue == 0) {
            RelicRecoveryVuMark vuMark = RelicRecoveryVuMark.from(relicTemplate);
            if (vuMark != RelicRecoveryVuMark.UNKNOWN) {

                telemetry.addData("VuMark", "%s visible", vuMark);

                OpenGLMatrix pose = ((VuforiaTrackableDefaultListener) relicTemplate.getListener()).getPose();
                telemetry.addData("Pose", format(pose));

                if (pose != null) {
                    VectorF trans = pose.getTranslation();
                    Orientation rot = Orientation.getOrientation(pose, AxesReference.EXTRINSIC, AxesOrder.XYZ, AngleUnit.DEGREES);

                    double tX = trans.get(0);
                    double tY = trans.get(1);
                    double tZ = trans.get(2);

                    double rX = rot.firstAngle;
                    double rY = rot.secondAngle;
                    double rZ = rot.thirdAngle;
                }
            } else {
                telemetry.addData("VuMark", "not visible");
            }

            telemetry.update();
            if (vuMark.toString().equals("LEFT")) {
                telemetry.addLine("Go to left column"); //Encoded move to left column
                targetValue = 1;
            }

            if (vuMark.toString().equals("RIGHT")) {
                telemetry.addLine("Go to right column"); //Encoded move to right column
                targetValue = 2;
            }
            if (vuMark.toString().equals("CENTER")) {
                telemetry.addLine("Go to middle column");
                targetValue = 3;
            }

            //String format(OpenGLMatrix transformationMatrix) {
            //    return (transformationMatrix != null) ? transformationMatrix.formatAsTransform() : "null";
            //}
        }
        telemetry.addData("TARGET VALUE: ", targetValue);
        telemetry.update();
        sleep(1000);
    }
       /* //move forwards
        robot.setDrivePower(0.5, false);
        sleep(1000);
        robot.setDrivePower(0, true);
        //turn left
        robot.turn(0.65, true);
        sleep(1000);
        robot.turn(0, false);

        robot.setDrivePower(0, false);
        //Checks if robot detected center or right pattern, if not it pushes into left box.
        if(targetValue == 2) { //case Center
            robot.setDrivePower(0.5, false);
            sleep(1250);
            robot.setDrivePower(0, false);

            sleep(500);
        }
        else if(targetValue == 3) { //case Right
            robot.setDrivePower(0.5, false);
            sleep(1500);
            robot.setDrivePower(0, false);

            sleep(500);
        }
        else { //case Left

            //move forwards
            robot.setDrivePower(0.5, false);
            sleep(1000);
            robot.setDrivePower(0, false);

            sleep(500);
        }

        robot.arm4.setPosition(.40);
        robot.arm5.setPosition(.60);
        sleep(1500);
        //move backwards
        robot.setDrivePower(0.5, true);
        sleep(250);
        robot.setDrivePower(0, true);
*/


    }
//}
