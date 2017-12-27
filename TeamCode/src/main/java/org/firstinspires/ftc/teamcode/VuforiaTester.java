package org.firstinspires.ftc.teamcode;

/**
 * Created by dina.brustein on 12/27/2017.
 */

import org.firstinspires.ftc.robotcore.external.ClassFactory;
import org.firstinspires.ftc.robotcore.external.matrices.OpenGLMatrix;
import org.firstinspires.ftc.robotcore.external.matrices.VectorF;
import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.robotcore.external.navigation.AxesOrder;
import org.firstinspires.ftc.robotcore.external.navigation.AxesReference;
import org.firstinspires.ftc.robotcore.external.navigation.Orientation;
import org.firstinspires.ftc.robotcore.external.navigation.RelicRecoveryVuMark;
import org.firstinspires.ftc.robotcore.external.navigation.VuforiaLocalizer;
import org.firstinspires.ftc.robotcore.external.navigation.VuforiaTrackable;
import org.firstinspires.ftc.robotcore.external.navigation.VuforiaTrackableDefaultListener;
import org.firstinspires.ftc.robotcore.external.navigation.VuforiaTrackables;
import org.firstinspires.ftc.teamcode.R;
import org.firstinspires.ftc.teamcode.Vuforia;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.HardwareMap;

@Autonomous(name = "Vuforia Auton", group = "LinearOpMode")
public class VuforiaTester extends LinearOpMode {

    public void setup(){

    }

    public void runOpMode(){

        Vuforia vuforia = new Vuforia();

        waitForStart();


        while(!(vuforia.identifyTarget(hardwareMap) != 0) && opModeIsActive()){
            vuforia.identifyTarget(hardwareMap);
        }

        telemetry.addData("Target ", vuforia.identifyTarget(hardwareMap));
    }

}
