package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.HardwareMap;

import org.firstinspires.ftc.robotcore.external.ClassFactory;
import org.firstinspires.ftc.robotcore.external.matrices.OpenGLMatrix;
import org.firstinspires.ftc.robotcore.external.matrices.VectorF;
import org.firstinspires.ftc.robotcore.external.navigation.*;
import org.firstinspires.ftc.robotcore.external.navigation.RelicRecoveryVuMark;
import org.firstinspires.ftc.robotcore.external.navigation.VuforiaLocalizer;
import org.firstinspires.ftc.robotcore.external.navigation.VuforiaTrackable;
import org.firstinspires.ftc.robotcore.external.navigation.VuforiaTrackableDefaultListener;
import org.firstinspires.ftc.robotcore.external.navigation.VuforiaTrackables;
import org.firstinspires.ftc.robotcore.external.*;
import org.firstinspires.ftc.robotcore.external.Telemetry;

/**
 * Created by dina.brustein on 11/22/2017.
 */
@Autonomous(name="Vuforia", group = "Linear OpMode")
public class Vuforia {

    // Constructor
    public Vuforia() {
    }

    VuforiaTrackables targets;
    VuforiaTrackableDefaultListener listener;
    VuforiaLocalizer vuforia_localizer;
    float Tx,Tz,Ty,Deg;
    float Rx,Ry;
    String target_name;
    private float offset=0;
    private float mm_to_Inch = 25.4f;

    //Vuforia Set Up
    public int identifyTarget(HardwareMap h) {

        // Vuforia LicenseKey Link .. https://developer.vuforia.com/user/register
        // Basic vuforia set up perams....
        VuforiaLocalizer.Parameters params = new VuforiaLocalizer.Parameters(R.id.cameraMonitorViewId);
        params.vuforiaLicenseKey = "AWxF7cr/////AAAAGeD4IygGuUa9keX3IQHxtEgFno0SNHQ99jVMYjRFlTKEYQ69ErBzstMqNC2+Rblp4NIKhEH9iYoD/Hr+L0W/sUaV4M8jP1tq0Z2rQgG9hVAS0ISc7+aoWBufWxmVNEZb604khShVrI9N4wKPhPEii6aju88ni///TFebYOPlLGcZ5IQSaJisYPboqYmyobXYq5hbf9Fg1hO3tVozWMrwnf8ejb4n6fn8RN1dJCqvh6lrf03cHyOuOSF6lvv3zy2fC0fJc+mJtLo03o1EJAjZzPCQEa1ommGUGnUZheZD+z5Cl/vK9X7HuZmGkPCRgwjw7T6Qpb0hjrKfLxYYdYY79tUy3OM/Z5ez73rZYTY6fulX";//?
        params.cameraDirection = VuforiaLocalizer.CameraDirection.FRONT; //FRONT or BACK (which camera you want to use). THIS IS THE ONLY PART OF THE PROGRAM YOU NEED TO CHANGE.
        params.cameraMonitorFeedback = VuforiaLocalizer.Parameters.CameraMonitorFeedback.AXES; //You can choose CameraMonitorFeedback.whatever, but axes is just easier
        vuforia_localizer = ClassFactory.createVuforiaLocalizer(params);


        //load tragets and assign them names....
        targets = vuforia_localizer.loadTrackablesFromAsset("RelicVuMark");

        VuforiaTrackable relicTemplate = targets.get(0);
        relicTemplate.setName("relicVuMarkTemplate");

        RelicRecoveryVuMark vuMark = RelicRecoveryVuMark.from(relicTemplate);

        if (vuMark == RelicRecoveryVuMark.LEFT) {
            return 1;
        }
        else if(vuMark == RelicRecoveryVuMark.CENTER){
            return 2;
        }
        else if(vuMark == RelicRecoveryVuMark.RIGHT){
            return 3;
        }
        else {
            return 0;
        }

    }
}
