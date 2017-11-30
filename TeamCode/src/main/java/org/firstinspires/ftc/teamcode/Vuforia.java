package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;

import org.firstinspires.ftc.robotcore.external.ClassFactory;
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

public class Vuforia {

    public Vuforia() {

    }

    VuforiaTrackables targets;
    VuforiaTrackableDefaultListener vlistener;
    VuforiaLocalizer vuforia;
    float Tx, Tz, Ty, Deg;
    float Rx, Ry;
    String targetName;
    private float offset = 0;
    private float mmToInches = 25.4f;

    public void initVuforia() {
        VuforiaLocalizer.Parameters params = new VuforiaLocalizer.Parameters(R.id.cameraMonitorViewId);
        params.vuforiaLicenseKey = "AWxF7cr/////AAAAGeD4IygGuUa9keX3IQHxtEgFno0SNHQ99jVMYjRFlTKEYQ69ErBzstMqNC2+Rblp4NIKhEH9iYoD/Hr+L0W/sUaV4M8jP1tq0Z2rQgG9hVAS0ISc7+aoWBufWxmVNEZb604khShVrI9N4wKPhPEii6aju88ni///TFebYOPlLGcZ5IQSaJisYPboqYmyobXYq5hbf9Fg1hO3tVozWMrwnf8ejb4n6fn8RN1dJCqvh6lrf03cHyOuOSF6lvv3zy2fC0fJc+mJtLo03o1EJAjZzPCQEa1ommGUGnUZheZD+z5Cl/vK9X7HuZmGkPCRgwjw7T6Qpb0hjrKfLxYYdYY79tUy3OM/Z5ez73rZYTY6fulX";
        params.cameraDirection = VuforiaLocalizer.CameraDirection.FRONT;
        params.cameraMonitorFeedback = VuforiaLocalizer.Parameters.CameraMonitorFeedback.AXES;
        vuforia = ClassFactory.createVuforiaLocalizer(params);

        targets = vuforia.loadTrackablesFromAsset("RelicVuMark");
        VuforiaTrackable template = targets.get(0);
    }

}

