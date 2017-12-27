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
    public void Init_Vuforia(){

        // Vuforia LicenseKey Link .. https://developer.vuforia.com/user/register
        // Basic vuforia set up perams....
        VuforiaLocalizer.Parameters params = new VuforiaLocalizer.Parameters(R.id.cameraMonitorViewId);
        params.vuforiaLicenseKey = "AWxF7cr/////AAAAGeD4IygGuUa9keX3IQHxtEgFno0SNHQ99jVMYjRFlTKEYQ69ErBzstMqNC2+Rblp4NIKhEH9iYoD/Hr+L0W/sUaV4M8jP1tq0Z2rQgG9hVAS0ISc7+aoWBufWxmVNEZb604khShVrI9N4wKPhPEii6aju88ni///TFebYOPlLGcZ5IQSaJisYPboqYmyobXYq5hbf9Fg1hO3tVozWMrwnf8ejb4n6fn8RN1dJCqvh6lrf03cHyOuOSF6lvv3zy2fC0fJc+mJtLo03o1EJAjZzPCQEa1ommGUGnUZheZD+z5Cl/vK9X7HuZmGkPCRgwjw7T6Qpb0hjrKfLxYYdYY79tUy3OM/Z5ez73rZYTY6fulX";//?
        params.cameraDirection = VuforiaLocalizer.CameraDirection.FRONT; //FRONT or BACK (which camera you want to use). THIS IS THE ONLY PART OF THE PROGRAM YOU NEED TO CHANGE.
        params.cameraMonitorFeedback = VuforiaLocalizer.Parameters.CameraMonitorFeedback.AXES; //You can choose CameraMonitorFeedback.whatever, but axes is just easier
        vuforia_localizer = ClassFactory.createVuforiaLocalizer(params);

        //load tragets and assign them names....
        targets = vuforia_localizer.loadTrackablesFromAsset("RelicVuMark");
        targets.get(0).setName("LEFT");
        targets.get(1).setName("CENTER");
        targets.get(2).setName("RIGHT");

        // set up targets to their field positions....
        // target_field_position( target # , field X position , field Y position , target Y rotation )
        // Rotation is in a counter clockwise rotation
        // all coords start from bottom left of field if viewed from top down .

        target_field_position( 0 , 0 , 0 , 0 );//wheels ?
        target_field_position( 1 , 0 , 0 , 0 );//tools ?
        target_field_position( 2 , 47 , 0 , 0 );//legos ?
        target_field_position( 3 , 95 , 0 , 0 );//gears ?

        //Setting Phone position is not needed to work but can be used for setting phones offset to robots center .
        OpenGLMatrix phoneLocationOnRobot = OpenGLMatrix
                .translation(0,0,0) // values should be in mm .( left/right , up/down , forward/backward ).
                .multiplied(Orientation.getRotationMatrix(AxesReference.EXTRINSIC, AxesOrder.YZY,AngleUnit.DEGREES, -90, 0, 0));
        ((VuforiaTrackableDefaultListener)targets.get(0).getListener()).setPhoneInformation(phoneLocationOnRobot, params.cameraDirection);
        ((VuforiaTrackableDefaultListener)targets.get(1).getListener()).setPhoneInformation(phoneLocationOnRobot, params.cameraDirection);
        ((VuforiaTrackableDefaultListener)targets.get(2).getListener()).setPhoneInformation(phoneLocationOnRobot, params.cameraDirection);
        ((VuforiaTrackableDefaultListener)targets.get(3).getListener()).setPhoneInformation(phoneLocationOnRobot, params.cameraDirection);

        targets.activate();

    }

    //Vuforia Track Target
    public void Track_Target(){

        for (VuforiaTrackable targ : targets ){

            //Target position to robot
            listener = (VuforiaTrackableDefaultListener) targ.getListener();
            OpenGLMatrix pose = listener.getPose();

            if( pose != null) {

                VectorF Tdata = pose.getTranslation();
                Tx = Tdata.get(0) / mm_to_Inch;
                Ty = Tdata.get(1) / mm_to_Inch;
                Tz = Tdata.get(2) / mm_to_Inch;

                Tx=Tx+offset;

                Orientation orientation = Orientation.getOrientation(pose, AxesReference.EXTRINSIC, AxesOrder.XYZ, AngleUnit.DEGREES);
                Deg = orientation.secondAngle;

                target_name = targ.getName();

            }

            //Robot position on field
            OpenGLMatrix robotLoc = listener.getUpdatedRobotLocation();
            if (robotLoc != null) {

                VectorF Rdata = robotLoc.getTranslation();
                Rx = Rdata.get(0) / mm_to_Inch;
                Ry = Rdata.get(1) / mm_to_Inch;
            }


        }

    }

    // this is a helper function to set targets field positions.
    private void target_field_position( int target_num , float tx_position , float ty_position , int ty_angle ){
        tx_position = (tx_position * mm_to_Inch);
        ty_position = (ty_position * mm_to_Inch);
        OpenGLMatrix TargetLocationOnField = OpenGLMatrix
                .translation(tx_position , ty_position , 0 )
                .multiplied(Orientation.getRotationMatrix(AxesReference.EXTRINSIC, AxesOrder.XZX,AngleUnit.DEGREES, 90 , ty_angle , 0));
        targets.get(target_num).setLocation(TargetLocationOnField);
    }

    public String Identify_Target(){
        RelicRecoveryVuMark vuMark = RelicRecoveryVuMark.from(relicTemplate);

        if(vuMark != RelicRecoveryVuMark.UNKNOWN){
            return vuMark.getName();
        }

        return "";
    }

}
