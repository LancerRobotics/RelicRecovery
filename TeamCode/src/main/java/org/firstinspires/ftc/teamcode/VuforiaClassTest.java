package org.firstinspires.ftc.teamcode;

import org.firstinspires.ftc.robotcore.external.ClassFactory;
import org.firstinspires.ftc.robotcore.external.matrices.OpenGLMatrix;
import org.firstinspires.ftc.robotcore.external.matrices.VectorF;
import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.robotcore.external.navigation.AxesOrder;
import org.firstinspires.ftc.robotcore.external.navigation.AxesReference;
import org.firstinspires.ftc.robotcore.external.navigation.Orientation;
import org.firstinspires.ftc.robotcore.external.navigation.VuforiaLocalizer;
import org.firstinspires.ftc.robotcore.external.navigation.VuforiaTrackable;
import org.firstinspires.ftc.robotcore.external.navigation.VuforiaTrackableDefaultListener;
import org.firstinspires.ftc.robotcore.external.navigation.VuforiaTrackables;
import org.firstinspires.ftc.teamcode.R;

public class VuforiaClassTest{

    // Constructor
    public VuforiaClassTest() {
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
        params.vuforiaLicenseKey = "ATsODcD/////AAAAAVw2lR...d45oGpdljdOh5LuFB9nDNfckoxb8COxKSFX";//?
        params.cameraDirection = VuforiaLocalizer.CameraDirection.FRONT; //FRONT / BACK
        params.cameraMonitorFeedback = VuforiaLocalizer.Parameters.CameraMonitorFeedback.AXES;
        vuforia_localizer = ClassFactory.createVuforiaLocalizer(params);

        //load tragets and assign them names....
        targets = vuforia_localizer.loadTrackablesFromAsset("RelicVuMark");
        targets.get(0).setName("?");
        targets.get(1).setName("??");
        targets.get(2).setName("???");
        targets.get(3).setName("????");

        // set up targets to there field positions....
        // target_field_position( target # , field X position , field Y position , target Y rotation )
        // Rotation is in a counter clockwise rotation
        // all coords start from bottom left of field if viewed from top down .

        /*
        Field Example of Target locations starting with target 0 to 3
        All values are in inches.
        Ez. X value inches , Y value inches , rotation

        *  *  *  2  *  *  *  3  *  *  *  *  *
        *    142,47,0    142,95,0           *
        *                                   *
        1  0,107,90                         * //The 90
        *                                   *
        *                                   *
        *                 C                 *
        0  0,59,90                          *
        *                                   *
        *                                   *
        *                                   *
        *          Robot                    *
        *  *  *  *  *  *  *  *  *  *  *  *  *
        /\
        start counting values from here ( bottom , left )= 0,0,0


        */


        target_field_position( 0 , 0 , 59 , 00 );//wheels pic ? //pic is (target number, x, y, y rotation) NO Z! (that's up)
        target_field_position( 1 , 0 , 0 , 0 );//tools pic ?
        target_field_position( 2 , 47 , 0 , 0 );//legos pic ?
        target_field_position( 3 , 95 , 0 , 0 );//gears pic ?

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

    //Vuforia Track Target Method
    public void Track_Target(){

        for (VuforiaTrackable targ : targets ){

            //Target position to robot
            listener = (VuforiaTrackableDefaultListener) targ.getListener();
            OpenGLMatrix pose = listener.getPose(); // pose = position

            if( pose != null) {

                VectorF Tdata = pose.getTranslation();
                Tx = Tdata.get(0) / mm_to_Inch;//Translation Information = T
                Ty = Tdata.get(1) / mm_to_Inch;
                Tz = Tdata.get(2) / mm_to_Inch;

                Tx=Tx+offset; //if the phone is not dead center, does the same as phoneLocationOnRobot method

                Orientation orientation = Orientation.getOrientation(pose, AxesReference.EXTRINSIC, AxesOrder.XYZ, AngleUnit.DEGREES);
                Deg = orientation.secondAngle;

                target_name = targ.getName();

            }

            //Robot position on field
            OpenGLMatrix robotLoc = listener.getUpdatedRobotLocation();
            if (robotLoc != null) {

                VectorF Rdata = robotLoc.getTranslation();
                Rx = Rdata.get(0) / mm_to_Inch; // R = Robot, Rx = Robot x position
                Ry = Rdata.get(1) / mm_to_Inch; // Rz is up/down, which is not needed
            }


        }

    }

    // this is a helper function to set targets field positions.
    private void target_field_position( int target_num , float tx_position , float ty_position , int ty_angle ){
        tx_position = (tx_position * mm_to_Inch);
        ty_position = (ty_position * mm_to_Inch);
        OpenGLMatrix TargetLocationOnField = OpenGLMatrix
                .translation(tx_position , ty_position , 0 )
                .multiplied(Orientation.getRotationMatrix(AxesReference.EXTRINSIC, AxesOrder.XZX,AngleUnit.DEGREES, 90 , ty_angle , 0));//Change degrees, 90 is vertical, 0 is flat on the ground
        targets.get(target_num).setLocation(TargetLocationOnField);
    }
}
