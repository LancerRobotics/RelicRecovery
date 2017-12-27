package org.firstinspires.ftc.teamcode;

/**
 * Created by dina.brustein on 12/27/2017.
 */
@Autonomous(name = "Vuforia Auton", group = "LinearOpMode")
public class VuforiaTester extends LinearOpMode {
    HardwareMechanumRobot robot = new HardwareMechanumRobot();

    public void setup(){

    }

    public void runOpMode(){
        robot.init(hardwareMap, true);

        Vuforia vuforia = new Vuforia();
        vuforia.Init_Vuforia();

        waitForStart();

        while(!(vuforia.Identify_Target.equals(""))){
            vuforia.Identify_Target();
        }

        telemetry.addData("Target ", vuforia.Identify_Target(), "detected");
    }

}
