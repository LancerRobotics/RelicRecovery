package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.ColorSensor;


/**
 * This file contains an minimal example of a Linear "OpMode". An OpMode is a 'program' that runs in either
 * the autonomous or the teleop period of an FTC match. The names of OpModes appear on the menu
 * of the FTC Driver Station. When an selection is made from the menu, the corresponding OpMode
 * class is instantiated on the Robot Controller and executed.
 *
 * This particular OpMode just executes a basic Tank Drive Teleop for a two wheeled robot
 * It includes all the skeletal structure that all linear OpModes contain.
 *
 * Use Android Studios to Copy this Class, and Paste it into your team's code folder with a new name.
 * Remove or comment out the @Disabled line to add this opmode to the Driver Station OpMode list
 */

@Autonomous
//@Disabled
public class ColorSensorAutonomous extends LinearOpMode {
    public void setup(){

    }

    public void runOpMode(){
        HardwareMechanumRobot robot = new HardwareMechanumRobot;

        robot.init(hardwareMap, true);
        waitForStart();

        ColorSensor color = robot.color;

        telemetry.addData("Red: ", color.red());
        telemetry.update();

        telemetry.addData("Blue: ", color.blue());
        telemetry.update();

        if(color.red() > color.blue()){
            telemetry.addLine("Will hit other jewel");
        }
        else {
            telemetry.addLine("Will hit this jewel");
        }
    }
}