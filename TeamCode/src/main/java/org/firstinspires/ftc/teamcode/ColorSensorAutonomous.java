package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.ColorSensor;

@Autonomous
//@Disabled
public class ColorSensorAutonomous extends LinearOpMode {
    HardwareMechanumRobot robot = new HardwareMechanumRobot();
    ColorSensor color;

    public void setup(){

    }

    public void runOpMode(){
        robot.init(hardwareMap, true);

        waitForStart();

        robot.jewel0.setPosition(.4);
        robot.jewel1.setPosition(.4);

        sleep(500);

        robot.setDrivePower(.2, false);
        sleep(250);
        robot.setDrivePower(0, true);

        sleep(250);
        //Fix this part - the movers have to move at the same time! - multithreading...
        robot.jewel0.setPosition(.9);
        robot.jewel1.setPosition(.9);

        color = robot.color;

        telemetry.addData("Red: ", color.red());
        telemetry.update();

        telemetry.addData("Blue: ", color.blue());
        telemetry.update();

        if(color.red() > color.blue()){
            telemetry.addLine("Will hit red jewel");
        }
        else {
            telemetry.addLine("Will hit blue jewel");
        }
    }
}
