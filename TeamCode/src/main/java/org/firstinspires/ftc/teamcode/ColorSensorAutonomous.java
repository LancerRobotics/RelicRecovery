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

        sleep(500);

        robot.setDrivePower(.2, false);
        sleep(250);
        robot.setDrivePower(0, true);

        sleep(500);



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
