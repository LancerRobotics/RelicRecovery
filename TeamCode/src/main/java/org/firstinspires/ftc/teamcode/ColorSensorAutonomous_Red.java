/**
package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.ColorSensor;

@Autonomous
//@Disabled
public class ColorSensorAutonomous_Red extends LinearOpMode {
    HardwareMechanumRobot robot = new HardwareMechanumRobot();
    //ColorSensor color;

    public void setup(){

    }

    public void runOpMode(){
        robot.init(hardwareMap, true);

//
        //for a CR Servo, dont set the position to anything
        //robot.jewel_hitter.setPosition(.3);
        //robot.arm1.setPosition(robot.ARM_1_CLOSED);

        waitForStart();

        sleep(1000);
        sleep(3000);
        //MAKE THE JEWEL HITTER MOVE FIRST, THEN THE OTHER 2 JEWEL SERVOS

        telemetry.addData("Red: ", robot.color_sensor.red());
        telemetry.update();

        telemetry.addData("Blue: ", robot.color_sensor.blue());
        telemetry.update();
        sleep(1000);

        //I added "-3" because the red is much stronger than blue
        if(robot.color_sensor.red()-3 > robot.color_sensor.blue()){
            telemetry.addLine("Will hit red jewel");
            telemetry.update();
            //MAKE RED AND BLUE AUTONS!!!
            robot.jewel_hitter.setPower(-.4);
            sleep(400);
            robot.jewel_hitter.setPower(0);
        }
        else {
            telemetry.addLine("Will hit blue jewel");
            telemetry.update();
            robot.jewel_hitter.setPower(.4);
            sleep(400);
            robot.jewel_hitter.setPower(0);
        }
        sleep(1000);
//        robot.jewel1.setPosition(.65);
        sleep(5000);
    }
}
*/