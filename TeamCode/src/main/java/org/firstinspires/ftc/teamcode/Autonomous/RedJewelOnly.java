package org.firstinspires.ftc.teamcode.Autonomous;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;

import org.firstinspires.ftc.teamcode.HardwareMechanumRobot;

/**
 * Created by david on 12/10/2017.
 */
//@Autonomous

public class RedJewelOnly extends LinearOpMode {
    HardwareMechanumRobot robot = new HardwareMechanumRobot();

    public void setup(){

    }

    public void runOpMode(){
        robot.init(hardwareMap, true);
//
        //for a CR Servo, dont set the position to anything
        //robot.jewel_hitter.setPosition(.3);
        //robot.arm1.setPosition(robot.ARM_1_CLOSED);

        waitForStart();

//        robot.jewel_hitter.setPower(-0.5);
//        sleep(300);
//        robot.jewel_hitter.setPower(0);
//
//        sleep(500);
//
//        robot.jewel0.setPower(-0.5);
//        sleep(1000);
//        robot.jewel0.setPower(0);

        //MAKE THE JEWEL HITTER MOVE FIRST, THEN THE OTHER 2 JEWEL SERVOS

        telemetry.addData("Blue: ", robot.color.blue());
        telemetry.addData("Red: ", robot.color.red());
        telemetry.update();

        telemetry.update();
        sleep(1000);

        //I added "-3" because the red is much stronger than blue
        if(robot.color.red()-3 > robot.color.blue()){
            telemetry.addLine("Will hit other jewel");
            telemetry.update();
//            robot.jewel_hitter.setPower(.4);
//            sleep(400);
        }
        else {
            telemetry.addLine("Will hit this jewel");
            telemetry.update();
            //MAKE RED AND BLUE AUTONS!!!
//            robot.jewel_hitter.setPower(-.4);
//            sleep(400);
//            robot.jewel_hitter.setPower(0);
        }

//        robot.jewel_hitter.setPower(0);
//
//        sleep(1000);
////        robot.jewel0.setPosition(.65);
//        robot.jewel1.setPosition(.65);
//        sleep(500);
//
//
//        robot.jewel0.setPower(0.5);
//        sleep(2000);
//        robot.jewel0.setPower(0);

        sleep(500);

    }
}
