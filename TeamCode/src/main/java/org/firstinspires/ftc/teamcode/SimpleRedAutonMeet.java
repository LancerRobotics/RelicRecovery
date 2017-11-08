package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;

/**
 * Created by david.lin on 11/6/2017.
 */

public class SimpleRedAutonMeet extends LinearOpMode{
    HardwareMechanumRobot robot = new HardwareMechanumRobot();
    public void setup(){

    }

    public void runOpMode(){
        waitForStart();

        robot.init(hardwareMap, true);

        robot.setDrivePower(0.86, false);
        sleep(500);
        robot.setDrivePower(0, true);
        robot.turn(0.86, false);
        sleep(100);
        robot.setDrivePower(0, true);
        robot.setDrivePower(0.10, true);
        sleep(100);
        robot.setDrivePower(0, true);

    }
}
