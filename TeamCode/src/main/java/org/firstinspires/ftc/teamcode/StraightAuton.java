package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;

/**
 * Created by david on 12/2/2017.
 */

public class StraightAuton extends LinearOpMode {

    HardwareMechanumRobot robot = new HardwareMechanumRobot();

    public void runOpMode() {

        robot.init(hardwareMap, true);

        waitForStart();

        robot.setDrivePower(0.5, false);
        sleep(1500);
        robot.setDrivePower(0, true);

        sleep(1000);

        robot.setDrivePower(0.5, true);
        sleep(500);
        robot.setDrivePower(0, true);
    }
}
