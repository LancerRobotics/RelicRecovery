package org.firstinspires.ftc.teamcode.Autonomous;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;

import org.firstinspires.ftc.teamcode.HardwareMechanumRobot;

/**
 * Created by david on 12/2/2017.
 */
@Autonomous(name="STRAIGHT AUTON - USE THIS", group="Linear Opmode")
public class StraightAuton extends LinearOpMode {

    HardwareMechanumRobot robot = new HardwareMechanumRobot();

    public void runOpMode() {

        robot.init(hardwareMap, true);

        waitForStart();
        //Clamp onto glyph
        robot.autonGlyphL.setPosition(0);
        robot.autonGlyphR.setPosition(1);
        sleep(500);

        //Drive straight
        robot.setDrivePower(0.5, false);
        sleep(1500);
        robot.setDrivePower(0, true);
        sleep(1000);

        //Open clamp
        robot.autonGlyphL.setPosition(1);
        robot.autonGlyphR.setPosition(0);
        sleep(500);

        //Drive backwards
        robot.setDrivePower(0.35, true);
        sleep(500);
        robot.setDrivePower(0, true);
    }
}
