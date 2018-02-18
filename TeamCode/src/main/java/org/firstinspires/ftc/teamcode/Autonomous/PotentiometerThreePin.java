package org.firstinspires.ftc.teamcode.Autonomous;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.AnalogInput;

/**
 * Created by Paspuleti on 2/16/2018.
 */

@Autonomous(name = "Potentiometer for three pin test", group = "Autonomous")
public class PotentiometerThreePin extends LinearOpMode {
    public void runOpMode() {
        AnalogInput potentiometer;
        potentiometer = hardwareMap.analogInput.get("potentiometer");
        waitForStart();
        while (opModeIsActive()) {
            // Reading voltage
            double voltReading = (float) potentiometer.getVoltage();
            //convert voltage to distance (cm)

            double percentTurned = voltReading/5 * 100;
            /*
            while(percentTurned < 0.8) {
                motor.setPower(0.3);
            }
            motor.setPower(0);
*/
            telemetry.addData("time", "elapsed time: " + time);
            telemetry.addData("raw val", "voltage:  " + Double.toString(voltReading));
            // this is our calculated value
            telemetry.addData("PercentRot", "percent: " + Double.toString(percentTurned));
            telemetry.update();
        }
    }
}
