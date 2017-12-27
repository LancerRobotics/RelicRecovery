package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

@TeleOp(name = "Vuforia_Test_FIRST")

public class VuforiaTesting extends OpMode {

    VuforiaClassTest vuforia = new VuforiaClassTest();

    @Override
    public void init() {
        vuforia.Init_Vuforia();
    }

    @Override
    public void loop() {
        vuforia.Track_Target();
        telemetry.addData( "TX_" , vuforia.Tx );
        telemetry.addData( "TY_" , vuforia.Ty );
        telemetry.addData( "TZ_" , vuforia.Tz );
        telemetry.addData( "Deg_" , vuforia.Deg );
        telemetry.addData( "Name_" , vuforia.target_name );

        telemetry.addData( "RX_" , vuforia.Rx );
        telemetry.addData( "RY_" , vuforia.Ry );
    }

}
