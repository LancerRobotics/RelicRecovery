package org.firstinspires.ftc.teamcode;

/**
 * Created by spork on 9/30/2017.
 */
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.Gamepad;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.util.Range;


@TeleOp(name="EK_TankDrive", group="TankDrive")
public class EK_17_18 extends OpMode {

    private DcMotor leftFrontMotor = null;
    private DcMotor leftBackMotor = null;
    private DcMotor rightFrontMotor = null;
    private DcMotor rightBackMotor = null;
    private Servo leftArmservo = null;
    private Servo rightArmservo  = null;


    public void init() {
        // Run once when driver hits "INIT".
        // Robot setup code goes here.
        leftFrontMotor = hardwareMap.dcMotor.get("leftFront");
        leftBackMotor = hardwareMap.dcMotor.get("leftBack");
        rightFrontMotor = hardwareMap.dcMotor.get("rightFront");
        rightBackMotor = hardwareMap.dcMotor.get("rightBack");
        leftArmservo = hardwareMap.servo.get("leftArm");
        rightArmservo = hardwareMap.servo.get("rightArm");

        leftArmservo.setPosition(0);
        rightArmservo.setPosition(180);
        leftFrontMotor.setPower(0);
        leftBackMotor.setPower(0);
        rightFrontMotor.setPower(0);
        rightBackMotor.setPower(0);
    }

    public void start() {
        // Run once when driver hits "PLAY".
        // Robot start-of-match code goes here.

    }

    public void loop() {
        // Code executed continuously until robot end.
        double right = -gamepad1.right_stick_y;
        double left = gamepad1.left_stick_y;
        boolean Button_x = gamepad1.x;
        boolean Button_b = gamepad1.b;

        if (Button_x == true){
            leftArmservo.setPosition(0);
            rightArmservo.setPosition(90);
        }
        if (Button_b == true){
            leftArmservo.setPosition(45 );
            rightArmservo.setPosition(180);
        }


        leftFrontMotor.setPower(left);
        leftBackMotor.setPower(left);
        rightFrontMotor.setPower(right);
        rightBackMotor.setPower(right);
    }

    public void stop() {
        // Run once when driver hits "STOP" or time elapses.
        // Robot end-of-match code goes here.
        leftFrontMotor.setPower(0);
        leftBackMotor.setPower(0);
        rightFrontMotor.setPower(0);
        rightBackMotor.setPower(0);
        leftArmservo.setPosition(0);
        rightArmservo.setPosition(180);
    }
}

