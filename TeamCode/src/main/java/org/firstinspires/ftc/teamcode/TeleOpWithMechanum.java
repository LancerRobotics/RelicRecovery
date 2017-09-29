package org.firstinspires.ftc.teamcode;

/**
 * Created by dina.brustein on 9/27/2017.
 */
import android.content.BroadcastReceiver;
import android.content.ComponentName;
import android.content.ContentResolver;
import android.content.Context;
import android.content.Intent;
import android.content.IntentFilter;
import android.content.IntentSender;
import android.content.ServiceConnection;
import android.content.SharedPreferences;
import android.content.pm.ApplicationInfo;
import android.content.pm.PackageManager;
import android.content.res.AssetManager;
import android.content.res.Configuration;
import android.content.res.Resources;
import android.database.DatabaseErrorHandler;
import android.database.sqlite.SQLiteDatabase;
import android.graphics.Bitmap;
import android.graphics.drawable.Drawable;
import android.net.Uri;
import android.os.Bundle;
import android.os.Handler;
import android.os.Looper;
import android.os.UserHandle;
import android.support.annotation.Nullable;
import android.view.Display;

import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.util.Range;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.Gamepad;
import com.qualcomm.robotcore.hardware.HardwareMap;

import java.io.File;
import java.io.FileInputStream;
import java.io.FileNotFoundException;
import java.io.FileOutputStream;
import java.io.IOException;
import java.io.InputStream;


public class TeleOpWithMechanum {

    MechanumRobot robot = new MechanumRobot();

    public void runOpMode(){

        Gamepad gamepad1 = new Gamepad();
        Gamepad gamepad2 = new Gamepad();
        
        //robot.init(hardwareMap);

        if(gamepad1.left_stick_x < -0.15){
            robot.strafe(0.86,true);
        }

        if(gamepad1.left_stick_x > 0.15){
            robot.strafe(0.86,false);
        }

        if(gamepad1.left_stick_y > 0.15){
            robot.setDrivePower(0.8,false);
        }

        if(gamepad1.left_stick_y < -0.15){
            robot.setDrivePower(0.8,true);
        }

        if(gamepad1.right_stick_x > 0.15){
            robot.turn(0.35,false);
        }

        if(gamepad1.left_stick_x < -0.15){
            robot.turn(0.35, true);
        }
    }

}
