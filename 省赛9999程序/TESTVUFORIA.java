package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.OpMode;

import org.firstinspires.ftc.robotcore.external.ClassFactory;
import org.firstinspires.ftc.robotcore.external.navigation.RelicRecoveryVuMark;
import org.firstinspires.ftc.robotcore.external.navigation.VuforiaLocalizer;
import org.firstinspires.ftc.robotcore.external.navigation.VuforiaTrackable;
import org.firstinspires.ftc.robotcore.external.navigation.VuforiaTrackables;

import java.util.ArrayList;
import java.util.List;

/**
 * Created by sdghjqr01 on 2018/1/28.
 */

public class TESTVUFORIA {
    VuforiaLocalizer vuforia;
    private double VuMarkIdentification;
    double VuMarkIdentificationF;
    double VuMarkIdentificationFR2;
    double
            VuMarkIdentificationB;
    double VuMarkIdentificationB2;
    double VuMarkIdentificationB2FOUR;
    double VuMarkIdentificationR2FOUR;
    double VuMarkIdentificationR2PONG;
    double VuMarkIdentificationB2PONG;
    double VuMarkIdentificationR2SeO3;
    private double result;
    double CHANGE = 100;
    double CHANGE1;
    private OpMode myOpMode;
    private TESTROBOT myRobot;
    private VuforiaTrackables targets;
    //private ElapsedTime runtime = new ElapsedTime();

    public TESTVUFORIA() {
        targets = null;
        VuMarkIdentification = 0;
        VuMarkIdentificationF = 0;
        double VuMarkIdentificationFR2;
        result = 0;
    }

    public void initializeVuforia(OpMode opMode, TESTROBOT robot) {
        myOpMode = opMode;
        myRobot = robot;
        int cameraMonitorViewId = myOpMode.hardwareMap.appContext.getResources().getIdentifier("cameraMonitorViewId", "id", myOpMode.hardwareMap.appContext.getPackageName());

        VuforiaLocalizer.Parameters parameters = new VuforiaLocalizer.Parameters(cameraMonitorViewId);

        parameters.vuforiaLicenseKey = "AWZ1FLb/////AAAAGU5Ty2dHZETahEyraiiGxnEQecKVoS80GUybCIaO/G9VaSocMoYwajkfThEQAnKSjRKBKImxk4y6DQ/rwvaiEHEV3Zw5gFDJrRzwioETWkY6VxHxto9LcE8kyU+gek6uVaUKbYHMWijhGvvlzQ+XgeRbjssMmE7/usViVHtUKquiE+pgRXK5l6N872+b9Rfgj++crhShbWGDZ5pY3amumchaeDWh8hpBKkWvEgAfa5SH6gkomgK28q2gd8ibiRIwEvEY8vQCiv+wzpfDFfz2qHyjGNEKh7rV4eTz3kf3mBmotu37UZzTyUFCsqXJo/LJTRrLPY9pxuQauenDF0s/T1dYxZeVS0cbMBQ6JpssuqYw";

        parameters.cameraDirection = VuforiaLocalizer.CameraDirection.FRONT;

        this.vuforia = ClassFactory.createVuforiaLocalizer(parameters);

        targets = this.vuforia.loadTrackablesFromAsset("RelicVuMark");

        targets.get(0).setName("relic");

        List<VuforiaTrackable> allTrackables = new ArrayList<VuforiaTrackable>();
        allTrackables.addAll(targets);
    }

    public void activeTracking() {
        if (targets != null)
            targets.activate();
    }

    public void addNavgation() {
        VuforiaTrackable target = targets.get(0);
        RelicRecoveryVuMark vuMark = RelicRecoveryVuMark.from(target);

        if (vuMark == RelicRecoveryVuMark.UNKNOWN) {
            myOpMode.telemetry.addData("VuMark", "not visible");
        } else if (vuMark != null && vuMark == RelicRecoveryVuMark.CENTER) {
            myOpMode.telemetry.addData("VuMark", "CENTER");
        } else if (vuMark != null && vuMark == RelicRecoveryVuMark.LEFT) {
            myOpMode.telemetry.addData("VuMark", "LEFT");
        } else if (vuMark != null && vuMark == RelicRecoveryVuMark.RIGHT) {
            myOpMode.telemetry.addData("VuMark", "RIGHT");
        }
        myOpMode.telemetry.update();
    }


    public void IdentifyR()     //special for REDteam
    {
        VuforiaTrackable target = targets.get(0);
        RelicRecoveryVuMark vuMark = RelicRecoveryVuMark.from(target);

        if (vuMark != null && vuMark == RelicRecoveryVuMark.CENTER) {
            VuMarkIdentificationF = 272;              //440
            CHANGE1 = 50;//65
        } else if (vuMark != null && vuMark == RelicRecoveryVuMark.LEFT) {
            VuMarkIdentificationF = 440;//47.0//610
            CHANGE1 = -120;
        } else if (vuMark != null && vuMark == RelicRecoveryVuMark.RIGHT) {
            VuMarkIdentificationF = 30;//83.8//270
            CHANGE1 = 100;
        } else {
            VuMarkIdentificationF = 272;//270
            CHANGE1 = 50;

        }

    }

    public void IdentifyR2()     //special for REDteam
    {
        VuforiaTrackable target = targets.get(0);
        RelicRecoveryVuMark vuMark = RelicRecoveryVuMark.from(target);

        if (vuMark != null && vuMark == RelicRecoveryVuMark.CENTER) {
            VuMarkIdentificationFR2 = 123;
        } else if (vuMark != null && vuMark == RelicRecoveryVuMark.LEFT) {
            VuMarkIdentificationFR2 = 144;//47.0
        } else if (vuMark != null && vuMark == RelicRecoveryVuMark.RIGHT) {
            VuMarkIdentificationFR2 = 104;//83.8
        } else {
            VuMarkIdentificationFR2 = 104;
        }

    }

    public void IdentifyB()     //special for BLUEteam
    {
        VuforiaTrackable target = targets.get(0);
        RelicRecoveryVuMark vuMark = RelicRecoveryVuMark.from(target);

        if (vuMark != null && vuMark == RelicRecoveryVuMark.CENTER) {
            VuMarkIdentificationB = 241;
            CHANGE1 = 65;
        } else if (vuMark != null && vuMark == RelicRecoveryVuMark.LEFT) {
            VuMarkIdentificationB = 70;//47.0
            CHANGE1 = -100;
        } else if (vuMark != null && vuMark == RelicRecoveryVuMark.RIGHT) {
            VuMarkIdentificationB = 410;//83.8
            CHANGE1 = 100;
        } else {
            VuMarkIdentificationB = 240;
            CHANGE1 = 65;
        }

    }

    public void IdentifyB2()     //special for BLUEteam
    {
        VuforiaTrackable target = targets.get(0);
        RelicRecoveryVuMark vuMark = RelicRecoveryVuMark.from(target);

        if (vuMark != null && vuMark == RelicRecoveryVuMark.CENTER) {
            VuMarkIdentificationB2 = 123;              //123
        } else if (vuMark != null && vuMark == RelicRecoveryVuMark.LEFT) {
            VuMarkIdentificationB2 = 104;//47.0          //104
        } else if (vuMark != null && vuMark == RelicRecoveryVuMark.RIGHT) {
            VuMarkIdentificationB2 = 144;//83.8            //144
        } else {
            VuMarkIdentificationB2 = 106;
        }
    }

    public void IdentifyB2FOUR()     //special for BLUEteam
    {
        VuforiaTrackable target = targets.get(0);
        RelicRecoveryVuMark vuMark = RelicRecoveryVuMark.from(target);

        if (vuMark != null && vuMark == RelicRecoveryVuMark.CENTER) {
            VuMarkIdentificationB2FOUR = 855;      //870
            CHANGE = -90;//65
            //123
        } else if (vuMark != null && vuMark == RelicRecoveryVuMark.LEFT) {
            VuMarkIdentificationB2FOUR = 603;//634//650//83.8            //144//670
            CHANGE = -120;//100
        } else if (vuMark != null && vuMark == RelicRecoveryVuMark.RIGHT) {
            VuMarkIdentificationB2FOUR = 1000;//1047//47.0          //104   //1170//947
            CHANGE = 100;
        } else {
            VuMarkIdentificationB2FOUR = 857;      //870
            CHANGE = -90;
        }
    }

    public void IdentifyR2FOUR()     //special for BLUEteam
    {

        VuforiaTrackable target = targets.get(0);
        RelicRecoveryVuMark vuMark = RelicRecoveryVuMark.from(target);

        if (vuMark != null && vuMark == RelicRecoveryVuMark.CENTER) {
            VuMarkIdentificationR2FOUR = 857;      //835      //   820
            CHANGE = -61; //-61
            //123
        } else if (vuMark != null && vuMark == RelicRecoveryVuMark.LEFT) {
            VuMarkIdentificationR2FOUR = 992;//987 //1085          //104   //1170  //
            CHANGE = -75;  //-75 //-90
        } else if (vuMark != null && vuMark == RelicRecoveryVuMark.RIGHT) {
            VuMarkIdentificationR2FOUR = 650;//83.8            //144//670   //630
            CHANGE = 120;//75//90

        } else {
            VuMarkIdentificationR2FOUR = 857;      //870
            CHANGE = -61;
        }
    }


    public void IdentifyB2F()     //special for BLUEteam
    {
        VuforiaTrackable target = targets.get(0);
        RelicRecoveryVuMark vuMark = RelicRecoveryVuMark.from(target);

        if (vuMark != null && vuMark == RelicRecoveryVuMark.CENTER) {
            VuMarkIdentificationB2 = 123;
        } else if (vuMark != null && vuMark == RelicRecoveryVuMark.LEFT) {
            VuMarkIdentificationB2 = 104;//47.0
        } else if (vuMark != null && vuMark == RelicRecoveryVuMark.RIGHT) {
            VuMarkIdentificationB2 = 144;//83.8
        } else {
            VuMarkIdentificationB2 = 106;
        }


    }

    public void IdentifyR2PONG()     //special for BLUEteam
    {

        VuforiaTrackable target = targets.get(0);
        RelicRecoveryVuMark vuMark = RelicRecoveryVuMark.from(target);

        if (vuMark != null && vuMark == RelicRecoveryVuMark.CENTER) {
            VuMarkIdentificationR2PONG = 370;      //835      //   820
            CHANGE = -61; //-61
            //123
        } else if (vuMark != null && vuMark == RelicRecoveryVuMark.LEFT) {
            VuMarkIdentificationR2PONG = 560;//987 //1085          //104   //1170  //
            CHANGE = -75;  //-75 //-90
        } else if (vuMark != null && vuMark == RelicRecoveryVuMark.RIGHT) {
            VuMarkIdentificationR2PONG = 160;//83.8            //144//670   //630
            CHANGE = 120;//75//90

        } else {
            VuMarkIdentificationR2PONG = 360;      //870
            CHANGE = -61;
        }
    }
    public void IdentifyR2SeO3()     //special for BLUEteam
    {

        VuforiaTrackable target = targets.get(0);
        RelicRecoveryVuMark vuMark = RelicRecoveryVuMark.from(target);

        if (vuMark != null && vuMark == RelicRecoveryVuMark.CENTER)
        {
            VuMarkIdentificationR2SeO3 = 370;      //835      //   820

            //123
        }
        else if (vuMark != null && vuMark == RelicRecoveryVuMark.LEFT)
        {
            VuMarkIdentificationR2SeO3 = 560;//987 //1085          //104   //1170  //

        }
        else if (vuMark != null && vuMark == RelicRecoveryVuMark.RIGHT)
        {
            VuMarkIdentificationR2SeO3 = 160;//83.8            //144//670   //630


        }
        else
        {
            VuMarkIdentificationR2PONG = 360;      //870

        }
    }
    public void IdentifyB2PONG()     //special for BLUEteam
    {

        VuforiaTrackable target = targets.get(0);
        RelicRecoveryVuMark vuMark = RelicRecoveryVuMark.from(target);

        if (vuMark != null && vuMark == RelicRecoveryVuMark.CENTER) {
            VuMarkIdentificationB2PONG = 310;      //835      //   820
            CHANGE = -90;

        } else if (vuMark != null && vuMark == RelicRecoveryVuMark.LEFT) {
            VuMarkIdentificationB2PONG = 160;//987 //1085          //104   //1170  //
            CHANGE = -120;  //-75 //-90
        } else if (vuMark != null && vuMark == RelicRecoveryVuMark.RIGHT) {
            VuMarkIdentificationB2PONG = 560;//83.8            //144//670   //630
            CHANGE = 100;//75//90

        } else {
            VuMarkIdentificationB2PONG = 310;      //870
            CHANGE = -90;
        }
    }
}
