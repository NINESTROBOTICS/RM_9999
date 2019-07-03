/* Copyright (c) 2017 FIRST. All rights reserved.
 *
 * Redistribution and use in source and binary forms, with or without modification,
 * are permitted (subject to the limitations in the disclaimer below) provided that
 * the following conditions are met:
 *
 * Redistributions of source code must retain the above copyright notice, this list
 * of conditions and the following disclaimer.
 *
 * Redistributions in binary form must reproduce the above copyright notice, this
 * list of conditions and the following disclaimer in the documentation and/or
 * other materials provided with the distribution.
 *
 * Neither the name of FIRST nor the names of its contributors may be used to endorse or
 * promote products derived from this software without specific prior written permission.
 *
 * NO EXPRESS OR IMPLIED LICENSES TO ANY PARTY'S PATENT RIGHTS ARE GRANTED BY THIS
 * LICENSE. THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS
 * "AS IS" AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO,
 * THE IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE
 * ARE DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT OWNER OR CONTRIBUTORS BE LIABLE
 * FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL
 * DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR
 * SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER
 * CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY,
 * OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE
 * OF THIS SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.
 */

package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.util.ElapsedTime;


/**
 * This file contains an minimal example of a Linear "OpMode". An OpMode is a 'program' that runs in either
 * the autonomous or the teleop period of an FTC match. The names of OpModes appear on the menu
 * of the FTC Driver Station. When an selection is made from the menu, the corresponding OpMode
 * class is instantiated on the Robot Controller and executed.
 *
 * This particular OpMode just executes a basic Tank Drive Teleop for a two wheeled robot
 * It includes all the skeletal structure that all linear OpModes contain.
 *
 * Use Android Studios to Copy this Class, and Paste it into your team's code folder with a new name.
 * Remove or comment out the @Disabled line to add this opmode to the Driver Station OpMode list
 */

@Autonomous(name="R2PONG", group="Linear Opmode")
//@Disabledp
public class R2PONG extends LinearOpMode
{

    private TESTROBOT robot =  new TESTROBOT() ;
    private  TESTVUFORIA IDENTIFY = new TESTVUFORIA() ;
    private ElapsedTime runtime = new ElapsedTime();

    private static  double    Rundis   = 195;//720    //400
    private static  double    Runback  = 427;//530//700   //830  //490

    @Override
    public void runOpMode()
    {
        robot.initializeRobot(this,"RED");
        IDENTIFY.initializeVuforia(this,robot);

        while(!isStarted())
        {
            telemetry.addData(">", "Press Play to start");
            telemetry.update();
        }

        waitForStart();

        Running.start();

        IDENTIFY.activeTracking();
        // IDENTIFY.addNavgation();
        runtime.reset();
        while (runtime.milliseconds()<=500)            //imageIDENTIFY
        {
            IDENTIFY.IdentifyR2PONG();
        }
        /////////////////////////////////////JEWELRY////////////////////////////////////////////
        //jewelry

        robot.servoLift();
        sleep(500);//lift

        robot.stopAllMotors();
        sleep(200);

        robot.Jewelry();//jewelry
        sleep(200);

        robot.servoDown();
        sleep(500);//down

        robot.servoW.setPosition(robot.Wposition);
        sleep(200);

        /////////////////////////////////////Glyph1/////////////////////////////////////////

        robot.stopAllMotors();
        sleep(100);

        robot.runMaxSpeed(0,450);

        telemetry.addData("success","//");
        robot.rotateToTargetDirection(90);

        robot.mecanumRun(-0.48,0,0);
        sleep(500);

        robot.runHalfSpeed(IDENTIFY.VuMarkIdentificationR2PONG,0 );

        robot.rotateToTargetDirection(90);


        robot.runMaxSpeed(0,Rundis);

        robot.CollectGlyph(0.9,0.75);//collect glyph

        robot.runHalfSpeed(0,220);

        robot.rotateToTargetDirection(90);

        robot.CollectGlyph(0,0);

        robot.runMaxSpeed(-20,-Runback+40);

        robot.runHalfSpeed(0,-150);

        robot.rotateToTargetDirection(90);

        robot.servoD.setPosition(0.07);

        robot.CollectGlyph(0.9,-0.75);//collect glyph
        //robot.standOffWall3(24);

        robot.stopAllMotors();
        sleep(130);

        robot.CollectGlyph(-0.9,-0.75);//collect glyph
        //robot.standOffWall3(24);

        robot.stopAllMotors();
        sleep(30);

        robot.CollectGlyph(0.9,0.75);

        robot.stopAllMotors();
        sleep(200);

        robot.SOCL2();
        // robot.runHalfSpeed(0,50);

        //robot.rotateToTargetDirection(90);

       /* robot.servoD.setPosition(1);
        robot.stopAllMotors();
        sleep(500);
       */

        robot.stopAllMotors();
        sleep(200);

        robot.Navigation();

        //robot.runHalfSpeed(0,20);


        robot.rotateToTargetDirection(90);

        robot.servoD.setPosition(1);

        robot.GlyphServo();
        sleep(1000);


        robot.mecanumRun(0,-0.2,0);
        sleep(300);  //400

        /////////////////////////////////////Glyph1/////////////////////////////////////////
        robot.rotateToTargetDirection(90);

        robot.GlyphServoDown();

        robot.runMaxSpeed(50,Rundis+65);//130

        robot.stopAllMotors();
        sleep(100);

        robot.runHalfSpeed(IDENTIFY.CHANGE,0);

        robot.stopAllMotors();
        sleep(100);

        robot.CollectGlyph(0.9,0.75);//collect glyph

        robot.runMaxSpeed(0,450);

        robot.rotateToTargetDirection(90);

        robot.runMaxSpeed(-50,-Runback-120);//-170//-50

        robot.runHalfSpeed(0,-117);

        robot.rotateToTargetDirection(90);

        robot.servoD.setPosition(0.07);

        //robot.CollectGlyph(-0.3,-0.3);

        robot.CollectGlyph(0.9,-0.75);//collect glyph
        //robot.standOffWall3(24);

        robot.stopAllMotors();
        sleep(130);

        robot.CollectGlyph(-0.9,0.75);//collect glyph
        //robot.standOffWall3(24);

        robot.stopAllMotors();
        sleep(30);

        robot.CollectGlyph(0.9,0.75);

        robot.stopAllMotors();
        sleep(200);

        robot.SOCL2();

        // robot.runHalfSpeed(0,22);

        robot.stopAllMotors();
        sleep(200);

        robot.CollectGlyph(0,0);
        // robot.runHalfSpeed(0,50);

        // robot.runHalfSpeed(0,40);

        robot.GlyphServoPin();

        robot.Navigation();

        robot.rotateToTargetDirection(90);

        robot.servoD.setPosition(1);

        robot.GlyphServo();
        sleep(1000);

        robot.mecanumRun(0,0.2,0);
        sleep(300);

        //robot.mecanumRun(0,-0.3,0);
        //sleep(300);

        // robot.stopAllMotors();
        //sleep(200);

        robot.runHalfSpeed(0,15);

        robot.stopAllMotors();

        robot.GlyphServoDown();

        robot.stopAllMotors();

    }

    private Thread Running =new Thread(new Runnable()
    {
        @Override
        public void run()
        {
            while (opModeIsActive())//robot.IsRunning==true&&
            {
                robot.IsRunning=true;
            }
            robot.IsRunning=false;
        }
    });
}
