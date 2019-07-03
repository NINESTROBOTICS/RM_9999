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
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit;


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

@Autonomous(name="R1", group="Linear Opmode")
//@Disabled
public class R1 extends LinearOpMode
{

    private TESTROBOT robot =  new TESTROBOT() ;
    private  TESTVUFORIA IDENTIFY = new TESTVUFORIA() ;
    private ElapsedTime runtime = new ElapsedTime();
    double distance1=0;
    double DELTAdistance=0;

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
        runtime.reset();
        while (runtime.milliseconds()<=500)            //imageIDENTIFY
        {
            IDENTIFY.IdentifyR();
        }

        /////////////////////////////////////JEWELRY////////////////////////////////////////////
        //jewelry
        robot.servoLift();
        sleep(700);//lift

        robot.Jewelry();//jewelry

        //sleep(500);

        robot.stopAllMotors();
        sleep(200);

        robot.servoDown();
        sleep(700);//down

        robot.servoW.setPosition(robot.Wposition);
        //robot.rotateToTargetDirection(0);
        /////////////////////////////////////Glyph1/////////////////////////////////////////
        robot.mecanumRun(0,0.4,0);
        sleep(1500);//走下平衡板

        robot.rotateToTargetDirection(180);

        //robot.standOffWallB2(IDENTIFY.VuMarkIdentificationF);

        robot.runHalfSpeed(IDENTIFY.VuMarkIdentificationF,0);

        robot.rotateToTargetDirection(180);

        robot.servoD.setPosition(0.1);

        robot.stopAllMotors();
        sleep(150);

        robot.SOCL2();

        //robot.rotateToTargetDirection(180);

        robot.Navigation();

        robot.servoD.setPosition(1);

        robot.stopAllMotors();
        sleep(200);  //400

        robot.GlyphServo();

        robot.stopAllMotors();
        sleep(200);

        robot.stopAllMotors();
        sleep(200);  //400

        robot.runDistance(0,-50,0.5);


        ////////////////////////////////////////////////////////////////////////////////////////////
        robot.runHalfSpeed(0,200);

        robot.runHalfSpeed(670-IDENTIFY.VuMarkIdentificationF,0);

        robot.GlyphServoDown();

        robot.rotateToTargetDirection(180);


        robot.CollectGlyph(0.9,0.75);//collect the second glyph

        robot.runMaxSpeed(0,430);

        robot.runHalfSpeed(0,450);   //400

        robot.rotateToTargetDirection(180);

        robot.runMaxSpeed(0,-900);   //925

        robot.rotateToTargetDirection(180);

        robot.runHalfSpeed(-220,0);

        /*
        robot.CollectGlyph(0.9,-0.75);//collect glyph
        //robot.standOffWall3(24);

        robot.stopAllMotors();
        sleep(150);

        robot.CollectGlyph(-0.9,0.75);//collect glyph
        //robot.standOffWall3(24);

        robot.stopAllMotors();
        sleep(50);

        ////////////////////////////////////////////////////////////////////////////////////////////

        //robot.GlyphServoPin();

        robot.servoD.setPosition(0.1);

        robot.stopAllMotors();
        sleep(150);

        robot.SOCL2();

        //robot.rotateToTargetDirection(180);

        robot.Navigation();

        robot.servoD.setPosition(1);

       // robot.rotateToTargetDirection(180);

        //robot.motorRISE.setPower(0.7);
        //sleep(500);

        //robot.motorRISE.setPower(0);

        robot.GlyphServo();

        robot.stopAllMotors();
        sleep(300);  //400

       // robot.runHalfSpeed(0,50);
        robot.mecanumRun(0,0.2,0);
        sleep(200);
       // robot.GlyphServoDown();
       */
        robot.stopAllMotors();
        ////////////////////////////////////////////////////////////////////////////////////////////

        robot.IsRunning=false;


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
