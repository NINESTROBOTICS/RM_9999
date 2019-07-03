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
import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.util.ElapsedTime;
import com.qualcomm.robotcore.util.Range;



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

@Autonomous(name="B1", group="Linear Opmode")
//@Disabled
public class B1 extends LinearOpMode
{

    private TESTROBOT robot =  new TESTROBOT() ;
    private  TESTVUFORIA IDENTIFY = new TESTVUFORIA() ;
    private ElapsedTime runtime = new ElapsedTime();

    @Override
    public void runOpMode()
    {

        robot.initializeRobot(this,"BLUE");
        IDENTIFY.initializeVuforia(this,robot);



        waitForStart();

        Running.start();

        IDENTIFY.activeTracking();

        runtime.reset();
        while (runtime.milliseconds()<=500)            //imageIDENTIFY
        {
            IDENTIFY.IdentifyB();
        }
        /////////////////////////////////////JEWELRY////////////////////////////////////////////
       //jewelry

        robot.servoLift();
        sleep(700);//lift

        robot.stopAllMotors();
        sleep(200);

        robot.Jewelry();//jewelry
        sleep(200);

        robot.servoDown();
        sleep(700);//down

        robot.servoW.setPosition(robot.Wposition);
        sleep(200);
        //robot.rotateToTargetDirection(0);

       /////////////////////////////////////Glyph1/////////////////////////////////////////

        robot.mecanumRun(0,-0.4,0);//Run to box
        sleep(1400);

        robot.rotateToTargetDirection(0);//Calibration (Turn to the front)


        //robot.standOffWallR(IDENTIFY.VuMarkIdentificationB);//Adjust the diatance

        robot.runHalfSpeed(-IDENTIFY.VuMarkIdentificationB,0);

        robot.rotateToTargetDirection(0);//Calibration

        robot.servoD.setPosition(0.1);

        robot.stopAllMotors();
        sleep(150);

        robot.SOCL2();

        robot.rotateToTargetDirection(0);

        robot.stopAllMotors();
        sleep(200);

        robot.Navigation();

        robot.servoD.setPosition(1);

        robot.stopAllMotors();
        sleep(200);

        robot.GlyphServo();

          //400
        robot.stopAllMotors();
        sleep(200);

        robot.runDistance(0,-50,0.5);

        /////////////////////////////////////Glyph2/////////////////////////////////////////////////

        robot.runHalfSpeed(0,200);//150

        //robot.standOffWallB2(84);//RANGE2

        robot.runHalfSpeed(IDENTIFY.VuMarkIdentificationB-590,0);//500

        robot.GlyphServoDown();

        //robot.rotateToTargetDirection(25);

        robot.CollectGlyph(0.9,0.75);//collect the second glyph

        robot.runMaxSpeed(0,550);

        robot.runHalfSpeed(0,400);

        //robot.CollectGlyph(0,0);

        //robot.rotateToTargetDirection(25);

        robot.runMaxSpeed(0,-910);

        robot.rotateToTargetDirection(0);

        robot.runHalfSpeed(150,0);

        //robot.runMaxSpeed(200,0);
        ////////////////////////////////////////////////////////////////////////////////////////////
        //robot.standOffWallR(IDENTIFY.VuMarkIdentificationB);

        robot.GlyphServoPin();

        robot.servoD.setPosition(0.1);

        robot.stopAllMotors();
        sleep(150);

        robot.SOCL2();

        robot.rotateToTargetDirection(0);

        robot.Navigation();

        robot.servoD.setPosition(1);

        robot.rotateToTargetDirection(0);

        robot.motorRISE.setPower(0.7);
        sleep(500);

        robot.motorRISE.setPower(0);

        robot.GlyphServo();

        robot.stopAllMotors();
        sleep(200);

        robot.runDistance(0,-50,0.4);

        robot.stopAllMotors();
        sleep(200);  //400

        robot.runHalfSpeed(0,50);

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
