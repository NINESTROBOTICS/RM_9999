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

import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

/**
 * This file contains an example of an iterative (Non-Linear) "OpMode".
 * An OpMode is a 'program' that runs in either the autonomous or the teleop period of an FTC match.
 * The names of OpModes appear on the menu of the FTC Driver Station.
 * When an selection is made from the menu, the corresponding OpMode
 * class is instantiated on the Robot Controller and executed.
 *
 * This particular OpMode just executes a basic Tank Drive Teleop for a two wheeled robot
 * It includes all the skeletal structure that all iterative OpModes contain.
 *
 * Use Android Studios to Copy this Class, and Paste it into your team's code folder with a new name.
 * Remove or comment out the @Disabled line to add this opmode to the Driver Station OpMode list
 */

@TeleOp(name="REV14241FF", group="Iterative Opmode")
//@Disabled
public class REV14241FF extends OpMode
{
    private TESTROBOT142421 robot =  new TESTROBOT142421() ;
    private final double Maxpower   = 0.8;
    private double power;
    private double strx;
    private double stry;
    private double tril;
    private double trir;
    private  double power2=0.2;
    boolean PRESENT=false;
    boolean PREVIOUS=false;
    boolean Switch=false;
    boolean PRESENTC=false;
    boolean PREVIOUSC=false;
    boolean SwitchC=false;


    boolean ServocollectPRESENT=false;
    boolean ServocollectPREVIOUS=false;
    boolean ServoSwitch=false;

///////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
    private boolean sao_left_CurrState  = false ;
    private boolean sao_left_PrevState  = false ;

    private boolean sao_right_CurrState  = false ;
    private boolean sao_right_PrevState  = false ;

    private int sao_switch = SAO_STOP ;

    private static final int SAO_STOP    = 0 ;
    private static final int SAO_FORWARD = 1 ;
    private static final int SAO_REVERSE = 2 ;


    private boolean YELL_left_CurrState  = false ;
    private boolean YELL_left_PrevState  = false ;

    private boolean YELL_right_CurrState  = false ;
    private boolean YELL_right_PrevState  = false ;

    private int YELL_switch = YELL_STOP ;

    private static final int YELL_STOP    = 0 ;
    private static final int YELL_FORWARD = 1 ;
    private static final int YELL_REVERSE = 2 ;
    //各种开关的布尔
    /*
     * Code to run ONCE when the driver hits INIT
     */
    @Override
    public void init()
    {
       robot.initializeRobot(this,"BLUE");
       //robot.servoCL.setPosition(0.1);
       robot.sensorColor.enableLed(false);
       robot.sensorColor.enableLed(false);
    }

    /*
     * Code to run REPEATEDLY after the driver hits INIT, but before they hit PLAY
     */
    @Override
    public void init_loop()
    {
    }

    /*
     * Code to run ONCE when the driver hits PLAY
     */
    @Override
    public void start()
    {
        robot.IsRunning = true;
        doTelemetry.start();
        SAO.start();
        EXCHANGE.start();
        collectandput.start();
        RELIC.start();
        SERVEC.start();
       //test.start();
    }

    /*
     * Code to run REPEATEDLY after the driver hits PLAY but before they hit STOP
     */
    @Override
    public void loop()
    {

    }

    /*
     * Code to run ONCE after the driver hits STOP
     */
    @Override
    public void stop()
    {
        robot.IsRunning = false;
    }


    Thread EXCHANGE =new Thread(new Runnable()
    {
        @Override
        public void run()
        {
            while(robot.IsRunning)
            {
                PRESENT=gamepad1.y;
                if (gamepad1.y && PRESENT!=PREVIOUS)
                {
                    Switch=!Switch;
                }
                else
                {
                    if (Switch)
                    {
                        Driver2();
                    }
                    else
                    {
                        Driver1();
                    }
                }
                PREVIOUS=PRESENT;                    //一键换位
            }
        }
    });

    private Thread doTelemetry =new Thread(new Runnable()
    {
        @Override
        public void run()
        {
            while (robot.IsRunning)
            {
                robot.addRobotTelemetry();
                //telemetry.addData("cm1",  robot.rangeSensor.getDistance(DistanceUnit.CM));
                //telemetry.addData("cm2",  robot.rangeSensor2.getDistance(DistanceUnit.CM));             //输出返回值
                telemetry.update();
            }
        }
    });
    private Thread SAO =new Thread(new Runnable()
    {
        @Override
        public void run()
        {
            while (robot.IsRunning)
            {
                SAO();                                 //扫方块
                Rise();                                //方块抬升
                STRANGEsao();
            }
        }
    });


    private Thread collectandput =new Thread(new Runnable()
    {
        @Override
        public void run()
        {
            while (robot.IsRunning)
            {
                servocollect();
                PUTRELIC();
            }
        }
    });
    private Thread RELIC =new Thread(new Runnable()
    {
        @Override
        public void run()
        {
            while (robot.IsRunning)
            {
                Relic();
                UPDOWN();
            }
        }
    });

    private Thread SERVEC =new Thread(new Runnable()
    {
        @Override
        public void run()
        {
            while (robot.IsRunning)
            {
                //SERVEC();
                STRANGEsao();
                PACKUP();
            }
        }
    });



    void Driver1()
    {
       /* if (gamepad1.dpad_up)
        {
            robot.setMotorPower(0.15,0.15);                   //直走 MAN
        }
        else if (gamepad1.dpad_down)
        {
            robot.setMotorPower(-0.15,-0.15);               //后退 MAN
        }
        else if (gamepad1.dpad_left)
        {
            robot. setMotorPower(-power2,power2);                //左转，慢 0.2
        }
        else if(gamepad1.dpad_right)
        {
            robot.setMotorPower(power2,-power2);                //右转，慢
        }
        */
        if(gamepad1.left_trigger >= 0.05)
        {    power=scaleInput(gamepad1.left_trigger)/1.5;
            robot.setMotorPower(-power,power,power,-power);  //向左平移
        }
        else if(gamepad1.right_trigger >= 0.05)
        {power=scaleInput(gamepad1.right_trigger)/1.5;
            robot.setMotorPower(power,-power,-power,power);   //向右平移
        }
        else if (gamepad1.dpad_left)
        {
            robot.setMotorPower(-power2,power2,power2,-power2);               //左平移，慢 0.2
        }
        else if(gamepad1.dpad_right)
        {
            robot.setMotorPower(power2,-power2,-power2,power2);                //右平移，慢
        }
       //else if(Math.abs(gamepad1.left_stick_x)>=0.1 || Math.abs(gamepad1.left_stick_y)>=0.1)
        //{
        //    strx=scaleInput(-gamepad1.left_stick_x)/3;
         //   stry=scaleInput(-gamepad1.left_stick_y)/3;
        //    robot.mecanumRun(0,-stry,-strx);
      //  }
        else
        {
            strx = scaleInput(-gamepad1.left_stick_y);      //tank操作
            stry = scaleInput(-gamepad1.right_stick_y);
            robot.setMotorPower ( strx,stry);
        }

    }
    void Driver2()
    {
        /*if (gamepad1.dpad_up)
        {
            robot.setMotorPower(-0.15,-0.15);                   //直走 MAN
        }
        else if (gamepad1.dpad_down)
        {
            robot.setMotorPower(0.15,0.15);               //后退 MAN
        }
        else if (gamepad1.dpad_left)
        {
            robot. setMotorPower(-power2,power2);                //左转，慢 0.2
        }
        else if(gamepad1.dpad_right)
        {
            robot.setMotorPower(power2,-power2);                //右转，慢
        }
        */
        if(gamepad1.right_trigger >= 0.05)
        {    power=scaleInput(gamepad1.right_trigger)/1.8;
            robot.setMotorPower(-power,power,power,-power);  //向you平移
        }
        else if(gamepad1.left_trigger >= 0.05)
        {power=scaleInput(gamepad1.left_trigger)/1.8;
            robot.setMotorPower(power,-power,-power,power);   //向zuo平移
        }
        else if (gamepad1.dpad_right)
        {
            robot.setMotorPower(-power2,power2,power2,-power2);               //you平移，慢 0.2
        }
        else if(gamepad1.dpad_left)
        {
            robot.setMotorPower(power2,-power2,-power2,power2);                //zuo平移，慢
        }
        else if(Math.abs(gamepad1.left_stick_x)>=0.1 || Math.abs(gamepad1.left_stick_y)>=0.1)
        {
            strx=scaleInput(gamepad1.left_stick_x)/3;
            stry=scaleInput(gamepad1.left_stick_y)/3;
            robot.mecanumRun(0,stry,strx);
        }
        else
        {
            strx=scaleInput(gamepad1.right_stick_x)/1.5;
            stry=scaleInput(gamepad1.right_stick_y);
            robot.mecanumRun(0,stry,strx);

        }


    }

    void SAO()//扫方块
    {
        sao_left_CurrState = gamepad2.left_bumper;

        if ((sao_left_CurrState == true) && (sao_left_CurrState != sao_left_PrevState)) {
            if (sao_switch != SAO_FORWARD) sao_switch = SAO_FORWARD;
            else sao_switch = SAO_STOP;
        }
        else {
            sao_right_CurrState = gamepad2.right_bumper;

            if ((sao_right_CurrState == true) && (sao_right_CurrState != sao_right_PrevState)) {
                if (sao_switch != SAO_REVERSE) sao_switch = SAO_REVERSE;
                else sao_switch = SAO_STOP;
            }
        }

        sao_left_PrevState = sao_left_CurrState;
        sao_right_PrevState = sao_right_CurrState;


        switch (sao_switch) {
            case SAO_FORWARD://collect
                robot.motorSL.setPower( 0.78);
                robot.motorSR.setPower(-0.78);
                break;

            case SAO_REVERSE:
                robot.motorSL.setPower(-0.5);
                robot.motorSR.setPower( 0.5);
                break;

            case SAO_STOP:
                robot.motorSL.setPower(0);
                robot.motorSR.setPower(0);
                break;
        }
    }
    private void Rise()//升降装置
    {

        if(gamepad2.dpad_up)
        {
            robot.motorRISE.setPower(-0.5);
            robot.GlyphServoPin();
        }
        else if(gamepad2.dpad_down)
        {
            robot.motorRISE.setPower(0.2);
            robot.GlyphServoDown();
        }

        else
        {
            robot.motorRISE.setPower(-0.007);
        }

    }
    void Relic()//大黄人滑轨
    {

        if(gamepad2.right_trigger>=0.05)
        {
            trir= scaleInput(gamepad2.right_trigger);
            robot.motorRelic.setPower(trir);///2.5
        }
        else if(gamepad2.left_trigger>=0.05)
        {
            tril= scaleInput(gamepad2.left_trigger);
            robot.motorRelic.setPower(-tril);///2.5
        }
        else
        {
            robot.motorRelic.setPower(0.005);
        }

    }
   /* void SERVEC()
    {
        PRESENTC=gamepad2.x;
        if (gamepad2.x && PRESENTC!=PREVIOUSC)
        {
            SwitchC=!SwitchC;
        }
        else
        {
            if (SwitchC)
            {
                //robot.servoCR.setPosition(0.1);
                robot.servoCL.setPosition(0);

            }
            else
            {
               // robot.servoCR.setPosition(0.55);
                robot.servoCL.setPosition(1);
            }
        }
        PREVIOUSC=PRESENTC;
    }
   */
   void STRANGEsao()
   {
       if(gamepad1.x)
       {

           robot.servoD.setPosition(1);
       }

   }
    void UPDOWN()//举起放下大黄人
    {

        if(gamepad2.y)
        {
            robot.servoA1.setPosition(0.3);
        }
        else if(gamepad2.x)
        {

            robot.servoA1.setPosition(1);
        }


    }
    void PUTRELIC()
    {

        if(gamepad2.a)
        {
            robot.servoA2.setPosition(0.85);
        }
        else if(gamepad2.b)
        {
            robot.servoA2.setPosition(0.20);
        }
    }

    void PACKUP()
    {
        if(gamepad2.left_stick_y>0.6)
        {
            robot.servoW.setPosition(robot.Wposition);
            robot.servoD.setPosition(1);
            robot.servoJ.setPosition(0.935);
        }

    }
    void servocollect()
    {
        /* ServocollectPRESENT=gamepad2.a;
        if (gamepad2.a && ServocollectPRESENT!=ServocollectPREVIOUS)
        {
            ServoSwitch=!ServoSwitch;
        }
        else
        {
            if (ServoSwitch)
            {
                robot.servoS1.setPosition(0.10);
                robot.servoS2.setPosition(0.90);
            }
            else
            {
                robot.servoS1.setPosition(0.65);
                robot.servoS2.setPosition(0.35);
            }
        }

        ServocollectPREVIOUS=ServocollectPRESENT;*/
       if (gamepad1.b)//forward
        {
            robot.GlyphServo();
        }
        else if(gamepad1.a)//back
        {
            robot.GlyphServoDown();
        }


    }


    Thread tele =new Thread(new Runnable()
    {
        @Override
        public void run()
        {
            while (robot.IsRunning)
            {

            }
        }
    });

    double scaleInput( double dVal )
    {
        double[] scaleArray = { 0.0, 0.05, 0.09, 0.10, 0.12, 0.15, 0.18, 0.24, 0.30, 0.36, 0.43, 0.50, 0.60, 0.72, 0.85, 1.00, 1.00 };

        int index = (int) (dVal * 16.0);

        if (index < 0)
        {
            index = -index;
        }

        if (index > 16)
        {
            index = 16;
        }

        double dScale;

        if (dVal < 0)
        {
            dScale = -scaleArray[index];
        }
        else
        {
            dScale = scaleArray[index];
        }
        return dScale;
    }



}
