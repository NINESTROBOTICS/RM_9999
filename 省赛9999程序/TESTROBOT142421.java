package org.firstinspires.ftc.teamcode;

import com.qualcomm.hardware.bosch.BNO055IMU;
import com.qualcomm.hardware.bosch.JustLoggingAccelerationIntegrator;
import com.qualcomm.hardware.modernrobotics.ModernRoboticsI2cRangeSensor;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.hardware.ColorSensor;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DistanceSensor;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.util.Range;

import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.robotcore.external.navigation.AxesOrder;
import org.firstinspires.ftc.robotcore.external.navigation.AxesReference;
import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit;
import org.firstinspires.ftc.robotcore.external.navigation.Orientation;


public class TESTROBOT142421 {
    private OpMode myOpMode;

    /**
     * Dc Motors
     */
    private DcMotor motor[] = new DcMotor[MOTOR_NUMBER];

    public DcMotor motorSL = null;
    public DcMotor motorSR = null;
    public DcMotor motorRISE = null;
    public DcMotor motorRelic = null;
    public ModernRoboticsI2cRangeSensor rangeSensor;
    public ModernRoboticsI2cRangeSensor rangeSensor2;
    public BNO055IMU imu;
    Orientation angles;


    //public ModernRoboticsI2cGyro gyro = null;
    public DistanceSensor sensorDistance = null;
    //public DistanceSensor sensorDistance3= null;
    //public ModernRoboticsI2cRangeSensor rangeSensor3 = null;
    //public DigitalChannel digitalTouch;


    public Servo servoJ = null;//JEWELRY
    public Servo servoS1 = null;
    public Servo servoS2 = null;
    //public Servo servoCR = null;
    public Servo servoCL = null;
    public Servo servoW = null;
    //public CRServo servoA1 = null;
    public Servo servoA2 = null;
    public Servo servoA1 = null;
    public Servo servoD = null;
    public Servo servoIn =null;
    public ColorSensor sensorColor;
    boolean IsRunning;
    int OriginalDirection;

    private boolean teamColor;
    private static final double MAX_MOTOR_POWER = 1.0;
    double DMINPOWER=0.11;//0.15

    private static final boolean RED_COLOR = true;
    private static final boolean BLUE_COLOR = false;
    private static final double ENCODER_PROPORTION = 0.0018;
    private static final double WHEEL_DIAMETER = 400 / 3; //in mm
    private static final double GEAR_RATIO = 1.5;
    private static final double COUNTS_PER_MOTOR_REV_ANDYMARK_20 = 560;// AndyMark 1:20 Motor Encoder
    private static final double COUNTS_PER_MOTOR_REV_ANDYMARK_40 = 1120;// AndyMark 1:40 Motor Encoder
    private static final double COUNTS_PER_MOTOR_REV_ANDYMARK_60 = 1680;// AndyMark 1:60 Motor Encoder
    private static final double COUNTS_PER_MOTOR_REV = COUNTS_PER_MOTOR_REV_ANDYMARK_40;

    private static final byte R1 = 0;
    private static final byte L1 = 1;
    private static final byte R2 = 2;
    private static final byte L2 = 3;
    private static final byte MOTOR_NUMBER = 4;
    private final double Maxpower = 0.8;
    double Wposition = 0.332;
    double distance=9.1;

    public static final double TURNING_POWER = 0.006;
    private static final double MIN_TURNING_POWER = 0.05;//0.04
    private static final double MAX_TURNING_POWER = 0.55;

    private static final int EPS_ANGLE = 0;
    private static final int ROUND = 360;

    private static final double NaMINPOWER = 0.030;//0.035


    private double picturetarget = 0;
    private static final double Rightpicture = 75;
    private static final double Centerpicture = 50;
    private static final double Leftpicture = 25;

    private double rangetarget = 0;
    private static final double Rightrange = 75;
    private static final double Centerrange = 50;
    private static final double Leftrange = 25;

    public double LeftPower;
    public double RightPower;

    int turnDirection = 7;
    double errorsum;
    double rangesumB;
    double rangesumR;
    double Kp = 0.008; /*increase the Kp to enhance the oscillation and reduce the adjusting time *///0.008,0.0024,0.005
    double Ki = 90000; /*increase the Ki to decrease the oscillation and a longer adjusting time */
    double Kp1 = 0.003;
    double Ki1 = 150000;

    public TESTROBOT142421() {/* Constructor */}


    private int prevEncoderValue[] = new int[MOTOR_NUMBER];

    private int currEncoderValue[] = new int[MOTOR_NUMBER];

    private int targetPosition[] = new int[MOTOR_NUMBER];

    private double power[] = new double[MOTOR_NUMBER];

    private double targetPower[] = new double[MOTOR_NUMBER];

    private double correctedPower[] = new double[MOTOR_NUMBER];

    private double deltaValue[] = new double[MOTOR_NUMBER];


    public void initializeRobot(OpMode OP_MODE, String TEAM_COLOR) {
        for (int i = 0; i < MOTOR_NUMBER; i++)
            power[i] = 0.0;

        for (int i = 0; i < MOTOR_NUMBER; i++)
            targetPower[i] = 0.0;

        for (int i = 0; i < MOTOR_NUMBER; i++)
            correctedPower[i] = 0.0;

        for (int i = 0; i < MOTOR_NUMBER; i++)
            deltaValue[i] = 0.0;

        for (int i = 0; i < MOTOR_NUMBER; i++)
            targetPosition[i] = 0;


        // Save reference to Hardware map
        myOpMode = OP_MODE;
        motor[L1] = myOpMode.hardwareMap.dcMotor.get("L1");
        motor[L2] = myOpMode.hardwareMap.dcMotor.get("L2");
        motor[R1] = myOpMode.hardwareMap.dcMotor.get("R1");
        motor[R2] = myOpMode.hardwareMap.dcMotor.get("R2");


        motor[R1].setDirection(DcMotor.Direction.REVERSE);
        motor[R2].setDirection(DcMotor.Direction.REVERSE);
        motor[L1].setDirection(DcMotor.Direction.FORWARD);
        motor[L2].setDirection(DcMotor.Direction.FORWARD);

        motorSL = myOpMode.hardwareMap.dcMotor.get("motorSL");
        motorSR = myOpMode.hardwareMap.dcMotor.get("motorSR");
        motorRelic = myOpMode.hardwareMap.dcMotor.get("motorRelic");
        motorRISE = myOpMode.hardwareMap.dcMotor.get("motorRISE");

        motorSL.setDirection(DcMotor.Direction.REVERSE);
        motorSR.setDirection(DcMotor.Direction.REVERSE);

        setDcMotorMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        setDcMotorMode(DcMotor.RunMode.RUN_USING_ENCODER);
       // motorRISE.setMode(DcMotor.RunMode.RUN_TO_POSITION);

        servoJ = myOpMode.hardwareMap.servo.get("servoJ");

        servoS1 = myOpMode.hardwareMap.servo.get("servoS1");
        servoS2 = myOpMode.hardwareMap.servo.get("servoS2");

        servoS1.setDirection(Servo.Direction.REVERSE);
        servoS2.setDirection(Servo.Direction.REVERSE);

        servoW = myOpMode.hardwareMap.servo.get("servoW");
        //servoCL = myOpMode.hardwareMap.servo.get("servoCL");

        servoA1 = myOpMode.hardwareMap.servo.get("servoA1");
        servoA2 = myOpMode.hardwareMap.servo.get("servoA2");

        //servoA3 = myOpMode.hardwareMap.crservo.get("servoA3");
        servoD = myOpMode.hardwareMap.servo.get("servoD");

        servoIn = myOpMode.hardwareMap.servo.get("servoIn");

        sensorColor = myOpMode.hardwareMap.colorSensor.get("color");

        //gyro = myOpMode.hardwareMap.get(ModernRoboticsI2cGyro.class, "gyro");
        //gyro.calibrate();

        rangeSensor = myOpMode.hardwareMap.get(ModernRoboticsI2cRangeSensor.class, "range");
        rangeSensor2 = myOpMode.hardwareMap.get(ModernRoboticsI2cRangeSensor.class, "range2");
        //rangeSensor2 = myOpMode.hardwareMap.get(ModernRoboticsI2cRangeSensor.class, "sensor_range2");
        //rangeSensor3 = myOpMode.hardwareMap.get(ModernRoboticsI2cRangeSensor.class, "sensor_range3");


        sensorDistance = myOpMode.hardwareMap.get(DistanceSensor.class, "distance");
        //sensorDistance3 = myOpMode.hardwareMap.get(DistanceSensor.class,"sensor_distance3");
        //digitalTouch = myOpMode.hardwareMap.get(DigitalChannel.class, "sensor_digital");

        // set the digital channel to input.
        //digitalTouch.setMode(DigitalChannel.Mode.INPUT);



        teamColor = TEAM_COLOR.equals("RED") ? RED_COLOR : BLUE_COLOR;

        IsRunning = true;

        servoJ.setPosition(0.97);

        servoS1.setPosition(0.427);
        servoS2.setPosition(0.589);

        //servoCR.setPosition(0
        // .1);
        servoW.setPosition(Wposition);
        servoD.setPosition(1);
        servoIn.setPosition(0);
        BNO055IMU.Parameters parameters = new BNO055IMU.Parameters();
        parameters.angleUnit           = BNO055IMU.AngleUnit.DEGREES;
        parameters.accelUnit           = BNO055IMU.AccelUnit.METERS_PERSEC_PERSEC;
        parameters.calibrationDataFile = "BNO055IMUCalibration.json"; // see the calibration sample opmode
        parameters.loggingEnabled      = true;
        parameters.loggingTag          = "IMU";
        parameters.accelerationIntegrationAlgorithm = new JustLoggingAccelerationIntegrator();

        // Retrieve and initialize the IMU. We expect the IMU to be attached to an I2C port
        // on a Core Device Interface Module, configured to be a sensor of type "AdaFruit IMU",
        // and named "imu".
        imu =myOpMode. hardwareMap.get(BNO055IMU.class, "imu");
        imu.initialize(parameters);

    }

    public void setDcMotorMode(DcMotor.RunMode mode) {
        for (int i = 0; i < MOTOR_NUMBER; i++)
            motor[i].setMode(mode);

        //motorLF.setMode(mode);
        //motorLB.setMode(mode);
        //motorRF.setMode(mode);
        //motorRB.setMode(mode);
        //motorCenter.setMode(mode);
    }

    public void addRobotTelemetry()
    {
        myOpMode.telemetry.addData(">", "Position");
        myOpMode.telemetry.addData("LF Position", motor[L1].getCurrentPosition());
        myOpMode.telemetry.addData("LB Position", motor[L2].getCurrentPosition());
        myOpMode.telemetry.addData("RF Position", motor[R1].getCurrentPosition());
        myOpMode.telemetry.addData("RB Position", motor[R2].getCurrentPosition());
        //myOpMode.telemetry.addData("mm", "%.2f mm", rangeSensor.getDistance(DistanceUnit.MM));
        //myOpMode.telemetry.addData("mm", "%.2f mm", rangeSensor2.getDistance(DistanceUnit.MM));


        myOpMode.telemetry.addData("cm", "%.2f cm", rangeSensor.getDistance(DistanceUnit.CM));

        myOpMode.telemetry.addData("dis",sensorDistance.getDistance(DistanceUnit.CM));

        myOpMode.telemetry.addData("cm2", "%.2f cm", rangeSensor2.getDistance(DistanceUnit.CM));

        //myOpMode.telemetry.addData("angle",gyro.getIntegratedZValue());
        myOpMode.telemetry.addData("oritation",getangle());
        // myOpMode.telemetry.addData("x",sensorDistance3.getDistance(DistanceUnit.CM));//myOpMode.telemetry.addData("Direction",gyro.getIntegratedZValue());
        /// myOpMode.telemetry.addData("cm1",  rangeSensor.getDistance(DistanceUnit.CM));
        //myOpMode.telemetry.addData("cm2",  rangeSensor2.getDistance(DistanceUnit.CM));
        //myOpMode.telemetry.addData("Distance (cm)",
        // String.format(Locale.US, "%.02f", sensorDistance.getDistance(DistanceUnit.CM)));
        myOpMode.telemetry.update();
    }

    /* -----------------_(:3∠])_ -------------------------------------_(:3∠])_------------------*/
    public void setMotorPower(double LFPower, double RFPower, double LBPower, double RBPower /*,double CPower*/) {
        double maxPower = Math.max(Math.max(Math.abs(LFPower), Math.abs(LBPower)), Math.max(Math.abs(RFPower), Math.abs(RBPower)));

        if (maxPower > MAX_MOTOR_POWER) {
            LFPower = LFPower / maxPower * MAX_MOTOR_POWER;
            LBPower = LBPower / maxPower * MAX_MOTOR_POWER;
            RFPower = RFPower / maxPower * MAX_MOTOR_POWER;
            RBPower = RBPower / maxPower * MAX_MOTOR_POWER;
        }

        power[L1] = Range.clip(LFPower, -1, 1);
        power[L2] = Range.clip(LBPower, -1, 1);
        power[R1] = Range.clip(RFPower, -1, 1);
        power[R2] = Range.clip(RBPower, -1, 1);

        for (int i = 0; i < MOTOR_NUMBER; i++)
            motor[i].setPower(power[i]);

        //motorLF.setPower(power[LF]) ;
        //motorLB.setPower(power[LB]) ;
        //motorRF.setPower(power[RF]) ;
        //motorRB.setPower(power[RB]) ;
    }

    void setMotorPower(double L_POWER1, double R_POWER1) {
        setMotorPower(L_POWER1, R_POWER1, L_POWER1, R_POWER1);

        //motorL1.setPower(L_POWER1);
        //motorL2.setPower(L_POWER1);

        //motorR1.setPower(R_POWER1);
        //motorR2.setPower(R_POWER1);
    }

    public void adjustWithEncoder() {
        int num = 0;

        double averageValue = 0;

        updateEncoderValue();

        for (int i = 0; i < MOTOR_NUMBER; i++) {
            if (targetPower[i] == 0) continue;

            deltaValue[i] = (currEncoderValue[i] - prevEncoderValue[i]) / targetPower[i] / 10;

            averageValue += deltaValue[i];

            num++;
        }

        averageValue /= num;

        for (int i = 0; i < MOTOR_NUMBER; i++) {
            if (targetPower[i] == 0) {
                correctedPower[i] = 0.0;
                continue;
            }

            correctedPower[i] = (averageValue - deltaValue[i]) * ENCODER_PROPORTION * (targetPower[i] < 0 ? -1 : 1);
        }

        for (int i = 0; i < MOTOR_NUMBER; i++)
            power[i] = targetPower[i] + correctedPower[i];

        setMotorPower(power[L1] / 1.23, power[R1] / 1.23, power[L2] / 1.23, power[R2] / 1.23);//2.1
    }

    public void adjustWithEncoderHALF() {
        int num = 0;

        double averageValue = 0;

        updateEncoderValue();

        for (int i = 0; i < MOTOR_NUMBER; i++) {
            if (targetPower[i] == 0) continue;

            deltaValue[i] = (currEncoderValue[i] - prevEncoderValue[i]) / targetPower[i] / 10;

            averageValue += deltaValue[i];

            num++;
        }

        averageValue /= num;

        for (int i = 0; i < MOTOR_NUMBER; i++) {
            if (targetPower[i] == 0) {
                correctedPower[i] = 0.0;
                continue;
            }

            correctedPower[i] = (averageValue - deltaValue[i]) * ENCODER_PROPORTION * (targetPower[i] < 0 ? -1 : 1);
        }

        for (int i = 0; i < MOTOR_NUMBER; i++)
            power[i] = targetPower[i] + correctedPower[i];

        setMotorPower(power[L1] / 2.9, power[R1] / 2.9, power[L2] / 2.9, power[R2] / 2.9);
    }//2.5


    private void updateEncoderValue() {
        System.arraycopy(currEncoderValue, 0, prevEncoderValue, 0, MOTOR_NUMBER);

        for (int i = 0; i < MOTOR_NUMBER; i++)
            currEncoderValue[i] = motor[i].getCurrentPosition();

        //currEncoderValue[LF] = motorLF.getCurrentPosition();
        //currEncoderValue[LB] = motorLB.getCurrentPosition();
        //currEncoderValue[RF] = motorRF.getCurrentPosition();
        //currEncoderValue[RB] = motorRB.getCurrentPosition();
    }

    private boolean hasArrived() {
        boolean buf = true;

        for (int i = 0; i < MOTOR_NUMBER; i++) {
            if (targetPower[i] > 0) {
                buf = buf && (motor[i].getCurrentPosition() >= targetPosition[i]);
            } else if (targetPower[i] < 0) {
                buf = buf && (motor[i].getCurrentPosition() <= targetPosition[i]);
            }

            /*
            if( targetPosition[i] > 0 )
            {
                buf = buf && ( motor[i].getCurrentPosition() >= targetPosition[i] ) ;
            }
            else if( targetPosition[i] < 0 )
            {
                buf = buf && ( motor[i].getCurrentPosition() <= targetPosition[i] ) ;
            }
             */
        }

        return buf;
    }

    public void runMaxSpeed(double xDistance, double yDistance) {
        final double max_speed = 5.00;

        runDistance(xDistance, yDistance, max_speed);
    }

    public void runHalfSpeed(double xDistance, double yDistance) {
        final double half_speed = 2.5;

        runDistanceHALFSPEED(xDistance, yDistance, half_speed);
    }


    public void mecanumRun(double xSpeed, double ySpeed, double aSpeed) {
        //xSpeed表示 X 轴运动的速度，即左右方向，定义向右为正；
        //ySpeed表示 Y 轴运动的速度，即前后方向，定义向前为正
        //aSpeed表示 yaw 轴自转的角速度，定义逆时针为正 偏航角     向左平移 (-power,0,0)
        //                                                        向右平移（power,0,0）
        //                                                         向左前方 （-power,power,0）
        //

        double speedLF = ySpeed + xSpeed - aSpeed;
        double speedRF = ySpeed - xSpeed + aSpeed;
        double speedRB = ySpeed + xSpeed + aSpeed;
        double speedLB = ySpeed - xSpeed - aSpeed;


        double maxSpeed = Math.max(Math.max(Math.abs(speedLF), Math.abs(speedLB)), Math.max(Math.abs(speedRF), Math.abs(speedRB)));

        if (maxSpeed > Maxpower) {
            speedLF = speedLF / maxSpeed * Maxpower;
            speedLB = speedLB / maxSpeed * Maxpower;
            speedRF = speedRF / maxSpeed * Maxpower;
            speedRB = speedRB / maxSpeed * Maxpower;
        }
        setMotorPower(speedLF, speedRF, speedLB, speedRB);
    }

    public void runDistance(double xDistance, double yDistance, double speed) {
        //setDcMotorMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        //setDcMotorMode(DcMotor.RunMode.RUN_USING_ENCODER);
        //right == +
        /**
         * 本方法利用编码器，引入齿轮比例，编码器度数等几个参数，精确控制机器行进
         * 以右和前为正，输入移动的横向和纵向位移（毫米）就可以让机器跑到指定位置
         */
        double totalDistance = Math.sqrt(Math.pow(xDistance, 2) + Math.pow(yDistance, 2));

        setTargetPosition(xDistance * 0.99, yDistance * 0.99);
        //while( !hasArrived() )mecanumRun(speed / totalDistance * xDistance , speed / totalDistance * yDistance , 0);


        mecanumCalculate(speed / totalDistance * xDistance, speed / totalDistance * yDistance, 0);

        while (!hasArrived() && IsRunning == true) adjustWithEncoder();


        stopAllMotors();
    }

    public void runDistanceHALFSPEED(double xDistance, double yDistance, double speed) {
        //setDcMotorMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        //setDcMotorMode(DcMotor.RunMode.RUN_USING_ENCODER);
        //right == +
        double totalDistance = Math.sqrt(Math.pow(xDistance, 2) + Math.pow(yDistance, 2));

        setTargetPosition(xDistance * 0.99, yDistance * 0.99);
        //while( !hasArrived() )mecanumRun(speed / totalDistance * xDistance , speed / totalDistance * yDistance , 0);


        mecanumCalculate(speed / totalDistance * xDistance, speed / totalDistance * yDistance, 0);

        while (!hasArrived() && IsRunning == true) adjustWithEncoder();


        stopAllMotors();
    }

    private void setTargetPosition(double xDistance, double yDistance) {
        double distance[] = new double[MOTOR_NUMBER];

        distance[R1] = (yDistance * 1.2) - (xDistance * Math.sqrt(2));
        distance[L1] = (yDistance * 1.2) + (xDistance * Math.sqrt(2));
        distance[R2] = (yDistance * 1.2) + (xDistance * Math.sqrt(2));
        distance[L2] = (yDistance * 1.2) - (xDistance * Math.sqrt(2));

        for (int i = 0; i < MOTOR_NUMBER; i++)
            targetPosition[i] = motor[i].getCurrentPosition() + (int) (distance[i] / (WHEEL_DIAMETER * Math.PI) / GEAR_RATIO * COUNTS_PER_MOTOR_REV);
    }

    public void stopAllMotors() {
        setMotorPower(0, 0, 0, 0);
        //setDcMotorMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        //setDcMotorMode(DcMotor.RunMode.RUN_USING_ENCODER);

        for (int i = 0; i < MOTOR_NUMBER; i++)
            targetPower[i] = 0.0;

        for (int i = 0; i < MOTOR_NUMBER; i++)
            correctedPower[i] = 0.0;

        for (int i = 0; i < MOTOR_NUMBER; i++)
            deltaValue[i] = 0.0;

        for (int i = 0; i < MOTOR_NUMBER; i++)
            targetPosition[i] = 0;

        for (int i = 0; i < MOTOR_NUMBER; i++)
            prevEncoderValue[i] = 0;

        for (int i = 0; i < MOTOR_NUMBER; i++)
            currEncoderValue[i] = 0;
    }

    public void mecanumCalculate(double xSpeed, double ySpeed, double aSpeed) {
        targetPower[L1] = ySpeed + xSpeed - aSpeed;
        targetPower[L2] = ySpeed - xSpeed - aSpeed;
        targetPower[R1] = ySpeed - xSpeed + aSpeed;
        targetPower[R2] = ySpeed + xSpeed + aSpeed;

        double maxSpeed = Math.max(Math.max(Math.abs(targetPower[L1]), Math.abs(targetPower[L2])), Math.max(Math.abs(targetPower[R2]), Math.abs(targetPower[R2])));

        if (maxSpeed > MAX_MOTOR_POWER) {
            targetPower[L1] = targetPower[L1] / maxSpeed * MAX_MOTOR_POWER;
            targetPower[L2] = targetPower[L2] / maxSpeed * MAX_MOTOR_POWER;
            targetPower[R1] = targetPower[R1] / maxSpeed * MAX_MOTOR_POWER;
            targetPower[R2] = targetPower[R2] / maxSpeed * MAX_MOTOR_POWER;
        }
    }


    /*public void turn(int angle) {
        int currentDirection = gyro.getIntegratedZValue();

        int targetDirection = currentDirection + angle;

        do {
            currentDirection = gyro.getIntegratedZValue();

            double turningPower = (targetDirection - currentDirection) * TURNING_POWER;

            if (Math.abs(turningPower) < MIN_TURNING_POWER) {
                if (turningPower != 0) {
                    turningPower = turningPower / Math.abs(turningPower) * MIN_TURNING_POWER;
                }
            }

            turningPower = Range.clip(turningPower, -MAX_TURNING_POWER, MAX_TURNING_POWER);

            //mecanumRun(0, 0, turningPower);
            setMotorPower(-turningPower, turningPower);

        }
        while (IsRunning == true && angle < 0 ? (currentDirection > targetDirection + EPS_ANGLE) : (currentDirection < targetDirection - EPS_ANGLE));

        stopAllMotors();
    }
    public void turnFWB(int angle) {
        int currentDirection = gyro.getIntegratedZValue();

        int targetDirection = currentDirection + angle;

        while (IsRunning == true && angle < 0 ? (currentDirection > targetDirection + EPS_ANGLE) : (currentDirection < targetDirection - EPS_ANGLE))
        {
            currentDirection = gyro.getIntegratedZValue();

            double turningPower = (targetDirection - currentDirection) * TURNING_POWER;

            if (Math.abs(turningPower) < MIN_TURNING_POWER) {
                if (turningPower != 0) {
                    turningPower = turningPower / Math.abs(turningPower) * MIN_TURNING_POWER;
                }
            }

            turningPower = Range.clip(turningPower, -MAX_TURNING_POWER, MAX_TURNING_POWER);

            //mecanumRun(0, 0, turningPower);
            setMotorPower(-turningPower, turningPower);
        }
        stopAllMotors();
    }
    */
    public void rotateToTargetDirection(double targetDirection) {
        /**
         * 基于机器人初始化时的朝向设定目标位置
         * targetDirection 的取值范围为 -179 ~ 180
         * 初始化时的朝向参数值为 0
         * 逆时针方向为正值，顺时针方向为负值
         */

     /*   int currentDirection = gyro.getIntegratedZValue();

        while (currentDirection > (ROUND / 2) && IsRunning == true) currentDirection -= ROUND;
        while (currentDirection <= -(ROUND / 2) && IsRunning == true) currentDirection += ROUND;

        int deltaDirection = targetDirection - currentDirection;

        if (deltaDirection > (ROUND / 2) && IsRunning == true) deltaDirection -= ROUND;
        else if (deltaDirection <= -(ROUND / 2) && IsRunning == true) deltaDirection += ROUND;
*/
       //turn(deltaDirection);
        IMUturn(targetDirection);
    }

    public void imut(double angle) {
        double currentDirection = angle();

        double targetDirection = currentDirection + angle;

        do {
            currentDirection = angle();

            double turningPower = (targetDirection - currentDirection) * TURNING_POWER;

            if (Math.abs(turningPower) < MIN_TURNING_POWER) {
                if (turningPower != 0) {
                    turningPower = turningPower / Math.abs(turningPower) * MIN_TURNING_POWER;
                }
            }

            turningPower = Range.clip(turningPower, -MAX_TURNING_POWER, MAX_TURNING_POWER);

            //mecanumRun(0, 0, turningPower);
            setMotorPower(-turningPower, turningPower);

        }
        while (IsRunning == true && angle < 0 ? (currentDirection > targetDirection + EPS_ANGLE) : (currentDirection < targetDirection - EPS_ANGLE));

        stopAllMotors();
    }
    public void imuturn(double targetDirection)
    {
        /**
         * 基于机器人初始化时的朝向设定目标位置
         * targetDirection 的取值范围为 -179 ~ 180
         * 初始化时的朝向参数值为 0
         * 逆时针方向为正值，顺时针方向为负值
         */

        double currentDirection =angle();

        while (currentDirection > (ROUND / 2) && IsRunning == true) currentDirection -= ROUND;
        while (currentDirection <= -(ROUND / 2) && IsRunning == true) currentDirection += ROUND;

        double deltaDirection = targetDirection - currentDirection;

        if (deltaDirection > (ROUND / 2) && IsRunning == true) deltaDirection -= ROUND;
        else if (deltaDirection <= -(ROUND / 2) && IsRunning == true) deltaDirection += ROUND;

        imut(deltaDirection);
        // turnLXH(deltaDirection);
    }

    public double angle(){
        angles = imu.getAngularOrientation(AxesReference.INTRINSIC, AxesOrder.ZYX, AngleUnit.DEGREES);
        return angles.firstAngle;
    }





    public double getDistanceCM()  {return rangeSensor.getDistance(DistanceUnit.CM);   }




    public void Standofwall(double distance)    //左侧range
    {
        while (rangeSensor.getDistance(DistanceUnit.CM) <= distance  || rangeSensor.getDistance(DistanceUnit.CM) >= distance + 1) {
            if (rangeSensor.getDistance(DistanceUnit.CM) < distance ) {
                while (rangeSensor.getDistance(DistanceUnit.CM) < distance ) {
                    double y = rangeSensor.getDistance(DistanceUnit.CM);
                    double error = distance - y;
                    rangesumR = rangesumR + error;
                    double power = (Math.abs(rangeSensor.getDistance(DistanceUnit.CM) * Kp1)) + (rangesumR / Ki1);
                    mecanumRun(power, 0, 0);
                }
            }
            else if (rangeSensor.getDistance(DistanceUnit.CM) > distance )
            {
                while (rangeSensor.getDistance(DistanceUnit.CM) > distance )
                {
                    double y = rangeSensor.getDistance(DistanceUnit.CM);
                    double error = distance - y;
                    rangesumR = rangesumR + error;
                    double power = (Math.abs(rangeSensor.getDistance(DistanceUnit.CM) * Kp1)) + (rangesumR / Ki1);
                    mecanumRun(-power, 0, 0);
                }
              }
          }
      }

    public void Jewelry() {
        //servo2.setPosition(0.1);//down
        
        if (colorDetected() == teamColor) //ball detected
        {
            // rotateToTargetDirection(turnDirection);
            servoW.setPosition(Wposition - 0.08);//0.15
        } else if (colorDetected() != teamColor)//ball detected
        {
            //rotateToTargetDirection(-turnDirection);
            servoW.setPosition(Wposition + 0.08);
        }
        //servo2.setPosition(0.9);//rise
        //rotateToTargetDirection(0);
    }


    public void Distancejudge()//正式使用前请重新关联新传感器和名字和调平移方向
    {
        double v;
        double deltav;
        double power;
        double EPSdistance = 2;//mm
        {
            if (sensorDistance.getDistance(DistanceUnit.MM) <= 20) {
                while (sensorDistance.getDistance(DistanceUnit.MM) <= 20 + EPSdistance && sensorDistance.getDistance(DistanceUnit.MM) >= 20 - EPSdistance) {
                    v = sensorDistance.getDistance(DistanceUnit.MM);
                    deltav = 20 - v;
                    power = deltav / 500;
                    mecanumRun(power, 0, 0);
                }
            } else if (sensorDistance.getDistance(DistanceUnit.MM) >= 20) {
                while (sensorDistance.getDistance(DistanceUnit.MM) >= 20 + EPSdistance && sensorDistance.getDistance(DistanceUnit.MM) <= 20 - EPSdistance) {
                    v = sensorDistance.getDistance(DistanceUnit.MM);
                    deltav = v - 20;
                    power = deltav / 500;
                    mecanumRun(-power, 0, 0);
                }
            }
            stopAllMotors();
        }
    }


    public void DistanceSWITCH() {
        double EPSdistance = 3;//mm
        if (sensorDistance.getDistance(DistanceUnit.MM) <= 20 + EPSdistance && sensorDistance.getDistance(DistanceUnit.MM) > 20 - EPSdistance) {
            stopAllMotors();
        } else {
            Distancejudge();
        }
    }

    void servoDown() {
        servoJ.setPosition(0.535);//0.535
    }

    void servoLift() {
        servoJ.setPosition(0.955);
    }//0.95

    public  double getangle()
    {
        angles = imu.getAngularOrientation(AxesReference.INTRINSIC, AxesOrder.ZYX, AngleUnit.DEGREES);
        return angles.firstAngle;
    }
    void IMUturn(double angle) {

        if (getangle() < angle && IsRunning) {
            while (getangle() < angle && IsRunning) {
                double x = getangle();
                double error = angle - x;
                errorsum = errorsum + error;
                double power = ((Math.abs(angle - x)) * Kp) + (errorsum / Ki);
                setMotorPower(-power, +power);
            }
        } else if (getangle() > angle && IsRunning) {
            while (getangle() > angle && IsRunning) {
                double x = getangle();
                double error = angle - x;
                errorsum = errorsum + error;
                double power = ((Math.abs(angle - x)) * Kp) + (errorsum / Ki);
                setMotorPower(+power, -power);
            }
        }
    }
    /*
    void turnLXH(int angle) {

            if (gyro.getIntegratedZValue() < angle - 1&&IsRunning) {
                while (gyro.getIntegratedZValue() < angle - 1&&IsRunning) {
                    double x = gyro.getIntegratedZValue();
                    double error = angle - x;
                    errorsum = errorsum + error;
                    double power = ((Math.abs(angle - x)) * Kp) + (errorsum / Ki);
                    setMotorPower(-power, +power);
                }
            } else if (gyro.getIntegratedZValue() > angle + 1&&IsRunning) {
                while (gyro.getIntegratedZValue() > angle + 1&&IsRunning)

                {
                    double x = gyro.getIntegratedZValue();
                    double error = angle - x;
                    errorsum = errorsum + error;
                    double power = ((Math.abs(angle - x)) * Kp) + (errorsum / Ki);
                    setMotorPower(+power, -power);
                }
            }
        }
    void turnLLT(int angle) {

            if (gyro.getIntegratedZValue() < angle&&IsRunning )
            {
                while (IsRunning&&gyro.getIntegratedZValue() < angle - 1) {
                    double x = gyro.getIntegratedZValue();
                    double power =0.2;
                    setMotorPower(-power, +power);
                }
            }
            else if (gyro.getIntegratedZValue() > angle&&IsRunning )
            {
                while (gyro.getIntegratedZValue() > angle + 1&&IsRunning)

                {
                    double x = gyro.getIntegratedZValue();
                    double power = 0.2;
                    setMotorPower(+power, -power);
                }
            }
        }

*/
    boolean colorDetected() {
        if (sensorColor.red() > sensorColor.blue()) {
            return RED_COLOR;
        } else return BLUE_COLOR;
    }

    void CollectGlyph(double powerL, double powerR) {//collect
        motorSL.setPower(powerL); //0.4
        motorSR.setPower(-powerR); //-1
    }

    void GlyphServo() {
        servoS1.setPosition(1);
        servoS2.setPosition(0);
    }

    void GlyphServoPin() {
        servoS1.setPosition(0.428);
        servoS2.setPosition(0.588);
    }

    void GlyphServoDown() {
        servoS1.setPosition(0.427);
        servoS2.setPosition(0.589);
    }

    void SOCL2() {
        boolean open;
        do {
            mecanumRun(0, -0.16, 0);
            if (sensorDistance.getDistance(DistanceUnit.CM) <= 30) {
                open = false;
            } else {
                open = true;
            }
        }
        while (open == true && IsRunning == true);

    }

    void GlyphRISE() {
    }

    void CRYPTOBOX()//Put the glyphs into the
    {
        runHalfSpeed(0, -100);

        GlyphServo();

        runHalfSpeed(0, 100);
    }

    public void runtobox() {
        runDistance(-picturetarget, 0, 3);
    }

    public void STOPmotor() {
        int position = 0;
        position = motorRISE.getCurrentPosition();
        if (Math.abs(motorRISE.getCurrentPosition() - position) >= 5) {
            if (motorRISE.getCurrentPosition() > position) {
                motorRISE.setPower(0.1);
            } else if (motorRISE.getCurrentPosition() < position) {
                motorRISE.setPower(-0.1);
            }
        } else {
            motorRISE.setPower(0);
        }

    }

    public void NAVI() {
        if (sensorDistance.getDistance(DistanceUnit.CM) <= 8.6)
        {            //12.65
            if (sensorDistance.getDistance(DistanceUnit.CM) <= 8.6 )
            {
                double v = sensorDistance.getDistance(DistanceUnit.CM);
                double deltav = 8.6 - v;
                double power = deltav / 55;
                power = Range.clip(power, -0.15, 0.15);
                mecanumRun(power, 0, 0);
            }
        }
        else if (sensorDistance.getDistance(DistanceUnit.CM) >= 8.6)
        {
            if (sensorDistance.getDistance(DistanceUnit.CM) >= 8.6 )
            {
                double v = sensorDistance.getDistance(DistanceUnit.CM);
                double deltav = v - 8.6;
                double power = deltav / 55;
                power = Range.clip(power, -0.15, 0.15);
                mecanumRun(-power, 0, 0);
            }
        }
        else stopAllMotors();
    }

    void Se03(double distance)
    {

        if (rangeSensor.getDistance(DistanceUnit.CM) <= distance&& IsRunning == true) {
            while (rangeSensor.getDistance(DistanceUnit.CM) <= distance&& IsRunning == true) {
                double v = rangeSensor.getDistance(DistanceUnit.CM);
                double deltav = distance - v;
                double power = deltav /35;//45
                if (Math.abs(power) < DMINPOWER) {
                    if (power != 0) {
                        power = (power / Math.abs(power)) * DMINPOWER;
                    }
                }
                power = Range.clip(power, -0.7, 0.7);
                setMotorPower(power, -power, -power, power);

            }
        }
        else if (rangeSensor.getDistance(DistanceUnit.CM) >= distance&& IsRunning == true)
        {
            while (rangeSensor.getDistance(DistanceUnit.CM) >= distance && IsRunning == true) {
                double v = rangeSensor.getDistance(DistanceUnit.CM);
                double deltav =  v -distance;
                double power = deltav / 35;//45
                if (Math.abs(power) <DMINPOWER) {
                    if (power != 0) {
                        power = (power / Math.abs(power)) * DMINPOWER;
                    }
                }
                power = Range.clip(power, -0.7, 0.7);
                setMotorPower(-power,power,power,-power);
            }
        }
        else
        {
            stopAllMotors();
        }

    }
    void stepoffwall(double distance)
    {

        if (rangeSensor2.getDistance(DistanceUnit.CM) <= distance && IsRunning == true) {
            while (rangeSensor2.getDistance(DistanceUnit.CM) <= distance && IsRunning == true) {
                double v = rangeSensor2.getDistance(DistanceUnit.CM);
                double deltav = distance - v;
                double power = deltav / 50;//45
                if (Math.abs(power) < DMINPOWER) {
                    if (power != 0) {
                        power = (power / Math.abs(power)) * DMINPOWER;
                    }
                }
                setMotorPower(-power, power, power, -power);

            }
        }
        else if (rangeSensor2.getDistance(DistanceUnit.CM) >= distance && IsRunning == true)
        {
            while (rangeSensor2.getDistance(DistanceUnit.CM) >= distance && IsRunning == true) {
                double v = rangeSensor2.getDistance(DistanceUnit.CM);
                double deltav =  v -distance;
                double power = deltav / 50;//45
                if (Math.abs(power) <DMINPOWER) {
                    if (power != 0) {
                        power = (power / Math.abs(power)) * DMINPOWER;
                    }
                }
                setMotorPower(power,-power,-power,power);
            }
        }

        else
        {
            stopAllMotors();
        }

    }
    void dis()
    {

        if (sensorDistance.getDistance(DistanceUnit.CM) <= distance && IsRunning == true) {
            while (sensorDistance.getDistance(DistanceUnit.CM) <= distance&& IsRunning == true) {
                double v = sensorDistance.getDistance(DistanceUnit.CM);
                double deltav = distance - v;
                double power = deltav / 50;
                if (Math.abs(power) < NaMINPOWER) {
                    if (power != 0) {
                        power = power / Math.abs(power) * NaMINPOWER;
                    }
                }
                power = Range.clip(power, -0.15, 0.15);
                setMotorPower(power, -power, -power, power);

            }
        }
        else if (sensorDistance.getDistance(DistanceUnit.CM) >= distance && IsRunning == true)
        {
            while (sensorDistance.getDistance(DistanceUnit.CM) >= distance && IsRunning == true) {
                double v = sensorDistance.getDistance(DistanceUnit.CM);
                double deltav =  v -distance;
                double power = deltav /50;
                if (Math.abs(power) <NaMINPOWER) {
                    if (power != 0) {
                        power = power / Math.abs(power) * NaMINPOWER;
                    }
                }
                power = Range.clip(power, -0.15, 0.15);
                setMotorPower(-power,power,power,-power);
            }
        }
        else
        {
            stopAllMotors();
        }

    }
}