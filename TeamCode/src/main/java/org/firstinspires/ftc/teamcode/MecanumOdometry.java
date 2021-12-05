/*
About this File:
Not really sure what this is even used for but it exists??
 */



package org.firstinspires.ftc.teamcode;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.util.ElapsedTime;
import org.firstinspires.ftc.robotcontroller.external.samples.HardwarePushbot;
import org.firstinspires.ftc.teamcode.IMU;

//@Autonomous
public class MecanumOdometry{// extends LinearOpMode {
    // Encoder constants
    public static final double CM_PER_TICK_A = -100.0 / 1673.0;
    public static final double CM_PER_TICK_B = -100.0 / 1693.0;
    public static final double CM_PER_TICK_C = -100.0 / 1434.0;
    // This is theoretically correct, but may need to be changed in practice
    public static final double STRAFE_CONSTANT = .5;

    // The current position and orientation of the robot
    private double x = 0;
    private double y = 0;
    private double theta = 0;

    // The angle at which the robot was facing when the OpMode started
    public float startingAngle = 0;

    // The previous encoder values
    private double oldA = 0;
    private double oldB = 0;
    private double oldC = 0;

    private double dx = 0;
    private double dy = 0;
    // The IMU handler - tracks the robot's orientation
    public IMU imu = new IMU();

    /**
     * Should be called in when the OpMode is initialized
     */
    public void init(HardwareMap hardwareMap) {
        imu.initIMU(hardwareMap);
    }

    public double getX() {
        return x;
    }

    public double getY() {
        return y;
    }

    public double getTheta() {
        return theta;
    }

    /**
     * Should be called when the OpMode is started
     * Once again
     *
     * @param a The current front-right encoder value
     * @param b The current front-left encoder value
     * @param c The current back-left encoder value
     */
    public void start(double a, double b, double c) {
        imu.update();
        startingAngle = imu.getZAngle();
        x = 0;
        y = 0;
        oldA = a;
        oldB = b;
        oldC = c;
    }

    /**
     * Updates the position and orientation based on new encoder values and the IMU
     *
     * @param a The current front-right encoder value
     * @param b The current front-left encoder value
     * @param c The current back-left encoder value
     */
    public void update(double a, double b, double c) {
        // Calculate the differences between the new and old encoder values
        // and convert them to centimeters.
        double dA = (a - oldA) * CM_PER_TICK_A;
        double dB = (b - oldB) * CM_PER_TICK_B;
        double dC = (c - oldC) * CM_PER_TICK_C;


        oldA = a;
        oldB = b;
        oldC = c;

        // Update the orientation using data from the IMU
        imu.update();
        theta = imu.getZAngle() - startingAngle;

        // Update the total displacement using the orientation and encoder displacements
        double averageEncoderChange = (Math.abs(dB) + Math.abs(dC)) / 2;
        if (dB > 0 && dC < 0) {
            dx = averageEncoderChange * STRAFE_CONSTANT * Math.cos(theta);
            dy = averageEncoderChange * STRAFE_CONSTANT * Math.sin(theta);
        } else if (dB < 0 && dC > 0) {
            dx = -averageEncoderChange * STRAFE_CONSTANT * Math.cos(theta);
            dy = -averageEncoderChange * STRAFE_CONSTANT * Math.sin(theta);
        } else {
            dx = (dA + dB) / 2 * Math.sin(theta);
            dy = (dA + dB) / 2 * Math.cos(theta);
        }

        // Add the displacement to the total position
        this.x += dx;
        this.y += dy;
    }

    /*
    DcMotor RFmotor;
    DcMotor RBmotor;
    DcMotor LFmotor;
    DcMotor LBmotor;
    DcMotor turnTable;
    DcMotor sliderSpool;
    DcMotor intakeMotor;
    public static int wheelDiameter = 96;
    public static double circumference = wheelDiameter* 3.1415;
    public static double ticksPerRotation = 537.7;
    public static double tileLength = 457.2;
    public static double rotationsPerTile= tileLength/circumference;
    public static double ticksPerTile= rotationsPerTile * ticksPerRotation;
    public static double ticksPerMm= ticksPerRotation/circumference;
    private final ElapsedTime runtime = new ElapsedTime();
    public static int slideLevelOne= 537*5;
    public static int slideLevelTwo= 537*10;
    public static int slideLevelThree= 537*20;


    @Override
    public void runOpMode() throws InterruptedException {

        RFmotor = hardwareMap.get(DcMotor.class, "rightfront");
        RBmotor = hardwareMap.get(DcMotor.class, "rightback");
        LFmotor = hardwareMap.get(DcMotor.class, "leftfront");
        LBmotor = hardwareMap.get(DcMotor.class, "leftback");
        turnTable = hardwareMap.get(DcMotor.class, "turntable");
        sliderSpool = hardwareMap.get(DcMotor.class, "slider");
        intakeMotor= hardwareMap.get(DcMotor.class, "intake");


        telemetry.speak("Press play to start");
        RFmotor.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        RBmotor.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        LFmotor.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        LBmotor.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);

        sliderSpool.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        intakeMotor.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);


        RFmotor.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        RBmotor.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        LBmotor.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        LBmotor.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        sliderSpool.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        intakeMotor.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        telemetry.addData("Status", "Resetting Encoders");    //
        telemetry.update();
        waitForStart();

        encoderDrive(.75,646.577, 0, 0, 646.577 );
        sliderSpool.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        intakeMotor.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        sliderSpool.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        turnTable.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        sleep(500);
        sliderSpool.setTargetPosition(slideLevelTwo);
        sliderSpool.setPower(.65);
        intakeMotor.setTargetPosition(5400);
        intakeMotor.setPower(.65);
        sleep(500);
        encoderDrive(.75,-380,-380,-380,-380 );
        sleep(500);
        encoderDrive(.75,-1150,1150,1150,-1150 );

        turnTable.setTargetPosition(5400);
        encoderDrive(.75,457.2,457.2,457.2,457.2);
        telemetry.addData("Path", "Complete");
        telemetry.update();

    }



    public void encoderDrive( double speed, double frontLeft, double frontRight, double backLeft, double backRight){
       int leftFrontTarget;
       int rightFrontTarget;
       int leftBackTarget;
       int rightBackTarget;

       leftFrontTarget = LFmotor.getCurrentPosition()+ (int)(frontLeft*ticksPerMm);
       rightFrontTarget = RFmotor.getCurrentPosition()+ (int)(frontRight*ticksPerMm);
       leftBackTarget = LBmotor.getCurrentPosition()+ (int)(backLeft*ticksPerMm);
       rightBackTarget = RBmotor.getCurrentPosition()+ (int)(backRight*ticksPerMm);

       LFmotor.setTargetPosition(leftFrontTarget);
       RFmotor.setTargetPosition(rightFrontTarget);
       LBmotor.setTargetPosition(leftBackTarget);
       RBmotor.setTargetPosition(rightBackTarget);

       LFmotor.setMode(DcMotor.RunMode.RUN_TO_POSITION);
       RFmotor.setMode(DcMotor.RunMode.RUN_TO_POSITION);
       RBmotor.setMode(DcMotor.RunMode.RUN_TO_POSITION);
       LBmotor.setMode(DcMotor.RunMode.RUN_TO_POSITION);


       LFmotor.setPower(Math.abs(speed));
       RFmotor.setPower(Math.abs(speed));
       LBmotor.setPower(Math.abs(speed));
       RBmotor.setPower(Math.abs(speed));



       LFmotor.setPower(0);
       RFmotor.setPower(0);
       RBmotor.setPower(0);
       LBmotor.setPower(0);



       LFmotor.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
       RFmotor.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
       RBmotor.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
       LBmotor.setMode(DcMotor.RunMode.RUN_USING_ENCODER);

    }
    */
    }



