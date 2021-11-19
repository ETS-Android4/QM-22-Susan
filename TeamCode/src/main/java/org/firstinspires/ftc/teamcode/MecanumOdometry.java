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

import org.firstinspires.ftc.robotcontroller.external.samples.HardwarePushbot;


@Autonomous
public class MecanumOdometry extends LinearOpMode {
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
    public static double tileLength = 609.6;
    public static double rotationsPerTile= tileLength/circumference;
    public static double ticksPerTile= rotationsPerTile * ticksPerRotation;



    // Encoder constants
//    public static final double CM_PER_TICK_A = -100.0 / 1673.0;
//    public static final double CM_PER_TICK_B = -100.0 / 1693.0;
//    public static final double CM_PER_TICK_C = -100.0 / 1434.0;
//    // This is theoretically correct, but may need to be changed in practice
//    public static final double STRAFE_CONSTANT = .5;
//
//    // The current position and orientation of the robot
//    private double x = 0;
//    private double y = 0;
//    private double theta = 0;
//
//    // The angle at which the robot was facing when the OpMode started
//    public float startingAngle = 0;
//
//    // The previous encoder values
//    private double oldA = 0;
//    private double oldB = 0;
//    private double oldC = 0;
//
//    private double dx = 0;
//    private double dy = 0;
//    // The IMU handler - tracks the robot's orientation


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

        RFmotor.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        RBmotor.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        LBmotor.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        LBmotor.setMode(DcMotor.RunMode.RUN_USING_ENCODER);

        waitForStart();


    }
    public void encoderDrive( double speed, double frontLeft, double frontRight, double backLeft, double backRight){
   int leftFrontTarget;
   int rightFrontTarget;
   int leftBackTarget;
   int rightBackTarget;

   leftFrontTarget = LFmotor.getCurrentPosition()+ (int)(frontLeft)
    }


    /**
     * Should be called in when the OpMode is initialized
     */






    @Override
    public void loop() {

    }


    //public void init(HardwareMap hardwareMap) { imu.initIMU(hardwareMap); }

//    public double getX() {
//        return x;
//    }
//
//    public double getY() {
//        return y;
//    }
//
//    public double getTheta() {
//        return theta;
//    }

    /**
     * Should be called when the OpMode is started
     * Once again
     *
     * @param a The current front-right encoder value
     * @param b The current front-left encoder value
     * @param c The current back-left encoder value
     */


    /**
     * Updates the position and orientation based on new encoder values and the IMU
     *
     * @param a The current front-right encoder value
     * @param b The current front-left encoder value
     * @param c The current back-left encoder value
     */


//    public void update(double a, double b, double c) {
//        // Calculate the differences between the new and old encoder values
//        // and convert them to centimeters.
//        double dA = (a - oldA) * CM_PER_TICK_A;
//        double dB = (b - oldB) * CM_PER_TICK_B;
//        double dC = (c - oldC) * CM_PER_TICK_C;
//
//
//        oldA = a;
//        oldB = b;
//        oldC = c;
//
//
//        // Update the total displacement using the orientation and encoder displacements
//        double averageEncoderChange = (Math.abs(dB) + Math.abs(dC)) / 2;
//        if (dB > 0 && dC < 0) {
//            dx = averageEncoderChange * STRAFE_CONSTANT * Math.cos(theta);
//            dy = averageEncoderChange * STRAFE_CONSTANT * Math.sin(theta);
//        } else if (dB < 0 && dC > 0) {
//            dx = -averageEncoderChange * STRAFE_CONSTANT * Math.cos(theta);
//            dy = -averageEncoderChange * STRAFE_CONSTANT * Math.sin(theta);
//        } else {
//            dx = (dA + dB) / 2 * Math.sin(theta);
//            dy = (dA + dB) / 2 * Math.cos(theta);
//        }
//
//        // Add the displacement to the total position
//        this.x += dx;
//        this.y += dy;
//
//
//
    }



