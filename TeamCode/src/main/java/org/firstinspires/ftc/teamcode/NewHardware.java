package org.firstinspires.ftc.teamcode;

import com.qualcomm.hardware.bosch.BNO055IMU;
import com.qualcomm.hardware.bosch.JustLoggingAccelerationIntegrator;
import com.qualcomm.hardware.rev.Rev2mDistanceSensor;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.ColorSensor;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.DistanceSensor;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.util.ElapsedTime;
import com.qualcomm.robotcore.util.Range;

import org.firstinspires.ftc.robotcore.external.hardware.camera.WebcamName;
import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.robotcore.external.navigation.AxesOrder;
import org.firstinspires.ftc.robotcore.external.navigation.AxesReference;
import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit;
import org.firstinspires.ftc.robotcore.external.navigation.Orientation;
import org.opencv.core.Point;
import org.openftc.easyopencv.OpenCvCamera;
import org.openftc.easyopencv.OpenCvCameraFactory;
import org.openftc.easyopencv.OpenCvCameraRotation;
import org.openftc.easyopencv.OpenCvWebcam;

import static org.firstinspires.ftc.robotcore.external.BlocksOpModeCompanion.telemetry;
import static org.firstinspires.ftc.teamcode.Constants.DEFAULT_ACCELERATION_INCREMENT;
import static org.firstinspires.ftc.teamcode.Constants.GEAR_RATIO_MULTIPLIER;
import static org.firstinspires.ftc.teamcode.Constants.INTAKE_SPEED;
import static org.firstinspires.ftc.teamcode.Constants.blueturnTablePower;
import static org.firstinspires.ftc.teamcode.Constants.redturnTablePower;
import static org.firstinspires.ftc.teamcode.Constants.slidePowerDown;
import static org.firstinspires.ftc.teamcode.Constants.slidePowerUp;

/*
import static org.firstinspires.ftc.teamcode.simpleBotCode.simpleBotConstants.DEFAULT_ACCELERATION_INCREMENT;
import static org.firstinspires.ftc.teamcode.simpleBotCode.simpleBotConstants.FLYWHEEL_SPEED;
import static org.firstinspires.ftc.teamcode.simpleBotCode.simpleBotConstants.GEAR_RATIO_MULTIPLIER;
import static org.firstinspires.ftc.teamcode.simpleBotCode.simpleBotConstants.INTAKE_SPEED;
import static org.firstinspires.ftc.teamcode.simpleBotCode.simpleBotConstants.LIFTER_MOTOR_DOWN;
import static org.firstinspires.ftc.teamcode.simpleBotCode.simpleBotConstants.LIFTER_MOTOR_UP;
import static org.firstinspires.ftc.teamcode.simpleBotCode.simpleBotConstants.ringDistanceArray;
*/

public class NewHardware {
    /* Public OpMode members. */
    //motors
    public DcMotor RFmotor = null;
    public DcMotor LFmotor = null;
    public DcMotor RBmotor = null;
    public DcMotor LBmotor = null;
    public DcMotor turnTable = null;
    public DcMotor sliderSpool = null;
    public DcMotor intakeMotor = null;

//    public DistanceSensor intakeSensor = null;
//    Rev2mDistanceSensor sensorTimeOfFlight = (Rev2mDistanceSensor) intakeSensor;

    public DistanceSensor shippingSensor = null;
    Rev2mDistanceSensor sensorTimeOfFlight1 = (Rev2mDistanceSensor) shippingSensor;

    public Servo turretBottom = null;
    public Servo turretLift = null;
    public Servo boxServo = null;

    private ShippingElementPipeline shippingPipeline;
    private OpenCvWebcam webcam;
    private Point elementPosition;
    /*
    public DistanceSensor wobbleRangeSensor = null;
    public DistanceSensor hopperRangeSensor = null;
    public DistanceSensor sideRangeSensor = null;
    Rev2mDistanceSensor sensorTimeOfFlight = (Rev2mDistanceSensor) sideRangeSensor;
    public DistanceSensor wobble2mRangeSensor = null;
    Rev2mDistanceSensor sensorTimeOfFlight1 = (Rev2mDistanceSensor) wobble2mRangeSensor;
    public DistanceSensor angleRangeSensor1 = null;
    public DistanceSensor angleRangeSensor2 = null;


    public ColorSensor wobbleColorSensor = null;
    //servos
    //public Servo lifter = null;
    public Servo shooter = null;
    public Servo wobbleServo = null;
    public Servo leftBlocker = null;
    public Servo rightBlocker = null;
    */

    /*
    proof that I know how to code:- DePre

     */

    //imu:
    public BNO055IMU imu;

    Orientation angles;
    Orientation lastAngles = new Orientation();
    public double globalAngle, power = .30, correction;


    /* local OpMode members. */
    HardwareMap hardwareMap = null;
    private final ElapsedTime period = new ElapsedTime();
    LinearOpMode opMode;


    /* Initialize standard Hardware interfaces */
    public void init(HardwareMap ahwMap, LinearOpMode opMode) {

        this.opMode = opMode;
        //IMU Setup:
        // Set up the parameters with which we will use our IMU. Note that integration
        // algorithm here just reports accelerations to the logcat log; it doesn't actually
        // provide positional information.
        BNO055IMU.Parameters parameters = new BNO055IMU.Parameters();
        parameters.angleUnit = BNO055IMU.AngleUnit.DEGREES;
        parameters.accelUnit = BNO055IMU.AccelUnit.METERS_PERSEC_PERSEC;
        //parameters.calibrationDataFile = "BNO055IMUCalibration.json"; // see the calibration sample opmode
        parameters.loggingEnabled = true;
        parameters.loggingTag = "IMU";
        parameters.accelerationIntegrationAlgorithm = new JustLoggingAccelerationIntegrator();


        // Save reference to Hardware map
        hardwareMap = ahwMap;

        // Define and Initialize Motors
        RFmotor = hardwareMap.get(DcMotor.class, "rightfront");
        RBmotor = hardwareMap.get(DcMotor.class, "rightback");
        LFmotor = hardwareMap.get(DcMotor.class, "leftfront");
        LBmotor = hardwareMap.get(DcMotor.class, "leftback");

        turnTable = hardwareMap.get(DcMotor.class, "turntable");
        sliderSpool = hardwareMap.get(DcMotor.class, "slider");
        intakeMotor= hardwareMap.get(DcMotor.class, "intake");


        // Define and Initialize Servos
        /*
        shooter = hwMap.get(Servo.class, "shooter");
        */
        turretBottom = hardwareMap.get(Servo.class, "turretBottom");
        turretLift = hardwareMap.get(Servo.class, "turretLift");
        boxServo = hardwareMap.get(Servo.class, "boxServo");

        //Define and Initialize BNO055IMU
        imu = hardwareMap.get(BNO055IMU.class, "imu");
        imu.initialize(parameters);

/*
        int cameraMonitorViewId = hardwareMap.appContext.getResources().getIdentifier("cameraMonitorViewId", "id", hardwareMap.appContext.getPackageName());
        webcam = OpenCvCameraFactory.getInstance().createWebcam(hardwareMap.get(WebcamName.class, "Webcam 1"), cameraMonitorViewId);
        shippingPipeline = new ShippingElementPipeline();
        webcam.setPipeline(shippingPipeline);
        webcam.setMillisecondsPermissionTimeout(2500);
        webcam.openCameraDeviceAsync(new OpenCvCamera.AsyncCameraOpenListener() {
            @Override
            public void onOpened() {
                webcam.startStreaming(640, 480, OpenCvCameraRotation.UPRIGHT);
                //telemetry.addData("Camera status:", "initialized");
                telemetry.update();
            }

            @Override
            public void onError(int errorCode) {
                // This will be called if the camera could not be opened
            }
        });
        while(shippingPipeline.getPoint() == null){
            telemetry.addData("camera ready?", "false");
            telemetry.addData("pipeline chosen", "Shipping");
            telemetry.update();
        }
        elementPosition = shippingPipeline.getPoint();
        telemetry.addData("camera ready?", "true");
*/
        /*
        sideRangeSensor = hwMap.get(DistanceSensor.class, "side_range_sensor");
        */


//        intakeSensor = hardwareMap.get(DistanceSensor.class, "intake_sensor");
        //shippingSensor = hardwareMap.get(DistanceSensor.class, "shipping_sensor");


        // Define and Initialize LED's
        // blinkinLedDriver = hwMap.get(RevBlinkinLedDriver.class, "blinkin");

        /*

        // Set rb behavior when power is zero (BRAKE = brake, FLOAT = no brake)
        RFmotor.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        LFmotor.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        RBmotor.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        LBmotor.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);


        // Set all motors to zero power for initialization
        RFmotor.setPower(0);
        LFmotor.setPower(0);
        RBmotor.setPower(0);
        LBmotor.setPower(0);
        flywheel.setPower(0);
        intake.setPower(0);
        */

        RFmotor.setDirection(DcMotor.Direction.FORWARD);
        LFmotor.setDirection(DcMotor.Direction.REVERSE);
        RBmotor.setDirection(DcMotor.Direction.FORWARD);
        LBmotor.setDirection(DcMotor.Direction.REVERSE);

        RFmotor.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        RBmotor.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        LFmotor.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        LBmotor.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        sliderSpool.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);

        //intakeMotor.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        // Set all motor encoder options.
        RFmotor.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        RBmotor.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        LFmotor.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        LBmotor.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);

        sliderSpool.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        turnTable.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);

        /*
        sliderSpool.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        intakeMotor.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        turnTable.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);

         */
        // Set rb behavior when power is zero (BRAKE = brake, FLOAT = no brake)
        RFmotor.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        LFmotor.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        RBmotor.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        LBmotor.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);

        //.o; Set all motors to zero power for initialization

        RFmotor.setPower(0);
        LFmotor.setPower(0);
        RBmotor.setPower(0);
        LBmotor.setPower(0);


    }//End init code

    //Driving Functions

    /**
     * Stops all motors and applies zero power behavior RFmotorom config (either BRAKE or FLOAT)
     */
    public void driveStop() {
        drive(0);
//        RFmotor.setPower(0);
//        LFmotor.setPower(0);
//        RBmotor.setPower(0);
//        LBmotor.setPower(0);
    }

    /**
     * Sets all wheels to same drive speed.
     *
     * @param speed -1 to 1
     */
    public void drive(double speed) {
        RFmotor.setPower(speed);
        LFmotor.setPower(speed);
        RBmotor.setPower(speed);
        LBmotor.setPower(speed);
    }

    /**
     * Sets wheels on left and right sides of robot to different speeds
     *
     * @param leftPower  Left Motors Speed -1 to 1
     * @param rightPower Right Motors Speed -1 to 1
     */
    public void drive(double leftPower, double rightPower) {
        RFmotor.setPower(rightPower);
        LFmotor.setPower(leftPower);
        RBmotor.setPower(rightPower);
        LBmotor.setPower(leftPower);
    }

    /**
     * Sets each drive motor to individual speeds, used by main mecanum driving function in simpleBotTeleOp.java
     *
     * @param frontrightPower Speed -1 to 1
     * @param frontleftPower  Speed -1 to 1
     * @param backrightPower  Speed -1 to 1
     * @param backleftPower   Speed -1 to 1
     */
    public void drivefour(double frontrightPower, double frontleftPower, double backrightPower, double backleftPower) {
        RFmotor.setPower(frontrightPower);
        LFmotor.setPower(frontleftPower);
        RBmotor.setPower(backrightPower);
        LBmotor.setPower(backleftPower);
    }

    /**
     * Strafes Mecanum Drivetrain
     *
     * @param speed single speed for all motors for strafing
     */
    public void strafe(double speed) {
        RFmotor.setPower(speed);
        LFmotor.setPower(-speed);
        RBmotor.setPower(-speed);
        LBmotor.setPower(speed);
    }

    /**
     * @param frontMotors Higher values = more right rotation
     * @param rearMotors  Higher values = more left rotation
     */
    public void strafe(double frontMotors, double rearMotors) {
        RFmotor.setPower(frontMotors);
        LFmotor.setPower(-frontMotors);
        RBmotor.setPower(-rearMotors);
        LBmotor.setPower(rearMotors);
    }

    /**
     * Rotates robot using equal speeds for all drive motors
     *
     * @param speed + values are left and - values are right
     */
    public void turn(double speed) {
        RFmotor.setPower(-speed);
        LFmotor.setPower(speed);
        RBmotor.setPower(-speed);
        LBmotor.setPower(speed);
    }



    /**
     * Drives robot southwest assuming North is tower side of field
     *
     * @param FLspeed speed for front left motor
     * @param BRspeed speed for back right motor
     */
    //This would be NorthEast if front of robot is intake side
    //these notes here above may not be correct?
    public void driveSouthWestAuto(double FLspeed, double BRspeed) {
        LFmotor.setPower(FLspeed);
        RBmotor.setPower(BRspeed);
    }
    public void driveNorthWestAuto(double FLspeed, double BRspeed) {
        RFmotor.setPower(FLspeed);
        LBmotor.setPower(BRspeed);
    }
    //IMU Functions:

    /**
     * Rotate left or right the number of degrees. Does not support turning more than 180 degrees.
     *
     * @param degrees Degrees to turn, + is left - is right
     * @param power   Motor power during turn (should not be negative)
     */
    public void rotate(double degrees, double power) throws InterruptedException {

        // restart imu movement tracking.
        resetAngle();

        // getAngle() returns + when rotating counter clockwise (left) and - when rotating
        // clockwise (right).

        if (degrees < 0) {   // turn right.
            power = -power;

        } else if (degrees > 0) {   // turn left.
            //return; //power = power; dont need to change anything because assuming the power is already positive, that should make the robot turn right when put into the turn function
        } else return;

        // set power to rotate.
        turn(power);

        // rotate until turn is completed.
        if (degrees < 0) {
            // On right turn we have to get off zero first.
            while (opMode.opModeIsActive() && getAngle() == 0) {
            }

            while (opMode.opModeIsActive() && getAngle() > degrees) {
            }
        } else {  // left turn.
            System.out.println("Starting Left Turn..?  Starting Angle is: " + getAngle());
            while (opMode.opModeIsActive() && getAngle() < degrees) {
                System.out.println("Turning to angle... Angle right now is:" + getAngle());
            }
        }
        // turn the motors off.
        driveStop();
        System.out.println("Done rotating");

        // wait for rotation to stop.
        Thread.sleep(100);

        // reset angle tracking on new heading.
        resetAngle();
    }

    /**
     * Rotates to a global heading using IMU
     *
     * @param degrees how far to turn?
     * @param power the power at which to turn?
     * @throws InterruptedException
     */
    public void rotateToGlobalAngle(int degrees, double power) throws InterruptedException {

        // restart imu movement tracking.
        Orientation currentAngle = imu.getAngularOrientation(AxesReference.INTRINSIC, AxesOrder.ZYX, AngleUnit.DEGREES);

        globalAngle = currentAngle.firstAngle;

        // getAngle() returns + when rotating counter clockwise (left) and - when rotating
        // clockwise (right).

        if (degrees < 0) {   // turn right.
            power = -power;

        } else if (degrees > 0) {   // turn left.
            return;//power = power; dont need to change anything because assuming the power is already positive, that should make the robot turn right when put into the turn function
        } else return;

        // set power to rotate.
        turn(power);

        // rotate until turn is completed.
        if (degrees < 0) {
            // On right turn we have to get off zero first.
            while (opMode.opModeIsActive() && getAngle() == 0) {
            }

            while (opMode.opModeIsActive() && getAngle() > degrees) {
            }
        } else    // left turn.
            while (opMode.opModeIsActive() && getAngle() < degrees) {
            }

        // turn the motors off.
        driveStop();

        // wait for rotation to stop.
        Thread.sleep(100);

        // reset angle tracking on new heading.
        resetAngle();
    }

    /**
     * Resets the cumulative angle tracking to zero.
     */
    private void resetAngle() {
        lastAngles = imu.getAngularOrientation(AxesReference.INTRINSIC, AxesOrder.ZYX, AngleUnit.DEGREES);

        globalAngle = 0;
    }

    /**
     * Get current cumulative angle rotation RFmotorom last reset.
     *
     * @return Angle in degrees. + = left, - = right.
     */
    public double getAngle() {
        // We have to process the angle because the imu works in euler angles so the Z axis is
        // returned as 0 to +180 or 0 to -180 rolling back to -179 or +179 when rotation passes
        // 180 degrees. We detect this transition and track the total cumulative angle of rotation.

        Orientation angles = imu.getAngularOrientation(AxesReference.INTRINSIC, AxesOrder.ZYX, AngleUnit.DEGREES);

        double deltaAngle = angles.firstAngle - lastAngles.firstAngle;

        if (deltaAngle < -180)
            deltaAngle += 360;
        else if (deltaAngle > 180)
            deltaAngle -= 360;

        globalAngle += deltaAngle;

        lastAngles = angles;

        return globalAngle;
    }

    /**
     * Get current heading relative to intialization angle (i think)
     *
     * @return Current heading / angle RFmotorom -180 to 180
     */
    public double getGlobalAngle() {

        Orientation angles = imu.getAngularOrientation(AxesReference.INTRINSIC, AxesOrder.ZYX, AngleUnit.DEGREES);
        return angles.firstAngle;
    }

    //Encoder Drive Functions:

    /**
     * Drives forward using 1 encoder
     * IMPORTANT: this is replaced by driveForwardByEncoderAndIMU which uses the imu to maintain heading
     *
     * @param positionChange This should be positive or negative based on direction
     * @param motor          rb.FL
     * @param power          Always positive, direction controlled by positionChange
     */
    public void driveForwardByEncoder(int positionChange, DcMotor motor, double power) {
        power = Math.abs(power);

        int oldPosition = motor.getCurrentPosition();
        int targetPosition = oldPosition + positionChange;
        double currentPower = 0.2; //Always start at 0.2 power

        if (positionChange > 0) {
            while (opMode.opModeIsActive() && motor.getCurrentPosition() < targetPosition * .75) {

                if (currentPower >= power) {
                    currentPower = power;
                } else {
                    currentPower = currentPower + DEFAULT_ACCELERATION_INCREMENT;
                }
                drive(currentPower);
            }
            //deceleration code:

            while (opMode.opModeIsActive() && motor.getCurrentPosition() < targetPosition) {

                drive(Range.clip(Math.abs(motor.getCurrentPosition() - targetPosition) / motor.getCurrentPosition() + oldPosition, .1, 1));

            }

            driveStop();
        } else if (positionChange < 0) {
            drive(-power);
            while (opMode.opModeIsActive() && motor.getCurrentPosition() > targetPosition) {
                Thread.yield();
            }
            driveStop();
        }

    }

    /**
     * Drives forward using 1 encoders
     *
     * @param positionChange        This should be positive or negative based on direction
     * @param motor                 rb.FL
     * @param power                 Always positive, direction controlled by positionChange
     * @param correctionGain        Controls how much corrections is applied to the motors if the angle is off, default is .06 i think
     * @param accelerationIncrement Positive Linear increment that is used to increase acceleration, default is DEFAULT_ACCELERATION_INCREMENT. To get no acceleration, just make this 1
     */
    public void driveForwardByEncoderAndIMU(int positionChange, DcMotor motor, double power, double correctionGain, double accelerationIncrement) {
        DcMotor oppositeMotor;
        if (motor == LFmotor) {
            oppositeMotor = LBmotor;
        } else if (motor == RFmotor) {
            oppositeMotor = RBmotor;
        } else if (motor == RBmotor) {

        }
        power = Math.abs(power);

        positionChange = (int) (positionChange * GEAR_RATIO_MULTIPLIER); //Used for changing gear ratios without changing all values in code

        int oldPosition = motor.getCurrentPosition();
        int targetPosition = oldPosition + positionChange;
        if (positionChange < 0) { //Make acceleration increment positive or negative based on power
            accelerationIncrement = -accelerationIncrement;
        }

        if (positionChange > 0) {
            double currentPower = 0.2; //Always start at 0.2 power
            drive(currentPower);
            while (opMode.opModeIsActive() && motor.getCurrentPosition() < targetPosition * .8) {
                currentPower += accelerationIncrement;
                if (currentPower > power) {
                    currentPower = power; //if .2 + accelerationIncrement is bigger than the target power (over max power)
                }

                // Use IMU to drive in a straight line.
                correction = checkCorrection(correctionGain);
                drive((currentPower + correction), (currentPower - correction));
//                Thread.yield();
            }

            while (opMode.opModeIsActive() && motor.getCurrentPosition() < targetPosition) {
//                double decelerationSpeed = (Range.clip(Math.abs(motor.getCurrentPosition() - FLtargetPosition) / motor.getCurrentPosition() + FLoldPosition, .1, power));
//                System.out.println(decelerationSpeed);
                if (currentPower > .35) { //If we're driving slower than .35, i don't think we need to rly worry about decceleration
                    correction = checkCorrection(correctionGain);
                    drive((currentPower / 2 + correction), (currentPower / 2 - correction));
                } else {
                    correction = checkCorrection(correctionGain);
                    drive((currentPower + correction), (currentPower - correction));
                }
            }

            driveStop();

        } else if (positionChange < 0) {
            double currentPower = -0.2; //Always start at 0.2 power
            drive(currentPower);
            while (opMode.opModeIsActive() && motor.getCurrentPosition() > targetPosition * 1.2) {

                if (currentPower >= -power) {
                    currentPower = -power;
                } else {
                    currentPower -= DEFAULT_ACCELERATION_INCREMENT;

                }
                // Use IMU to drive in a straight line.
                correction = checkCorrection(correctionGain);
                drive((currentPower + correction), (currentPower - correction));
                Thread.yield();
            }

            while (opMode.opModeIsActive() && motor.getCurrentPosition() > targetPosition) {
                double decelerationSpeed = (Range.clip(Math.abs(motor.getCurrentPosition() - targetPosition) / motor.getCurrentPosition() + oldPosition, power, -.1));
                System.out.println(decelerationSpeed);
                correction = checkCorrection(correctionGain);
                drive((decelerationSpeed + correction), (decelerationSpeed - correction));

            }

            driveStop();
        }

    }

    /**
     * Drives forward with IMU to maintain heading to line on field
     *
     * @param power     NEGATIVE OR POSITIVE POWER CONTROLS DIRECTION
     * @param lineColor Linecolor to detect (white, blue)
     */
//    public void driveForwardByIMUtoLine(double power, String lineColor) {
//
//
//        if (lineColor.equals("white")) {
//            if (power > 0) {
//                drive(power);
//                while (opMode.opModeIsActive() && remoteAuto.groundColorSensor.alpha() < WHITE_ALPHA_THRESHOLD) {
//
//                    int currentAngle; //Stores heading at beginning of function
//                    // Use IMU to drive in a straight line.
//                    correction = checkCorrection(.1);
//                    drive((power + correction), (power - correction));
//                    Thread.yield();
//                }
//
//                driveStop();
//            } else if (power < 0) {
//                drive(-power);
//                while (opMode.opModeIsActive() && remoteAuto.groundColorSensor.alpha() < WHITE_ALPHA_THRESHOLD) {
//                    // Use IMU to drive in a straight line.
//                    correction = checkCorrection(.1);
//                    drive((power - correction), (power + correction));
//                    Thread.yield();
//                }
//                driveStop();
//            }
//        }
//
//
//    }


    /**
     * Checks how much of a correction to make
     *
     * @return Power adjustment, + is adjust left - is adjust right.
     */
    private double checkCorrection(Double correctionGain) {
        // gain value determines how sensitive the correction is to direction changes
        //default = .10
        double correction, angle, gain = correctionGain;

        angle = getAngle();

        if (angle == 0)
            correction = 0;             // no adjustment.
        else
            correction = -angle;        // reverse sign of angle for correction.

        correction = correction * gain;

        return correction;
    }

    /**
     * Drives southwest, assuming tower goals are North and robot shooter is facing towards tower goals.
     *
     * @param positionChange Positive or negative value controls direction, Negative Values move north east
     * @param motor          RB.FL
     * @param power          Positive value, 0-1
     * @param correctionGain not entirely sure?
     */
    public void autoDriveSouthWestWithEncoderAndIMU(int positionChange, DcMotor motor, double power, double correctionGain) {
        power = Math.abs(power);
        positionChange = (int) (positionChange * GEAR_RATIO_MULTIPLIER); //Used for changing gear ratios without changing all values in code

        int oldPosition = motor.getCurrentPosition();
        int targetPosition = oldPosition - positionChange;
        double currentPower = 0.2; //Always start at 0.2 power

        if (positionChange > 0) {
            while (opMode.opModeIsActive() && motor.getCurrentPosition() > targetPosition) {

                // Use IMU to drive in a straight line.
                correction = checkCorrection(correctionGain);
                driveSouthWestAuto(-(power - correction), -(power + correction));
                Thread.yield();
            }

            driveStop();
        } else if (positionChange < 0) {
            while (opMode.opModeIsActive() && motor.getCurrentPosition() > targetPosition) {
                // Use IMU to drive in a straight line.
                correction = checkCorrection(correctionGain);
                driveSouthWestAuto((power + correction), (power - correction));
                Thread.yield();
            }
            driveStop();
        }

    }
    public void autoDriveNorthWestWithEncoderAndIMU(int positionChange, DcMotor motor, double power, double correctionGain) {
        power = Math.abs(power);
        positionChange = (int) (positionChange * GEAR_RATIO_MULTIPLIER); //Used for changing gear ratios without changing all values in code

        int oldPosition = motor.getCurrentPosition();
        int targetPosition = oldPosition - positionChange;
        double currentPower = 0.2; //Always start at 0.2 power

        if (positionChange > 0) {
            while (opMode.opModeIsActive() && motor.getCurrentPosition() > targetPosition) {

                // Use IMU to drive in a straight line.
                correction = checkCorrection(correctionGain);
                driveNorthWestAuto(-(power - correction), -(power + correction));
                Thread.yield();
            }

            driveStop();
        } else if (positionChange < 0) {
            while (opMode.opModeIsActive() && motor.getCurrentPosition() > targetPosition) {
                // Use IMU to drive in a straight line.
                correction = checkCorrection(correctionGain);
                driveNorthWestAuto((power + correction), (power - correction));
                Thread.yield();
            }
            driveStop();
        }

    }
    /**
     * Drives southwest, assuming tower goals are North and robot shooter is facing towards tower goals.
     *
     * @param positionChange Positive or negative value controls direction, Negative Values move north east
     * @param motor          RB.FL
     * @param power          Positive value, 0-1
     * @param correctionGain again not entirely sure lmao
     */
    //We should incorporate this into the code now.
    public void autoDriveNorthEastWithEncoderAndIMU(int positionChange, DcMotor motor, double power, double correctionGain) {

        power = Math.abs(power);

        int oldPosition = motor.getCurrentPosition();
        int targetPosition = oldPosition - positionChange;

        if (positionChange > 0) {
            while (opMode.opModeIsActive() && motor.getCurrentPosition() > targetPosition) {

                // Use IMU to drive in a straight line.
                correction = checkCorrection(correctionGain);
                driveSouthWestAuto(-(power - correction), -(power + correction));
                Thread.yield();
            }

            driveStop();
        } else if (positionChange < 0) {
            while (opMode.opModeIsActive() && motor.getCurrentPosition() > targetPosition) {
                // Use IMU to drive in a straight line.
                correction = checkCorrection(correctionGain);
                driveSouthWestAuto((power - correction), (power + correction));
                Thread.yield();
            }
            driveStop();
        }

    }


    public void strafeRightByEncoder(int positionChange, DcMotor motor, double power) {
        power = Math.abs(power);
        int oldPosition = motor.getCurrentPosition();
        int targetPosition = oldPosition + positionChange;

        if (positionChange > 0) {
            strafe(power);
            while (opMode.opModeIsActive() && motor.getCurrentPosition() < targetPosition) {
                Thread.yield();
            }
            driveStop();
        } else if (positionChange < 0) {
            strafe(-power);
            while (opMode.opModeIsActive() && motor.getCurrentPosition() > targetPosition) {
                Thread.yield();
            }
            driveStop();
        }

    }

    /**
     * Strafes to the right using IMU to maintain angle and encoders for position
     *
     * @param positionChange Use negative to strafe to the left
     * @param motor          rb.FL for example
     * @param power          0 - 1, positive values only
     */

    public void strafeRightByEncoderAndIMU(int positionChange, DcMotor motor, double power, double correctionGain) {

        power = Math.abs(power);
        positionChange = (int) (positionChange * GEAR_RATIO_MULTIPLIER); //Used for changing gear ratios without changing all values in code

        int oldPosition = motor.getCurrentPosition();
        int targetPosition = oldPosition - positionChange;

        //Current pos = 500
        //pos change = 200
        //target = 700
        //new target = 300

        if (positionChange > 0) {
            double currentPower = 0.2; //Always start at 0.2 power
            strafe(currentPower);
            while (opMode.opModeIsActive() && motor.getCurrentPosition() > targetPosition) {

                if (currentPower >= power) {
                    currentPower = power;
                } else {
                    currentPower += DEFAULT_ACCELERATION_INCREMENT;

                }

                // Use IMU to drive in a straight line.
                correction = checkCorrection(correctionGain);
                strafe((currentPower - correction), (currentPower + correction));
                Thread.yield();
            }
            driveStop();
            System.out.println("Ok, done running.");


        } else if (positionChange < 0) {
            double currentPower = -0.2; //Always start at 0.2 power
            strafe(currentPower);
            while (opMode.opModeIsActive() && motor.getCurrentPosition() < targetPosition) {

                if (currentPower >= -power) {
                    currentPower = -power;
                } else {
                    currentPower -= DEFAULT_ACCELERATION_INCREMENT;

                }

                // Use IMU to drive in a straight line.
                correction = checkCorrection(correctionGain);
                strafe((currentPower - correction), (currentPower + correction));

                Thread.yield();
            }
            driveStop();
        }

        driveStop();

    }


    public void turnClockwiseByEncoder(int positionChange, DcMotor motor, double power) {
        power = Math.abs(power);
        int oldPosition = motor.getCurrentPosition();
        int targetPosition = oldPosition + positionChange;

        if (positionChange > 0) {
            turn(power);
            while (opMode.opModeIsActive() && motor.getCurrentPosition() < targetPosition) {
                Thread.yield();
            }
            driveStop();
        } else if (positionChange < 0) {
            turn(-power);
            while (opMode.opModeIsActive() && motor.getCurrentPosition() > targetPosition) {
                Thread.yield();
            }
            driveStop();
        }
    }

    public void driveWithLeftMore(int positionChange, DcMotor motor, double power) {
        power = Math.abs(power);
        int oldPosition = motor.getCurrentPosition();
        int targetPosition = oldPosition + positionChange;

        if (positionChange > 0) {
            RFmotor.setPower(power);
            LFmotor.setPower(power);
            RBmotor.setPower(power);
            LBmotor.setPower(power);
            while (opMode.opModeIsActive() && motor.getCurrentPosition() < targetPosition) {
                Thread.yield();
            }

            driveStop();
        } else if (positionChange < 0) {
            RFmotor.setPower(-power * .66);
            LFmotor.setPower(-power);
            RBmotor.setPower(-power * .66);
            LBmotor.setPower(-power);
            while (opMode.opModeIsActive() && motor.getCurrentPosition() > targetPosition) {
                Thread.yield();
            }
            driveStop();
        }

    }

    public void driveWithRightMore(int positionChange, DcMotor motor, double power) {
        power = Math.abs(power);
        int oldPosition = motor.getCurrentPosition();
        int targetPosition = oldPosition + positionChange;

        if (positionChange > 0) {
            RFmotor.setPower(power);
            LFmotor.setPower(power);
            RBmotor.setPower(power);
            LBmotor.setPower(power);
            while (opMode.opModeIsActive() && motor.getCurrentPosition() < targetPosition) {
                Thread.yield();
            }

            driveStop();
        } else if (positionChange < 0) {
            RFmotor.setPower(-power);
            LFmotor.setPower(-power * .66);
            RBmotor.setPower(-power);
            LBmotor.setPower(-power * .66);
            while (opMode.opModeIsActive() && motor.getCurrentPosition() > targetPosition) {
                Thread.yield();
            }
            driveStop();
        }

    }


    /**
     * Moves Lift forward using 1 encoder
     *
     * @param position       This should be positive or 0?
     * @param motor          rb.sliderSpool
     *
     */
    public void LifterByEncoder(int position, DcMotor motor) {
        double power = .8;
        motor.setTargetPosition(position);
        motor.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        motor.setPower(power);
//        double power = 0;
//        int oldPosition = motor.getCurrentPosition();
//        int targetPosition = position;
//        double currentPower = 0.2; //Always start at 0.2 power
//
//        if (targetPosition < oldPosition) {
//            power = slidePowerUp;
//            while (opMode.opModeIsActive() && motor.getCurrentPosition() < targetPosition * .75) {
//
//                if (currentPower >= power) {
//                    currentPower = power;
//                } else {
//                    currentPower = currentPower + DEFAULT_ACCELERATION_INCREMENT;
//                }
//                sliderSpool.setPower(currentPower);
//            }
//            //deceleration code:
//
//            while (opMode.opModeIsActive() && motor.getCurrentPosition() < targetPosition) {
//
//                drive(Range.clip(Math.abs(motor.getCurrentPosition() - targetPosition) / motor.getCurrentPosition() + oldPosition, .1, 1));
//
//            }
//
//            sliderSpool.setPower(0);
//        } else if (targetPosition > oldPosition) {
//            power = slidePowerDown;
//            sliderSpool.setPower(-power);
//            while (opMode.opModeIsActive() && motor.getCurrentPosition() > targetPosition) {
//                Thread.yield();
//            }
//            sliderSpool.setPower(0);
//        }

    }
}


