package org.firstinspires.ftc.teamcode.PierreTeleOp;


import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.robotcore.external.Func;
import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.robotcore.external.navigation.AxesOrder;
import org.firstinspires.ftc.robotcore.external.navigation.AxesReference;
import org.firstinspires.ftc.robotcore.external.navigation.Orientation;

import java.util.Locale;

import static org.firstinspires.ftc.teamcode.Constants.DRIVE_STICK_THRESHOLD;
import static org.firstinspires.ftc.teamcode.Constants.TRIGGER_THRESHOLD;
import static org.firstinspires.ftc.teamcode.Constants.blueturnTablePower;


@TeleOp(name = "!Blue QM TeleOP", group = "!Primary")
public class BlueTeleOp extends LinearOpMode {

    //private final FtcDashboard dashboard = FtcDashboard.getInstance(); //Comment this out when not using dashboard

    private final HardwareRobot rb = new HardwareRobot();
    private final ElapsedTime runtime = new ElapsedTime();
    /*DcMotor RFmotor;
    DcMotor RBmotor;
    DcMotor LFmotor;
    DcMotor LBmotor;
    DcMotor turnTable;
    DcMotor sliderSpool;
    DcMotor intakeMotor;*/
    private double slowModeMultiplier = 1;
    Orientation angles;

    @Override
    public void runOpMode() throws InterruptedException {
        telemetry.setAutoClear(false);
        //telemetry = new MultipleTelemetry(telemetry, dashboard.getTelemetry());
        telemetry.addData("Status", "Initializing");
        telemetry.update();

        telemetry.addData("Status", "Initializing Hardware...");
        telemetry.update();
        rb.init(hardwareMap, this); //runs init stuff in HardwareSimpleBot.java
        //rb.lifterMotor.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        telemetry.addData("Status", "Hardware Map Initialized");
        telemetry.update();

        telemetry.addData("Status", "Calibrating IMU...");
        telemetry.update();
        // make sure the imu gyro is calibrated before continuing.
        //IMPORTANT: The gyro will not calibrate unless the robot is not moving, make sure the robot is still during initialization.
        /*while (!isStopRequested() && !rb.imu.isGyroCalibrated()) {
            sleep(50);
            idle();
        }*/

        telemetry.addData("imu calib status: ", rb.imu.getCalibrationStatus().toString());

        composeTelemetry();

        telemetry.addData("Status", "Initialized, Ready to Start. Make sure lifter is in back position");
        telemetry.update();
        // Wait for the game to start (driver presses PLAY)
        //rb.lifterMotor.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        //Set Servo Start positions
        /*
        rb.wobbleServo.setPosition(WOBBLE_ARMED);
        rb.leftBlocker.setPosition(BLOCKER_LEFT_START);
        rb.rightBlocker.setPosition(BLOCKER_RIGHT_START);
         */

        waitForStart(); //Everything up to here is initialization
        runtime.reset();
        telemetry.setAutoClear(true);

        //rb.runIntake(true, false); //Start with intake running TODO: Turn this on for real comp
        // run this until the end of the match (driver presses STOP)
        while (opModeIsActive()) {

            drive(); //Drive robot with sticks
            //telemetry.addData("Status:", "drive ok");
            //telemetry.update();
            turntable(); //
            //telemetry.addData("Status:", "turntable ok");
            //telemetry.update();
            slider();
            //telemetry.addData("Status:", "slider ok");
            //telemetry.update();
            intake();
            //telemetry.addData("Status:", "intake ok");
            //telemetry.update();
            telemetry.addData("FR Encoder", rb.RFmotor.getCurrentPosition());
            telemetry.addData("FL Encoder", rb.LFmotor.getCurrentPosition());
            telemetry.addData("BR Encoder", rb.RBmotor.getCurrentPosition());
            telemetry.addData("BL Encoder", rb.LBmotor.getCurrentPosition());
            telemetry.addData("Slider Spool Encoder", rb.sliderSpool.getCurrentPosition());
            telemetry.addData("trigger value", gamepad2.left_trigger);
            telemetry.addData("left stick value", gamepad2.left_stick_y);
            telemetry.update();
        }
    }
    private void drive() {
        /*
        double funnyNumber = 69420.69420; //Boone's first line of robotics code!!!!
        if (funnyNumber == 69420.69420){
            System.out.println("FUNNY NUMBER ACHIEVED, UNIVERSE SAVED. ALL HAIL KING CALEB");
            String calebGod = "Boone is the official founder of Niehism";
            System.out.println(calebGod);

        }
        else if(funnyNumber != 69420.69420){
            for(int i = 0;i<69420;i++) {
                System.out.println("THAT IS AN UNFUNNY NUMBER, NOW IT'S" + i);
                System.out.println("Fake Funny Number!");
            }

        }
        */


        //Init variables
//hi caleb :)



//
//Drive Controls:
// Left stick= Translational Movement
// Right Stick= Rotational Movement
        //DRIVE_STICK_THRESHOLD = deadzone


        double frontLeftPower;
        double frontRightPower;
        double rearLeftPower;
        double rearRightPower;

        //double MAX_SPEED = 1.0;

        double leftY = gamepad1.left_stick_y;
        double leftX = gamepad1.left_stick_x;
        double rightX = gamepad1.right_stick_x;

//        pattern = RevBlinkinLedDriver.BlinkinPattern.ORANGE;
//        blinkinLedDriver.setPattern(pattern);
        //DRIVE_STICK_THRESHOLD = deadzone
        if (rightX < -DRIVE_STICK_THRESHOLD || rightX > DRIVE_STICK_THRESHOLD || leftY < -DRIVE_STICK_THRESHOLD || leftY > DRIVE_STICK_THRESHOLD || leftX < -DRIVE_STICK_THRESHOLD || leftX > DRIVE_STICK_THRESHOLD) {
            //Get stick values and apply modifiers:

            /*
            double drive = (-gamepad1.left_stick_y * 1.10) * slowModeMultiplier;
            double strafe = (gamepad1.left_stick_x) * slowModeMultiplier;
            double turn = (gamepad1.right_stick_x * 1.25) * slowModeMultiplier;
            */
            double drive = -gamepad1.left_stick_y * slowModeMultiplier;
            double strafe = -(gamepad1.left_stick_x) * slowModeMultiplier;
            double turn = (gamepad1.right_stick_x) * slowModeMultiplier;

            //Calculate each individual motor speed using the stick values:
            //range.clip calculates a value between min and max, change those values to reduce overall speed
            /*
            frontLeftPower = Range.clip(drive + strafe + turn, -1.0, 1.0);
            frontRightPower = Range.clip(drive - strafe - turn, -1.0, 1.0);
            rearLeftPower = Range.clip(drive - strafe + turn, -1.0, 1.0);
            rearRightPower = Range.clip(drive + strafe - turn, -1.0, 1.0);
             */
            frontLeftPower = drive + strafe + turn;
            frontRightPower = drive - strafe - turn;
            rearLeftPower = drive - strafe + turn;
            rearRightPower = drive + strafe - turn;
            rb.drivefour(frontRightPower, frontLeftPower, rearRightPower, rearLeftPower);
            //rb.drive(frontRightPower, 1, rearRightPower, rearLeftPower);

            /*
            frontLeftPower = Range.clip(drive + turn + strafe, -1.0, 1.0);
            frontRightPower = Range.clip(drive - turn - strafe, -1.0, 1.0);
            rearLeftPower = Range.clip(drive - turn + strafe, -1.0, 1.0);
            rearRightPower = Range.clip(drive + turn - strafe, -1.0, 1.0);
            rb.drive(frontRightPower, frontLeftPower, rearRightPower, rearLeftPower);
            */
            /*
            RBmotor.setPower(rearRightPower);
            RFmotor.setPower(frontRightPower);
            LBmotor.setPower(rearLeftPower);
            LFmotor.setPower(frontLeftPower);

            telemetry.addData("Front-right motor", "%5.2f", frontRightPower);
            telemetry.addData("Back-right motor", "%5.2f", rearRightPower);
            telemetry.addData("Front-left motor", "%5.2f", frontLeftPower);
            telemetry.addData("Back-left motor", "%5.2f", rearLeftPower);
            telemetry.update();
             */
        }
        else {
            rb.driveStop();
            /*
            RBmotor.setPower(0);
            RFmotor.setPower(0);
            LBmotor.setPower(0);
            LFmotor.setPower(0); */
            //Stop robot if no stick value (delete this if u want to drift lol)
        }
        if (gamepad1.left_trigger > TRIGGER_THRESHOLD) {
            slowModeMultiplier = .25;
        } else {
            slowModeMultiplier = 1;
        }
        /*
       if (Speed < -DRIVE_STICK_THRESHOLD || Speed > DRIVE_STICK_THRESHOLD || Turn < -DRIVE_STICK_THRESHOLD || Turn > DRIVE_STICK_THRESHOLD || Strafe < -DRIVE_STICK_THRESHOLD || Strafe > DRIVE_STICK_THRESHOLD){
            rearRightPower = Range.clip(Speed - Turn - Strafe, -1, 1);
            rearLeftPower= Range.clip(Speed + Turn - Strafe,-1, 1);
            frontRightPower= Range.clip(Speed + Turn - Strafe, -1, 1);
            frontLeftPower = Range.clip(Speed + Turn + Strafe, -1, 1);

           telemetry.addData("Front-right motor", "%5.2f", frontRightPower);
           telemetry.addData("Back-right motor", "%5.2f", rearRightPower);
           telemetry.addData("Front-left motor", "%5.2f", frontLeftPower);
           telemetry.addData("Back-left motor", "%5.2f", rearLeftPower);
           telemetry.update();

            RBmotor.setPower(rearRightPower);
            RFmotor.setPower(frontRightPower);
            LBmotor.setPower(rearLeftPower);
            LFmotor.setPower(frontLeftPower);


       }
       else {
           RBmotor.setPower(0);
           RFmotor.setPower(0);
           LBmotor.setPower(0);
           LFmotor.setPower(0);
       }*/
    }
    private void turntable(){ //throws InterruptedException {
        //turntable
        if(gamepad2.a){
            rb.turnTable.setPower(blueturnTablePower);
            telemetry.addData("A button","Pressed");
            telemetry.update();
        } else {
            rb.turnTable.setPower(0);
            telemetry.addData("A button", "Not Pressed");
            telemetry.update();
        }
    }

    private void slider() throws InterruptedException{// throws InterruptedException {
        //Slider
        //going down won't ever be smooth because the motor stalls

        if (gamepad2.left_trigger > 0.05f && rb.sliderSpool.getCurrentPosition() < -30)
            rb.sliderSpool.setPower(.75);
        else if (gamepad2.left_bumper && rb.sliderSpool.getCurrentPosition() > -1840)
            rb.sliderSpool.setPower(-.75);
        else
            rb.sliderSpool.setPower(0);
        /*if (gamepad2.dpad_down){
            rb.sliderSpool.setPower(.75);
            rb.sliderSpool.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        }
        else if (gamepad2.dpad_up){
            rb.sliderSpool.setPower(-.75);
        }*/
    }

    private void intake(){ //throws InterruptedException{
        //Intake
        if (gamepad2.right_trigger > 0.1f)
            rb.intakeMotor.setPower(1);
        else if (gamepad2.right_bumper)
            rb.intakeMotor.setPower(-1);
        else{
            rb.intakeMotor.setPower(0);
        }
    }


    /**
     * Logs IMU data to telemetry
     */
    void composeTelemetry() throws InterruptedException {

        // At the beginning of each telemetry update, grab a bunch of data
        // from the IMU that we will then display in separate lines.
        telemetry.addAction(new Runnable() {
            @Override
            public void run() {
                // Acquiring the angles is relatively expensive; we don't want
                // to do that in each of the three items that need that info, as that's
                // three times the necessary expense.
                angles = rb.imu.getAngularOrientation(AxesReference.INTRINSIC, AxesOrder.ZYX, AngleUnit.DEGREES);
//               gravity = rb.imu.getGravity(); dont need gravity?
            }
        });

        telemetry.addLine()
                .addData("status", new Func<String>() {
                    @Override
                    public String value() {
                        return rb.imu.getSystemStatus().toShortString();
                    }
                })
                .addData("calib", new Func<String>() {
                    @Override
                    public String value() {
                        return rb.imu.getCalibrationStatus().toString();
                    }
                });

        telemetry.addLine()
                .addData("heading", new Func<String>() {
                    @Override
                    public String value() {
                        return formatAngle(angles.angleUnit, angles.firstAngle);
                    }
                })
                .addData("roll", new Func<String>() {
                    @Override
                    public String value() {
                        return formatAngle(angles.angleUnit, angles.secondAngle);
                    }
                })
                .addData("pitch", new Func<String>() {
                    @Override
                    public String value() {
                        return formatAngle(angles.angleUnit, angles.thirdAngle);
                    }
                });

        Thread.sleep(3000); //throttle display
    }
    //----------------------------------------------------------------------------------------------
    // Formatting
    //----------------------------------------------------------------------------------------------

    String formatAngle(AngleUnit angleUnit, double angle) {
        return formatDegrees(AngleUnit.DEGREES.fromUnit(angleUnit, angle));
    }

    String formatDegrees(double degrees) {
        return String.format(Locale.getDefault(), "%.1f", AngleUnit.DEGREES.normalize(degrees));
    }

    private void delay(double delayTime) {

        double startTime = getRuntime();
        while ((getRuntime() < startTime + delayTime) && opModeIsActive()) {
            //wait for delay
        }
    }

}



















