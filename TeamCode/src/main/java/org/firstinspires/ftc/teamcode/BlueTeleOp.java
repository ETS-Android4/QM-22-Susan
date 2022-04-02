package org.firstinspires.ftc.teamcode;


import static org.firstinspires.ftc.teamcode.Constants.ATURRET_INCREMENT;
import static org.firstinspires.ftc.teamcode.Constants.BTURRET_INCREMENT;
import static org.firstinspires.ftc.teamcode.Constants.DRIVE_STICK_THRESHOLD;
import static org.firstinspires.ftc.teamcode.Constants.DRIVE_STICK_THRESHOLD2;
import static org.firstinspires.ftc.teamcode.Constants.TRIGGER_THRESHOLD;
import static org.firstinspires.ftc.teamcode.Constants.TURRET_CYCLE_MS;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.util.ElapsedTime;
import com.qualcomm.robotcore.util.Range;

import org.firstinspires.ftc.robotcore.external.Func;
import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.robotcore.external.navigation.AxesOrder;
import org.firstinspires.ftc.robotcore.external.navigation.AxesReference;
import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit;
import org.firstinspires.ftc.robotcore.external.navigation.Orientation;

import java.util.Locale;


@TeleOp(name = "!!BlueSusanTeleOp", group = "!Primary")
public class BlueTeleOp extends LinearOpMode {

    //private final FtcDashboard dashboard = FtcDashboard.getInstance(); //Comment this out when not using dashboard

    private final NewHardware rb = new NewHardware();
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
    double tbottom = .5;
    double tlifter = .45;
    private boolean intakefunctionon = true;
    private boolean cargoin = false;
    private int turret_sleep = TURRET_CYCLE_MS;
    private int slidePos = 0;
    int runSpinner;
    double spin;
    public double startTime = runtime.seconds();

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
        rb.sliderSpool.setTargetPosition(0);
        rb.sliderSpool.setMode(DcMotor.RunMode.RUN_TO_POSITION);

        waitForStart(); //Everything up to here is initialization
        runtime.reset();
        telemetry.setAutoClear(true);

        //rb.runIntake(true, false); //Start with intake running TODO: Turn this on for real comp
        // run this until the end of the match (driver presses STOP)
        while (opModeIsActive()) {
            extrafunctions();
            presetPositions();
            drive(); //Drive robot with sticks
            turntable(); //
            //telemetry.addData("Status:", "turntable ok");
            //telemetry.update();
            slider();
            //telemetry.addData("Status:", "slider ok");
            //telemetry.update();
            intake();
            //telemetry.addData("Status:", "intake ok");
            //telemetry.update();
            turret();

            telemetry.addData("FR Encoder", rb.RFmotor.getCurrentPosition());
            telemetry.addData("FL Encoder", rb.LFmotor.getCurrentPosition());
            telemetry.addData("BR Encoder", rb.RBmotor.getCurrentPosition());
            telemetry.addData("BL Encoder", rb.LBmotor.getCurrentPosition());
//            telemetry.addData("Distance Sensor", rb.intakeSensor.getDistance(DistanceUnit.CM));
            telemetry.addData("Slider Spool Encoder", rb.sliderSpool.getCurrentPosition());

            /*
            telemetry.addData("trigger value", gamepad2.left_trigger);
            telemetry.addData("left stick value", gamepad2.left_stick_y);
            */
            telemetry.update();
        }
    }
    private void extrafunctions(){
        /*if (gamepad2.dpad_up){
            encoderfunctionoff = false;
        }
        else if (gamepad2.dpad_down){
            encoderfunctionoff = true;
        }*/
        if(gamepad1.right_bumper){
            intakefunctionon = true;
        }
        else{
            intakefunctionon = false;
        }
        //TODO: fix these value here (distance sensor)
//        if (rb.intakeSensor.getDistance(DistanceUnit.CM) < 11.7)
//            cargoin = true;
//        else
            cargoin = false;

    }
    private void drive() {

        //Init variables

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
        if (gamepad1.left_trigger > TRIGGER_THRESHOLD) {
            slowModeMultiplier = .25;
        }
        else if (gamepad1.right_trigger > TRIGGER_THRESHOLD) {
            slowModeMultiplier = 1;
        }
        else {
            slowModeMultiplier = .7;
        }
        if (rightX < -DRIVE_STICK_THRESHOLD || rightX > DRIVE_STICK_THRESHOLD || leftY < -DRIVE_STICK_THRESHOLD || leftY > DRIVE_STICK_THRESHOLD || leftX < -DRIVE_STICK_THRESHOLD || leftX > DRIVE_STICK_THRESHOLD) {
            //Get stick values and apply modifiers:

            /*
            double drive = (-gamepad1.left_stick_y * 1.10) * slowModeMultiplier;
            double strafe = (gamepad1.left_stick_x) * slowModeMultiplier;
            double turn = (gamepad1.right_stick_x * 1.25) * slowModeMultiplier;
            */
            double drive = -gamepad1.left_stick_y * slowModeMultiplier;
            double strafe = (gamepad1.left_stick_x) * slowModeMultiplier;
            double turn = (gamepad1.right_stick_x) * slowModeMultiplier;

            //Calculate each individual motor speed using the stick values:
            //range.clip calculates a value between min and max, change those values to reduce overall speed
            frontLeftPower = drive + strafe + turn;
            frontRightPower = drive - strafe - turn;
            rearLeftPower = drive - strafe + turn;
            rearRightPower = drive + strafe - turn;
            rb.drivefour(frontRightPower, frontLeftPower, rearRightPower, rearLeftPower);
        }
        else {
            rb.driveStop();
            /*
            RBmotor.setPower(0);
            RFmotor.setPower(0);
            LBmotor.setPower(0);
            LFmotor.setPower(0);
            */
            //Stop robot if no stick value (delete this if u want to drift lol)
        }
    }
    private void turntable() { //throws InterruptedException {
        //Using the spinner
        if (gamepad1.a) {
            runSpinner = 1;
            runtime.reset();
            resetStartTime();
        }
        if (gamepad1.b) {
            runSpinner = 2;
            runtime.reset();
            resetStartTime();
        }
        if (runSpinner == 1 && startTime < 1.2) {
            spin = 0.18/4 + (startTime * 0.18/4);
            //0.2,0.22
            telemetry.addData("spinner power", spin);
            telemetry.addData("is it working", 2);
        } else if (runSpinner == 1 && startTime > 1.2 && startTime < 1.4) {
            spin = 1/4;
            telemetry.addData("spinner power", spin);
        } else if (startTime > 1.4) {
            spin = 0;
            telemetry.addData("done:)", 1);
            runSpinner = 0;

        }
        if (runSpinner == 2 && startTime < 1.2) {
            spin = -(0.18/4 + (startTime * 0.18/4));
            telemetry.addData("spinner power", spin);
            telemetry.addData("is it working", 2);
        } else if (runSpinner == 2 && startTime > 1.2 && startTime < 1.4) {
            spin = -1/4;
            telemetry.addData("spinner power", spin);
        } else if (startTime > 1.4) {
            spin = 0;
            telemetry.addData("done:)", 1);
            runSpinner = 0;

        }
//        if (gamepad1.a) {
//            runSpinner = 1;
//            runtime.reset();
//            resetStartTime();
//        }
//        if (gamepad1.b) {
//            runSpinner = 2;
//            runtime.reset();
//            resetStartTime();
//        }
        if (runSpinner == 1 && startTime < 1.2) {
            spin = 0.18/4 + (startTime * 0.18/4);
            //0.2,0.22
            telemetry.addData("spinner power", spin);
            telemetry.addData("is it working", 2);
        } else if (runSpinner == 1 && startTime > 1.2 && startTime < 1.4) {
            spin = 1/4;
            telemetry.addData("spinner power", spin);
        } else if (startTime > 1.4) {
            spin = 0;
            telemetry.addData("done:)", 1);
            runSpinner = 0;

        }
        if (runSpinner == 2 && startTime < 1.2) {
            spin = -(0.18/4 + (startTime * 0.18/4));
            telemetry.addData("spinner power", spin);
            telemetry.addData("is it working", 2);
        } else if (runSpinner == 2 && startTime > 1.2 && startTime < 1.4) {
            spin = -1/4;
            telemetry.addData("spinner power", spin);
        } else if (startTime > 1.4) {
            spin = 0;
            telemetry.addData("done:)", 1);
            runSpinner = 0;
        }
    }
    private void presetPositions() throws InterruptedException {
        rb.sliderSpool.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        if(gamepad2.a){
            slidePos = 0;
            tlifter = .45;
            tbottom = .5;
        }
        if(gamepad2.b){
            tlifter = .53;
            tbottom = .54;
        }
    }
    private void slider() throws InterruptedException{
        //Slider
        /*
        if (!encoderfunctionoff){
            if (gamepad2.left_trigger > 0.05f)
                rb.sliderSpool.setPower(.75);
            else if (gamepad2.left_bumper)
                rb.sliderSpool.setPower(-.75);
            else
                rb.sliderSpool.setPower(0);
        }
        else {
            if (gamepad2.left_trigger > 0.05f && rb.sliderSpool.getCurrentPosition() < -30)
                rb.sliderSpool.setPower(.75);
            else if (gamepad2.left_bumper && rb.sliderSpool.getCurrentPosition() > -1840)
                rb.sliderSpool.setPower(-.75);
            else
                rb.sliderSpool.setPower(0);
        }
        */
        //TODO: Check these to make sure the gamepad is correct
        /*
        if (gamepad2.left_trigger > TRIGGER_THRESHOLD)
            rb.sliderSpool.setPower(.75);
        else if (gamepad2.left_bumper)
            rb.sliderSpool.setPower(-.75);
        else
            rb.sliderSpool.setPower(0);
        */
        //Limiting slide positions

        rb.sliderSpool.setTargetPosition(slidePos);
        rb.sliderSpool.setPower(1);

            if (gamepad2.dpad_up)
                slidePos = Range.clip(slidePos + 300, 0, 3200);
            else if (gamepad2.dpad_down)
                slidePos = Range.clip(slidePos - 300, 0, 3200);


        /*
        if ((gamepad2.left_trigger > 0.05f && rb.sliderSpool.getCurrentPosition() < -30))
            rb.sliderSpool.setPower(.75);
        else if (gamepad2.left_bumper && rb.sliderSpool.getCurrentPosition() > -1840)
            rb.sliderSpool.setPower(-.75);
        else if (gamepad2.dpad_down)
            rb.sliderSpool.setPower(.75);
        else if (gamepad2.dpad_up)
            rb.sliderSpool.setPower(-.75);
        else
            rb.sliderSpool.setPower(0);
        if (gamepad2.x){
            rb.sliderSpool.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
            rb.sliderSpool.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        }*/
    }

    private void intake(){ //throws InterruptedException{
        //Intake
        /*
        right trigger -
        right bumper
         */
        if (!intakefunctionon) {
            if (gamepad2.right_trigger > 0.1f)
                rb.intakeMotor.setPower(1);
            else if (gamepad2.right_bumper)
                rb.intakeMotor.setPower(-.8);
            else {
                rb.intakeMotor.setPower(0);
            }
        }
        else {
            if (gamepad2.right_bumper)
                rb.intakeMotor.setPower(-.8);
            else if ((gamepad2.right_trigger > 0.1f || !cargoin)) {
                //the encoder value condition is in place to ensure that the intake doesn't turn on
                //when trying to spit out cargo
                rb.intakeMotor.setPower(1);
            }
            else {
                rb.intakeMotor.setPower(0);
            }
        }
        /*
        if (gamepad2.right_trigger > 0.1f)
            rb.intakeMotor.setPower(-1);
        else if (gamepad2.right_bumper)
            rb.intakeMotor.setPower(1);
        else {
            rb.intakeMotor.setPower(0);
        }*/
        /*if (!intakefunctionon) {
            if (gamepad2.right_trigger > 0.1f)
                rb.intakeMotor.setPower(1);
            else if (gamepad2.right_bumper)
                rb.intakeMotor.setPower(-1);
            else {
                rb.intakeMotor.setPower(0);
            }
        }
        else {
            if (gamepad2.right_trigger > 0.1f)
                rb.intakeMotor.setPower(1);
            else if ((gamepad2.right_bumper || !cargoin) && rb.sliderSpool.getCurrentPosition() > -200) {
                //the encoder value condition is in place to ensure that the intake doesn't turn on
                //when trying to spit out cargo
                rb.intakeMotor.setPower(-1);
            }
            else {
                rb.intakeMotor.setPower(0);
            }
        }*/
    }
    private void turret() throws InterruptedException{
        /*
        if (gamepad1.left_trigger > TRIGGER_THRESHOLD)
            tbottom += TURRET_INCREMENT;
        else if (gamepad1.right_trigger > TRIGGER_THRESHOLD)
            tbottom -= TURRET_INCREMENT;

        if (gamepad1.left_bumper)
            tlifter += TURRET_INCREMENT;
        else if (gamepad1.right_bumper)
            tlifter -= TURRET_INCREMENT;

         */
        /*if(gamepad2.left_bumper){
            tlifter = TURRETTOPLEVELCLOSE;
        }
        else if(gamepad2.left_trigger > TRIGGER_THRESHOLD){
            tlifter = TURRETSHARED;
        }*/

        /*else if(gamepad2.right_stick_button){
            turret_sleep = SLOWTURRET_CYCLE_MS;
            if(gamepad2.left_stick_y > DRIVE_STICK_THRESHOLD2 && tlifter >= ATURRET_MIN_POS){// && tlifter <= ATURRET_MAX_POS){
                tlifter -= SLOWATURRET_INCREMENT;
            }
            else if(gamepad2.left_stick_y < -DRIVE_STICK_THRESHOLD2 && tlifter <= ATURRET_MAX_POS){// && tlifter >= ATURRET_MIN_POS){
                tlifter += SLOWATURRET_INCREMENT;
            }
            //since the y value needs to be reversed?
            if(gamepad2.right_stick_x > DRIVE_STICK_THRESHOLD2){// && tbottom >= BTURRET_MIN_POS){
                tbottom -= SLOWBTURRET_INCREMENT;
            }
            else if(gamepad2.right_stick_x < -DRIVE_STICK_THRESHOLD2){// && tbottom <= BTURRET_MAX_POS){
                tbottom += SLOWBTURRET_INCREMENT;
            }
        }*/
        //else {
        turret_sleep = TURRET_CYCLE_MS;
        /*if(gamepad2.left_stick_y > DRIVE_STICK_THRESHOLD2 && tlifter <= ATURRET_MIN_POS){// && tlifter <= ATURRET_MAX_POS){
            tlifter += ATURRET_INCREMENT;
        }
        else if(gamepad2.left_stick_y < -DRIVE_STICK_THRESHOLD2 && tlifter >= ATURRET_MAX_POS){// && tlifter >= ATURRET_MIN_POS){
            tlifter -= ATURRET_INCREMENT;
        }*/

        if(gamepad2.left_stick_y > DRIVE_STICK_THRESHOLD2){// && tlifter <= ATURRET_MAX_POS){
            tlifter += ATURRET_INCREMENT;
        }
        else if(gamepad2.left_stick_y < -DRIVE_STICK_THRESHOLD2){// && tlifter >= ATURRET_MIN_POS){
            tlifter -= ATURRET_INCREMENT;
        }
            /*
            if(gamepad2.left_stick_y > DRIVE_STICK_THRESHOLD2 && tlifter >= ATURRET_MIN_POS){// && tlifter <= ATURRET_MAX_POS){
                tlifter -= ATURRET_INCREMENT;
            }
            else if(gamepad2.left_stick_y < -DRIVE_STICK_THRESHOLD2 && tlifter <= ATURRET_MAX_POS){// && tlifter >= ATURRET_MIN_POS){
                tlifter += ATURRET_INCREMENT;
            }*/
        //since the y value needs to be reversed?
        if(gamepad2.right_stick_x > DRIVE_STICK_THRESHOLD2){// && tbottom >= BTURRET_MIN_POS){
            tbottom -= BTURRET_INCREMENT;
        }
        else if(gamepad2.right_stick_x < -DRIVE_STICK_THRESHOLD2){// && tbottom <= BTURRET_MAX_POS){
            tbottom += BTURRET_INCREMENT;
        }
        //}
        // Display the current value
        telemetry.addData("Bottom Servo Position", "%5.2f", rb.turretBottom.getPosition());
        telemetry.addData("Lifter Servo Position", "%5.2f", rb.turretLift.getPosition());
        telemetry.addData(">", "Press Stop to end test." );

        // Set the servo to the new position and pause;
        //turretBottom.setPower(bottom);
        rb.turretBottom.setPosition(tbottom);
        rb.turretLift.setPosition(tlifter);
        sleep(TURRET_CYCLE_MS);
        //sleep(TURRET_CYCLE_MS);
        //idle();
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



















