package org.firstinspires.ftc.teamcode.Auto;

import android.annotation.SuppressLint;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.robotcore.external.ClassFactory;
import org.firstinspires.ftc.robotcore.external.Func;
import org.firstinspires.ftc.robotcore.external.Telemetry;
import org.firstinspires.ftc.robotcore.external.hardware.camera.WebcamName;
import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.robotcore.external.navigation.AxesOrder;
import org.firstinspires.ftc.robotcore.external.navigation.AxesReference;
import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit;
import org.firstinspires.ftc.robotcore.external.navigation.Orientation;
import org.firstinspires.ftc.robotcore.external.navigation.VuforiaLocalizer;
import org.firstinspires.ftc.robotcore.external.tfod.Recognition;
import org.firstinspires.ftc.robotcore.external.tfod.TFObjectDetector;
import org.firstinspires.ftc.teamcode.HardwareRobot;
import org.firstinspires.ftc.robotcore.external.navigation.VuforiaLocalizer.CameraDirection;
import static org.firstinspires.ftc.teamcode.Constants.DEFAULT_ACCELERATION_INCREMENT;
import static org.firstinspires.ftc.teamcode.Constants.ENCODER_DRIVE_ONE_TILE;
import static org.firstinspires.ftc.teamcode.Constants.slideLevelOne;
import static org.firstinspires.ftc.teamcode.Constants.slideLevelThree;
import static org.firstinspires.ftc.teamcode.Constants.slideLevelTwo;


import java.util.List;
import java.util.Locale;

@Autonomous(name = "BlueAuto(Storage Unit Start)", group = "!Primary")
public class BlueFromStorageUnit extends LinearOpMode{
    private static final String TFOD_MODEL_ASSET = "FreightFrenzy_BCDM.tflite";
    private static final String[] LABELS = {
            "Ball",
            "Cube",
            "Duck",
            "Marker"
    };

    /*
     * IMPORTANT: You need to obtain your own license key to use Vuforia. The string below with which
     * 'parameters.vuforiaLicenseKey' is initialized is for illustration only, and will not function.
     * A Vuforia 'Development' license key, can be obtained free of charge from the Vuforia developer
     * web site at https://developer.vuforia.com/license-manager.
     *
     * Vuforia license keys are always 380 characters long, and look as if they contain mostly
     * random data. As an example, here is a example of a fragment of a valid key:
     *      ... yIgIzTqZ4mWjk9wd3cZO9T1axEqzuhxoGlfOOI2dRzKS4T0hQ8kT ...
     * Once you've obtained a license key, copy the string from the Vuforia web site
     * and paste it in to your code on the next line, between the double quotes.
     */
    private static final String VUFORIA_KEY =
            "Afq25Qj/////AAABmTNyx63v9EjXoDH/Dv1lQAVKbwOv664hHv+G8fj9ElyWB6pPSnb59s7kxcbx7RFwG1yjnz9E3ueQR0aM25dI/mPgRNPN1mlhi6IMDc7+tiP8Xj0ZJ90hj+B2gUzx78H79yhbCyQasCchLcbV2LNQT70JeBykiMLK7Mmr1XKPbE27d3Y5PHgVWzPAkwo2CyzNxsUPDMUNd/NDj4UQa4L9rYpaVzS0EoJni6QWFS1EBhJfH36M/qBmloEOP1MrXN6KEsyuWgQg7GIRh+k+9MSgs1glTCdEniwHNU2eI1Qvheq9AfF4OSK46yQM4gjR8mjIb3MlFmN8NJ9zNULolimrx1V5+jAGUMxdZ3u0GYunn7Fq";

    /**
     * {@link #vuforia} is the variable we will use to store our instance of the Vuforia
     * localization engine.
     */
    private VuforiaLocalizer vuforia;

    /**
     * {@link #tfod} is the variable we will use to store our instance of the TensorFlow Object
     * Detection engine.
     */
    private TFObjectDetector tfod;

    private final HardwareRobot rb = new HardwareRobot();
    // Declare OpMode members.
    private final ElapsedTime runtime = new ElapsedTime();

    Orientation angles;

    @SuppressLint("DefaultLocale")
    @Override
    public void runOpMode() throws InterruptedException {
        telemetry.setAutoClear(false);
        telemetry.addData(">", "REMEMBER TO CHECK WOBBLE MOTOR AND SERVO POSITIONS!");
        telemetry.addData("Status", "Initializing...");
        telemetry.update();

        rb.init(hardwareMap, this); //runs init stuff in HardwareSimpleBot.java
        // The TFObjectDetector uses the camera frames from the VuforiaLocalizer, so we create that
        // first.
        initVuforia();
        initTfod();
        /*
          Activate TensorFlow Object Detection before we wait for the start command.
          Do it here so that the Camera Stream window will have the TensorFlow annotations visible.
         */

        if (tfod != null) {
            tfod.activate();
            // The TensorFlow software will scale the input images from the camera to a lower resolution.
            // This can result in lower detection accuracy at longer distances (> 55cm or 22").
            // If your target is at distance greater than 50 cm (20") you can adjust the magnification value
            // to artificially zoom in to the center of image.  For best results, the "aspectRatio" argument
            // should be set to the value of the images used to create the TensorFlow Object Detection model
            // (typically 1.78 or 16/9).

            // Uncomment the following line if you want to adjust the magnification and/or the aspect ratio of the input images.
            tfod.setZoom(2, 16.0/9.0);
            //tfod.setZoom(3, 1.78);
        }


        telemetry.addData("Mode", "Calibrating IMU...");
        telemetry.update();

        // make sure the imu gyro is calibrated before continuing.
        while (!isStopRequested() && !rb.imu.isGyroCalibrated()) {
            sleep(50);
            idle();
        }

        telemetry.addData("imu calib status: ", rb.imu.getCalibrationStatus().toString());


        telemetry.addData("Mode", "Resetting Encoders...");
        telemetry.update();
        //Reset Encoders
        rb.RFmotor.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        rb.RBmotor.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        rb.LFmotor.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        rb.LBmotor.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        rb.sliderSpool.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        rb.intakeMotor.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);

        Thread.sleep(150);

        rb.RFmotor.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        rb.RBmotor.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        rb.LBmotor.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        rb.LBmotor.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        rb.sliderSpool.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        rb.intakeMotor.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        telemetry.addData("Mode", "Done Resetting Encoders...");
        telemetry.update();

        // Wait for the game to start (driver presses PLAY)
        telemetry.addData("Status", "Initialized");
        telemetry.update();
        waitForStart(); //Everything up to here is initialization
        runtime.reset();

        // run until the end of the match (driver presses STOP)
        //Create global variables here
        int angleFacingForward; //Stores heading at beginning of match so we can reference it later
        angles = rb.imu.getAngularOrientation(AxesReference.INTRINSIC, AxesOrder.ZYX, AngleUnit.DEGREES);
        angleFacingForward = (int) angles.firstAngle;
        telemetry.addData("Angle Captured=", angleFacingForward);
        telemetry.update();

        Thread.sleep(150); //Wait 1000ms for camera to detect after pressing Start (2000 for testing bc idk) TODO: LOWER THIS IF WE NEED MORE TIME FOR AUTO
        telemetry.addData(">", "One second has passsed... Finding duck/marker");
        //Get placement of marker from Camera (which is already on)

        int duckPlacement = 2;
                /*
                duckPlacement meaning
                0 = leftmost spot
                1 = center spot
                2 = rightmost spot
                 */
        if (tfod != null) {
            // getUpdatedRecognitions() will return null if no new information is available since
            // the last time that call was made.
            List<Recognition> updatedRecognitions = tfod.getUpdatedRecognitions();
            if (updatedRecognitions == null){
                updatedRecognitions = tfod.getRecognitions(); //check recognitions one more time? i think
            }
            if (updatedRecognitions != null) {
                telemetry.addData("# Object Detected", updatedRecognitions.size());

                // step through the list of recognitions and display boundary info.
                int i = 0;
                for (Recognition recognition : updatedRecognitions) {
                    telemetry.addData(String.format("label (%d)", i), recognition.getLabel());
                    telemetry.addData(String.format("  left,top (%d)", i), "%.03f , %.03f",
                            recognition.getLeft(), recognition.getTop());
                    telemetry.addData(String.format("  right,bottom (%d)", i), "%.03f , %.03f",
                            recognition.getRight(), recognition.getBottom());
                    i++;
                    if (recognition.getLabel().equals("Duck") || recognition.getLabel().equals("Cube")){
                        if(recognition.getLeft() + 50 < 650){
                            duckPlacement = 0;
                        }
                        else{
                            duckPlacement = 1;
                        }
                    }
                }
                telemetry.addData("Duck is on ", duckPlacement);
                telemetry.update();
                //leftmost ~ 280
                //rigthmost ~1020
                //ducklength ~100
                //middle = 650
                //0 - turntable
                //1 - slider
                //2 - intake
            }
            else {
                telemetry.addData("no duck detected", updatedRecognitions.size());
                telemetry.update();
            }
        }

        if (tfod != null) { //Shutdown to free up system resources
            telemetry.addData(">", "tfod Shutting Down...");
            telemetry.update();

            tfod.shutdown();
            Thread.sleep(250);

            telemetry.addData(">", "tfod Shutdown");
            telemetry.update();
        }

        //TODO: Actual Auto Driving Code goes here
        /**
         * This section here is the only part of the red code that should differ from Red Storage Unit
         * Once they deliver they should be in the same position
         */
        rb.driveForwardByEncoderAndIMU((int)(ENCODER_DRIVE_ONE_TILE*.5), rb.LFmotor, 1, .06, DEFAULT_ACCELERATION_INCREMENT * 2); //Drive to A Zone
        rb.strafeRightByEncoderAndIMU((int)(ENCODER_DRIVE_ONE_TILE*1.4), rb.LFmotor, 1, .05);
        rb.rotate(90, .3);

        //TODO: code to raise up sliders
        if (duckPlacement == 0){
            rb.LifterByEncoder(slideLevelOne, rb.sliderSpool);
        }
        else if (duckPlacement == 1){
            rb.LifterByEncoder(slideLevelTwo, rb.sliderSpool);
            rb.driveForwardByEncoderAndIMU((int)(ENCODER_DRIVE_ONE_TILE*.075), rb.LFmotor, 0.2, .06, DEFAULT_ACCELERATION_INCREMENT * 2); //Drive to A Zone
        }
        else{
            rb.LifterByEncoder(slideLevelThree, rb.sliderSpool);
            rb.driveForwardByEncoderAndIMU((int)(ENCODER_DRIVE_ONE_TILE*.15), rb.LFmotor, 0.2, .06, DEFAULT_ACCELERATION_INCREMENT * 2); //Drive to A Zone
        }
        rb.runIntake(true,true);
        Thread.sleep(500);
        rb.runIntake(false,true);



        /**
         * This part is from the alliance shipping hub to the carousel
         */
        rb.driveForwardByEncoderAndIMU(-(int)(ENCODER_DRIVE_ONE_TILE*.5), rb.LFmotor, 1, .06, DEFAULT_ACCELERATION_INCREMENT * 2); //Drive to A Zone
        rb.LifterByEncoder(slideLevelOne, rb.sliderSpool);
        rb.strafeRightByEncoderAndIMU((int)(ENCODER_DRIVE_ONE_TILE*2.5), rb.LFmotor, 1, .05);
        rb.driveForwardByEncoderAndIMU(-(int)(ENCODER_DRIVE_ONE_TILE*.3), rb.LFmotor, 0.5, .06, DEFAULT_ACCELERATION_INCREMENT * 2); //Drive to A Zone

        //then turn the carousel on?
        rb.setTurnTable(true,false);
        Thread.sleep(3000);
        rb.setTurnTable(false,false);

        //parking in the warehouse
        rb.strafeRightByEncoderAndIMU(-(int)(ENCODER_DRIVE_ONE_TILE*.75), rb.LFmotor, 1, .05);
        rb.rotate(-90, .3);
        rb.strafeRightByEncoderAndIMU(-(int)(ENCODER_DRIVE_ONE_TILE*1), rb.LFmotor, 1, .05);
        rb.driveForwardByEncoderAndIMU((int)(ENCODER_DRIVE_ONE_TILE*3.5), rb.LFmotor, 1, .06, DEFAULT_ACCELERATION_INCREMENT * 2); //Drive to A Zone


        /*
        rb.autoDriveSouthWestWithEncoderAndIMU(2104, rb.LFmotor, .8, .06);
        rb.autoDriveNorthWestWithEncoderAndIMU(2104, rb.RFmotor, .8, .06);
        rb.driveForwardByEncoderAndIMU((int)(ENCODER_DRIVE_ONE_TILE*1.2), rb.LFmotor, 1, .06, DEFAULT_ACCELERATION_INCREMENT * 2); //Drive to A Zone
        rb.strafeRightByEncoderAndIMU((int)(ENCODER_DRIVE_ONE_TILE*1.2), rb.LFmotor, 1, .05);
        rb.rotate(3, .3);
        */
        rb.driveStop();


    }


    /**
     * Initialize the Vuforia localization engine.
     */
    private void initVuforia() {
        /*
         * Configure Vuforia by creating a Parameter object, and passing it to the Vuforia engine.
         */
        VuforiaLocalizer.Parameters parameters = new VuforiaLocalizer.Parameters();

        parameters.vuforiaLicenseKey = VUFORIA_KEY;
        parameters.cameraDirection = CameraDirection.BACK;

        //  Instantiate the Vuforia engine
        vuforia = ClassFactory.getInstance().createVuforia(parameters);

        // Loading trackables is not necessary for the TensorFlow Object Detection engine.
    }

    /**
     * Initialize the TensorFlow Object Detection engine.
     */
    private void initTfod() {
        int tfodMonitorViewId = hardwareMap.appContext.getResources().getIdentifier(
                "tfodMonitorViewId", "id", hardwareMap.appContext.getPackageName());
        TFObjectDetector.Parameters tfodParameters = new TFObjectDetector.Parameters(tfodMonitorViewId);
        tfodParameters.minResultConfidence = 0.8f;
        tfodParameters.isModelTensorFlow2 = true;
        tfodParameters.inputSize = 320;
        tfod = ClassFactory.getInstance().createTFObjectDetector(tfodParameters, vuforia);
        tfod.loadModelFromAsset(TFOD_MODEL_ASSET, LABELS);
    }

    /**
     * Logs IMU data to telemetry, TODO: Throttle or disable for competition - I think we can comment this out
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



}

