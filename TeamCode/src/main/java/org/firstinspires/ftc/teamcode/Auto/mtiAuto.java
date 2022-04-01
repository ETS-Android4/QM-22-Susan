package org.firstinspires.ftc.teamcode.Auto;

import android.graphics.Color;

import com.acmerobotics.roadrunner.geometry.Pose2d;
import com.acmerobotics.roadrunner.geometry.Vector2d;
import com.qualcomm.hardware.rev.RevColorSensorV3;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.ColorSensor;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DistanceSensor;
import com.qualcomm.robotcore.hardware.NormalizedRGBA;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.hardware.SwitchableLight;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit;
import org.firstinspires.ftc.teamcode.RR.drive.SampleMecanumDrive;
import org.firstinspires.ftc.teamcode.RR.trajectorysequence.TrajectorySequence;
import org.firstinspires.ftc.teamcode.mechanism.WebcamExample;

@Autonomous(name = "mtiAuto", group = "Exercises")
public class mtiAuto extends LinearOpMode {
    private ElapsedTime runtime = new ElapsedTime(); //Declared AND Initialized
    public DcMotor FrontLeft; //Declared  but not initialized
    public DcMotor FrontRight;
    public DcMotor BackLeft;
    public DcMotor BackRight;
    public DcMotor Intake;
    public DcMotor Spinner;
    public DcMotor Intake2;
    public DcMotor Slide;
    public Servo Bucket;
    int x = 0;
    public WebcamExample webcamExample = null;
    final float[] hsvValues = new float[3];
    float gain = 2;
    boolean hasFreight;
    int level;
    Pose2d currentPose = new Pose2d(-6, 64, Math.toRadians(90));
    RevColorSensorV3 colorSensor;

    public void mecanumDrive(String driveType, double value1, double power) {

        FrontLeft.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        BackLeft.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        FrontRight.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        BackRight.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        Slide.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        if (driveType.equals("forward")) {
            FrontLeft.setTargetPosition((int) (42.78 * value1)); //enter value in inches
            BackLeft.setTargetPosition((int) (42.78 * value1));
            FrontRight.setTargetPosition((int) (42.78 * value1));
            BackRight.setTargetPosition((int) (42.78 * value1));
            FrontLeft.setMode(DcMotor.RunMode.RUN_TO_POSITION);
            BackLeft.setMode(DcMotor.RunMode.RUN_TO_POSITION);
            FrontRight.setMode(DcMotor.RunMode.RUN_TO_POSITION);
            BackRight.setMode(DcMotor.RunMode.RUN_TO_POSITION);
            FrontLeft.setPower(power);
            BackLeft.setPower(power);
            FrontRight.setPower(power);
            BackRight.setPower(power);
        } else if (driveType.equals("strafe")) {
            FrontLeft.setTargetPosition((int) (47.53 * value1)); //enter value in inches
            BackLeft.setTargetPosition((int) (-47.53 * value1));
            FrontRight.setTargetPosition((int) (-47.53 * value1));
            BackRight.setTargetPosition((int) (47.53 * value1));
            FrontLeft.setMode(DcMotor.RunMode.RUN_TO_POSITION);
            BackLeft.setMode(DcMotor.RunMode.RUN_TO_POSITION);
            FrontRight.setMode(DcMotor.RunMode.RUN_TO_POSITION);
            BackRight.setMode(DcMotor.RunMode.RUN_TO_POSITION);
            FrontLeft.setPower(power);
            BackLeft.setPower(power);
            FrontRight.setPower(power);
            BackRight.setPower(power);
        } else if (driveType.equals("turn")) {
            FrontLeft.setTargetPosition((int) (10.12 * value1)); //enter value in degrees
            BackLeft.setTargetPosition((int) (10.12 * value1));
            FrontRight.setTargetPosition((int) (-10.12 * value1));
            BackRight.setTargetPosition((int) (-10.12 * value1));
            FrontLeft.setMode(DcMotor.RunMode.RUN_TO_POSITION);
            BackLeft.setMode(DcMotor.RunMode.RUN_TO_POSITION);
            FrontRight.setMode(DcMotor.RunMode.RUN_TO_POSITION);
            BackRight.setMode(DcMotor.RunMode.RUN_TO_POSITION);
            FrontLeft.setPower(power);
            BackLeft.setPower(power);
            FrontRight.setPower(power);
            BackRight.setPower(power);
        }

        //noinspection StatementWithEmptyBody
        while ((FrontLeft.isBusy() || BackLeft.isBusy() || BackRight.isBusy() || FrontRight.isBusy()) && opModeIsActive()) {
        }
        FrontLeft.setPower(0.0);
        BackLeft.setPower(0.0);
        FrontRight.setPower(0.0);
        BackRight.setPower(0.0);

    }

    public void scoreFreight() {
//        if(level == 3) {
        Slide.setTargetPosition(-1050);
//        }
//
//        if(level == 2) {
//            Slide.setTargetPosition(-700);
//        }
//
//        if (level == 1) {
//            Slide.setTargetPosition(-400);
//        }

        Slide.setPower(1);
        Slide.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        while (Slide.isBusy() && opModeIsActive()) {
        }
        resetStartTime();
        double bucketTime = runtime.seconds();
        while (opModeIsActive() && runtime.seconds() < bucketTime + 1.5) {
            Bucket.setPosition(0.3);
        }
        while (opModeIsActive() && runtime.seconds() < bucketTime + 2.5) {
            Bucket.setPosition(0.77);
        }

        Slide.setTargetPosition(0);
        Slide.setPower(1);
        Slide.setMode(DcMotor.RunMode.RUN_TO_POSITION);
    }


    public void intakeColor() {

        runtime.reset();
        while (opModeIsActive() && hasFreight == false) {
            colorSensor();
        }
        if (hasFreight) {
            telemetry.addData("Freight", "Yes");

        } else {
            telemetry.addData("Freight", "No");
        }
        telemetry.update();
    }

    public void colorSensor() {
        // You can give the sensor a gain value, will be multiplied by the sensor's raw value before the
        // normalized color values are calculated. Color sensors (especially the REV Color Sensor V3)
        // can give very low values (depending on the lighting conditions), which only use a small part
        // of the 0-1 range that is available for the red, green, and blue values. In brighter conditions,
        // you should use a smaller gain than in dark conditions. If your gain is too high, all of the
        // colors will report at or near 1, and you won't be able to determine what color you are
        // actually looking at. For this reason, it's better to err on the side of a lower gain
        // (but always greater than  or equal to 1).
        float gain = 2;

        // Once per loop, we will update this hsvValues array. The first element (0) will contain the
        // hue, the second element (1) will contain the saturation, and the third element (2) will
        // contain the value. See http://web.archive.org/web/20190311170843/https://infohost.nmt.edu/tcc/help/pubs/colortheory/web/hsv.html
        // for an explanation of HSV color.
        final float[] hsvValues = new float[3];

        // Get a reference to our sensor object. It's recommended to use NormalizedColorSensor over
        // ColorSensor, because NormalizedColorSensor consistently gives values between 0 and 1, while
        // the values you get from ColorSensor are dependent on the specific sensor you're using.
        colorSensor = hardwareMap.get(RevColorSensorV3.class, "Color");

        // If possible, turn the light on in the beginning (it might already be on anyway,
        // we just make sure it is if we can).
        if (colorSensor instanceof SwitchableLight) {
            ((SwitchableLight) colorSensor).enableLight(true);
        }


        Bucket.setPosition(0.77);

        // Loop until we are asked to stop

        colorSensor.setGain(gain);

        // If the button state is different than what it as, then act

        if (colorSensor instanceof SwitchableLight) {
            SwitchableLight light = (SwitchableLight) colorSensor;
            light.enableLight(!light.isLightOn());
        }

        // Get the normalized colors from the sensor
        NormalizedRGBA colors = colorSensor.getNormalizedColors();

        /* Use telemetry to display feedback on the driver station. We show the red, green, and blue
         * normalized values from the sensor (in the range of 0 to 1), as well as the equivalent
         * HSV (hue, saturation and value) values. See http://web.archive.org/web/20190311170843/https://infohost.nmt.edu/tcc/help/pubs/colortheory/web/hsv.html
         * for an explanation of HSV color. */

        // Update the hsvValues array by passing it to Color.colorToHSV()
        Color.colorToHSV(colors.toColor(), hsvValues);

        /* If this color sensor also has a distance sensor, display the measured distance.
         * Note that the reported distance is only useful at very close range, and is impacted by
         * ambient light and surface reflectivity. */
        if (colorSensor instanceof DistanceSensor) {
            telemetry.addData("Distance (cm)", "%.3f", ((DistanceSensor) colorSensor).getDistance(DistanceUnit.CM));
        }

        double dist = ((DistanceSensor) colorSensor).getDistance(DistanceUnit.CM);
        double hue = hsvValues[0];

        if (dist < 2.5) {
            if (60 < hue && hue < 95 || hue > 100 && hsvValues[2] > 0.1) {
                telemetry.addLine("The intake has a block in it");
                hasFreight = true;
            } else {
                telemetry.addLine("The intake has a ball in it");
                hasFreight = true;
            }
        } else {
            telemetry.addLine("The intake is empty");
            hasFreight = false;
        }

        telemetry.update();

    }

//    public void getPosition() {
//        SampleMecanumDrive myLocalizer = new SampleMecanumDrive(hardwareMap);
//
//        myLocalizer.update();
//
//        Pose2d currentPose = myLocalizer.getPoseEstimate();
//    }

    @Override
    public void runOpMode() throws InterruptedException {
        SampleMecanumDrive drive = new SampleMecanumDrive(hardwareMap);
        FrontLeft = hardwareMap.get(DcMotor.class, "FrontLeft");
        BackLeft = hardwareMap.get(DcMotor.class, "BackLeft");
        FrontRight = hardwareMap.get(DcMotor.class, "FrontRight");
        BackRight = hardwareMap.get(DcMotor.class, "BackRight");
        Intake = hardwareMap.get(DcMotor.class, "Intake");
        Spinner = hardwareMap.get(DcMotor.class, "Spinner");
        Intake2 = hardwareMap.get(DcMotor.class, "Intake2");
        Slide = hardwareMap.get(DcMotor.class, "Slide");
        Bucket = hardwareMap.get(Servo.class, "Bucket");

        ColorSensor color;
        color = hardwareMap.get(ColorSensor.class, "Color");

        // Get a reference to our sensor object. It's recommended to use NormalizedColorSensor over
        // ColorSensor, because NormalizedColorSensor consistently gives values between 0 and 1, while
        // the values you get from ColorSensor are dependent on the specific sensor you're using.
        color = hardwareMap.get(RevColorSensorV3.class, "Color");

        // If possible, turn the light on in the beginning (it might already be on anyway,
        // we just make sure it is if we can).
        if (color instanceof SwitchableLight) {
            ((SwitchableLight) color).enableLight(true);
        }

        webcamExample = new WebcamExample();
        webcamExample.init(hardwareMap);
        //Set starting pose

        Pose2d startPose = new Pose2d(12, 64, Math.toRadians(0));

        drive.setPoseEstimate(startPose);

        // Get the normalized colors from the sensor

        int level;
        int[] counts = {0, 0, 0};
        for (int i = 0; i < 500; i++) {
            if (webcamExample.getShippingHubLevel() == 0) {
                i = 0;
                continue;
            }
            counts[webcamExample.getShippingHubLevel() - 1]++;
        }

        if (counts[0] > counts[1] && counts[0] > counts[2]) { // Level = 1
            level = 1;

        } else if (counts[1] > counts[0] && counts[1] > counts[2]) { // Level = 2
            level = 2;
        } else { // Level = 3
            level = 3;
        }
        telemetry.addData("Hub Level", level);
        telemetry.update();

        if (isStopRequested()) return;

        waitForStart();

        //Score first cube


        //Cycle from starting position to warehouse
        TrajectorySequence traj = drive.trajectorySequenceBuilder(startPose)
                .setReversed(false)
                .splineTo(new Vector2d(50, 66), Math.toRadians(0))
                .build();
        drive.followTrajectorySequence(traj);

        //intake freight

        //Cycle out of warehouse
        TrajectorySequence traj2 = drive.trajectorySequenceBuilder(traj.end())
                .setReversed(true)
                .splineTo(new Vector2d(12, 65), Math.toRadians(180))
                .build();
        drive.followTrajectorySequence(traj2);
//
//        while (!hasFreight && !isStopRequested()) {
//            colorSensor();
//            Intake.setPower(-0.4);
//            Intake2.setPower(1);
//            drive.setWeightedDrivePower(new Pose2d(-.15, 0, 0));
//            drive.setWeightedDrivePower(new Pose2d(.225, 0, 0));
//        }
//        sleep(300);
        drive.update();

        //Back to warehouse
        TrajectorySequence traj3 = drive.trajectorySequenceBuilder(drive.getPoseEstimate())
                .setReversed(false)
                .splineTo(new Vector2d(50, 66), Math.toRadians(0))
                .build();


        drive.followTrajectorySequence(traj3);
    }
}


