package org.firstinspires.ftc.teamcode.PierreAuto;

import static org.firstinspires.ftc.teamcode.Constants.DEFAULT_ACCELERATION_INCREMENT;
import static org.firstinspires.ftc.teamcode.Constants.ENCODER_DRIVE_ONE_TILE;
import static org.firstinspires.ftc.teamcode.Constants.slideLevelZero;

import android.annotation.SuppressLint;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.robotcore.external.navigation.AxesOrder;
import org.firstinspires.ftc.robotcore.external.navigation.AxesReference;
import org.firstinspires.ftc.robotcore.external.navigation.Orientation;
import org.firstinspires.ftc.teamcode.PierreTeleOp.HardwareRobot;

import java.util.Locale;

@Autonomous(name = "BlueParkStorageUnit", group = "!Primary")
public class BlueParkStorageUnit extends LinearOpMode {
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


        //TODO: Actual Auto Driving Code goes here
        rb.LifterByEncoder(slideLevelZero, rb.sliderSpool);
        rb.strafeRightByEncoderAndIMU(-(int)(ENCODER_DRIVE_ONE_TILE*.5), rb.LFmotor, 1, .05);
        rb.driveForwardByEncoderAndIMU((int)(ENCODER_DRIVE_ONE_TILE*1.3), rb.LFmotor, 1, .06, DEFAULT_ACCELERATION_INCREMENT * 2); //Drive to A Zone
        rb.strafeRightByEncoderAndIMU(-(int)(ENCODER_DRIVE_ONE_TILE*.7), rb.LFmotor, 1, .05);
        rb.driveStop();


    }

    /**
     * Logs IMU data to telemetry, TODO: Throttle or disable for competition - I think we can comment this out
     */
    /*
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
    */


    //----------------------------------------------------------------------------------------------
    // Formatting
    //----------------------------------------------------------------------------------------------

    String formatAngle(AngleUnit angleUnit, double angle) {
        return formatDegrees(AngleUnit.DEGREES.fromUnit(angleUnit, angle));
    }

    String formatDegrees(double degrees) {
        return String.format(Locale.getDefault(), "%.1f", AngleUnit.DEGREES.normalize(degrees));
    }
/**
 * Zachary's code
 * star wars transformers marvel nerf baby groot groughgu tj;oasolfjtltyguktfgh6iytijtjoijyoijdfskjg'skljyjyoijoty;ijy
 */

}
