package org.firstinspires.ftc.teamcode.PierreTeleOp;


import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.robotcore.external.navigation.Orientation;
import org.firstinspires.ftc.teamcode.PierreTeleOp.HardwareRobot;


@TeleOp(name = "!TF TeleOP", group = "!Primary")
public class TFteleop extends LinearOpMode {

    //private final FtcDashboard dashboard = FtcDashboard.getInstance(); //Comment this out when not using dashboard

    private final HardwareRobot rb = new HardwareRobot();
    private final ElapsedTime runtime = new ElapsedTime();
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

        telemetry.addData("Status", "Initialized, Ready to Start. Make sure lifter is in back position");
        telemetry.update();
        // Wait for the game to start (driver presses PLAY)

        waitForStart(); //Everything up to here is initialization
        runtime.reset();
        telemetry.setAutoClear(true);

        while (opModeIsActive()) {

            drive();
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
        if (gamepad1.dpad_up) {
            rb.drive(1);
    /*    } else if (gamepad1.dpad_down) {
            rb.drive(-1);
        } else if (gamepad1.dpad_left) {
            rb.strafe(-1);
        } else if (gamepad1.dpad_right) {
            rb.strafe(1);
        } else if (gamepad1.left_bumper) {
            rb.RFmotor.setPower(1);
            rb.LFmotor.setPower(-1);
            rb.RBmotor.setPower(1);
            rb.LBmotor.setPower(-1);
        } else if (gamepad1.right_bumper) {
            rb.RFmotor.setPower(-1);
            rb.LFmotor.setPower(1);
            rb.RBmotor.setPower(-1);
            rb.LBmotor.setPower(1);
    */
        } else {
            rb.driveStop();
        }
    }
}



















