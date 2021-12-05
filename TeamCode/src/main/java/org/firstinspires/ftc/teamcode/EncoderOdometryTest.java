package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

import org.firstinspires.ftc.teamcode.HardwareRobot;
import org.firstinspires.ftc.teamcode.MecanumOdometry;

@TeleOp(name = "EncoderOdometryTest", group = "Test")
public class EncoderOdometryTest extends LinearOpMode {
    private final HardwareRobot rb = new HardwareRobot();

    private final MecanumOdometry odometry = new MecanumOdometry();

    @Override
    public void runOpMode() throws InterruptedException {
        rb.init(hardwareMap, this);
        odometry.init(hardwareMap);
        telemetry.addData("FR Encoder", rb.RFmotor.getCurrentPosition());
        telemetry.addData("FL Encoder", rb.LFmotor.getCurrentPosition());
        telemetry.addData("BR Encoder", rb.RBmotor.getCurrentPosition());
        telemetry.addData("BL Encoder", rb.LBmotor.getCurrentPosition());
        telemetry.addData("Slider Spool Encoder", rb.sliderSpool.getCurrentPosition());

        telemetry.update();
        odometry.start(rb.RFmotor.getCurrentPosition(), rb.LFmotor.getCurrentPosition(), rb.LBmotor.getCurrentPosition());
        waitForStart();
        while (opModeIsActive()) {
            odometry.update(rb.RFmotor.getCurrentPosition(), rb.LFmotor.getCurrentPosition(), rb.LBmotor.getCurrentPosition());
            telemetry.addData("FR Encoder", rb.RFmotor.getCurrentPosition());
            telemetry.addData("FL Encoder", rb.LFmotor.getCurrentPosition());
            telemetry.addData("BR Encoder", rb.RBmotor.getCurrentPosition());
            telemetry.addData("BL Encoder", rb.LBmotor.getCurrentPosition());
            telemetry.addData("Slider Spool Encoder", rb.sliderSpool.getCurrentPosition());

            telemetry.update();
        }
    }
}
