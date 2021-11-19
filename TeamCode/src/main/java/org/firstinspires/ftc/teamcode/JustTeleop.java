package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.Gamepad;
import com.qualcomm.robotcore.util.ElapsedTime;
import com.qualcomm.robotcore.util.Range;


@TeleOp
    public class JustTeleop extends OpMode {
    DcMotor RFmotor;
    DcMotor RBmotor;
    DcMotor LFmotor;
    DcMotor LBmotor;
    DcMotor turnTable;
    DcMotor sliderSpool;
    DcMotor intakeMotor;

    public static final float DRIVE_STICK_THRESHOLD = .0f;
    public static final float DRIVE_STICK_THRESHOLD_SQUARED = DRIVE_STICK_THRESHOLD * DRIVE_STICK_THRESHOLD;


    public void init() {
        RFmotor = hardwareMap.get(DcMotor.class, "rightfront");
        RBmotor = hardwareMap.get(DcMotor.class, "rightback");
        LFmotor = hardwareMap.get(DcMotor.class, "leftfront");
        LBmotor = hardwareMap.get(DcMotor.class, "leftback");
        turnTable = hardwareMap.get(DcMotor.class, "turntable");
        sliderSpool = hardwareMap.get(DcMotor.class, "slider");
//            intakeMotor= hardwareMap.get(DcMotor.class, "intake");


        telemetry.speak("Press play to start");

    }

    @Override
    public void loop() {

            //turntable
            if(gamepad1.a){
                double turnTablePower=0.50;
                turnTable.setPower(turnTablePower);


            }
            else
                turnTable.setPower(0.0);




        if (gamepad1.left_trigger > 0.1)
            sliderSpool.setPower(.65);
        if (gamepad1.left_bumper)
            sliderSpool.setPower(-.65);
        else
            sliderSpool.setPower(0);


        //Init variables



//
//Drive Controls:
// Left stick= Translational Movement
// Right Stick= Rotational Movement
        //DRIVE_STICK_THRESHOLD = deadzone


        double Speed = -gamepad1.left_stick_y;
        double Turn = gamepad1.right_stick_x;
        double Strafe = gamepad1.left_stick_x;
        double MAX_SPEED = 1.0;

        double frontLeftPower;
        double frontRightPower;
        double rearLeftPower;
        double rearRightPower;


   if (Speed < -DRIVE_STICK_THRESHOLD || Speed > DRIVE_STICK_THRESHOLD || Turn < -DRIVE_STICK_THRESHOLD || Turn > DRIVE_STICK_THRESHOLD || Strafe < -DRIVE_STICK_THRESHOLD || Strafe > DRIVE_STICK_THRESHOLD && (gamepad1.left_stick_button || gamepad1.right_stick_button)){
       telemetry.speak("You're using slowmode");

       rearRightPower = Range.clip(Speed - Turn + Strafe, -1, 1);
        rearLeftPower = Range.clip(Speed + Turn - Strafe, -1, 1);
        frontRightPower = Range.clip(Speed - Turn - Strafe, -1, 1);
        frontLeftPower = Range.clip(Speed + Turn + Strafe, -1, 1);



        RBmotor.setPower(rearRightPower * 0.5);
        RFmotor.setPower(frontRightPower * 0.5);
        LBmotor.setPower(rearLeftPower * 0.5);
        LFmotor.setPower(frontLeftPower * 0.5);}
  else
        RBmotor.setPower(0);
        RFmotor.setPower(0);
        LBmotor.setPower(0);
        LFmotor.setPower(0);

   if (Speed < -DRIVE_STICK_THRESHOLD || Speed > DRIVE_STICK_THRESHOLD || Turn < -DRIVE_STICK_THRESHOLD || Turn > DRIVE_STICK_THRESHOLD || Strafe < -DRIVE_STICK_THRESHOLD || Strafe > DRIVE_STICK_THRESHOLD){
            rearRightPower = Range.clip(Speed - Turn + Strafe, -1, 1);
            rearLeftPower= Range.clip(Speed + Turn - Strafe,-1, 1);
            frontRightPower= Range.clip(Speed - Turn - Strafe, -1, 1);
            frontLeftPower = Range.clip(Speed + Turn + Strafe, -1, 1);

       telemetry.addData("Front-right motor", "%5.2f", frontRightPower);
       telemetry.addData("Back-right motor", "%5.2f", rearRightPower);
       telemetry.addData("Front-left motor", "%5.2f", frontLeftPower);
       telemetry.addData("Back-left motor", "%5.2f", rearLeftPower);
       telemetry.update();

            RBmotor.setPower(rearRightPower);
            RFmotor.setPower(frontRightPower);
            LBmotor.setPower(rearLeftPower);
            LFmotor.setPower(-frontLeftPower);


        }
        else
            RBmotor.setPower(0);
        RFmotor.setPower(0);
        LBmotor.setPower(0);
        LFmotor.setPower(0);
    }
    }


















