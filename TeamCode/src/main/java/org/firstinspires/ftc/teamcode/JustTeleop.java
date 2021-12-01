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
    public static int slideLevelOne= 537*5;
    public static int slideLevelTwo= 537*10;
    public static int slideLevelThree= 537*20;

    public static final float DRIVE_STICK_THRESHOLD = .0f;
    public static final float DRIVE_STICK_THRESHOLD_SQUARED = DRIVE_STICK_THRESHOLD * DRIVE_STICK_THRESHOLD;


    public void init() {
        RFmotor = hardwareMap.get(DcMotor.class, "rightfront");
        RBmotor = hardwareMap.get(DcMotor.class, "rightback");
        LFmotor = hardwareMap.get(DcMotor.class, "leftfront");
        LBmotor = hardwareMap.get(DcMotor.class, "leftback");
        turnTable = hardwareMap.get(DcMotor.class, "turntable");
        sliderSpool = hardwareMap.get(DcMotor.class, "slider");
        intakeMotor= hardwareMap.get(DcMotor.class, "intake");
        telemetry.addData("Hardware", "Initialized");
        telemetry.update();

        //Left front wheel in port 0
    //Right front in port 1
    //Left Back port 2
    //Right Back port 3
    // turntable in port 0
    //Slider in port 1
    //intake in port 2


    }

    @Override
    public void loop() {

        //turntable
        if(gamepad2.a){
            double turnTablePower=0.2;
            turnTable.setPower(turnTablePower);
        telemetry.addData("A button","Pressed");
        } else{
            turnTable.setPower(0.0);}

        //Slider

        if (gamepad2.left_trigger > 0.1)
            sliderSpool.setPower(-1);
        if (gamepad2.left_bumper)
            sliderSpool.setPower(.5);
        else
            sliderSpool.setPower(0);
        double funnyNumber = 69420.69420; //Boone's first line of robotics code!!!!
        //Intake
        if (gamepad2.right_trigger > 0.1)
            intakeMotor.setPower(gamepad2.right_trigger);
        if (gamepad2.right_bumper)
            intakeMotor.setPower(-.75);
        else{
            intakeMotor.setPower(0);
        }
        if (funnyNumber == 69420.69420){
            System.out.println("FUNNY NUMBER");
            String calebGod = "Boone is the official founder of Niehism";
            System.out.println(calebGod);
        }
        else if(funnyNumber != 69420.69420){
            /*for(int i = 0;i<69420;i++) {
                System.out.println("THAT IS AN UNFUNNY NUMBER, NOW IT'S" + i);
                System.out.println("Fake Funny Number!");
            }*/
            System.out.println("FUNNY NUMBER ACHIEVED, UNIVERSE SAVED. ALL HAIL KING CALEB");
        }


        //Init variables
//hi caleb :)



//
//Drive Controls:
// Left stick= Translational Movement
// Right Stick= Rotational Movement
        //DRIVE_STICK_THRESHOLD = deadzone

        double MAX_SPEED = 1.0;

        double frontLeftPower;
        double frontRightPower;
        double rearLeftPower;
        double rearRightPower;

        double leftY = gamepad1.left_stick_y;
        double leftX = gamepad1.left_stick_x;
        double rightX = gamepad1.right_stick_x;

//        pattern = RevBlinkinLedDriver.BlinkinPattern.ORANGE;
//        blinkinLedDriver.setPattern(pattern);
        //DRIVE_STICK_THRESHOLD = deadzone
        if (rightX < -DRIVE_STICK_THRESHOLD || rightX > DRIVE_STICK_THRESHOLD || leftY < -DRIVE_STICK_THRESHOLD || leftY > DRIVE_STICK_THRESHOLD || leftX < -DRIVE_STICK_THRESHOLD || leftX > DRIVE_STICK_THRESHOLD) {
            //Get stick values and apply modifiers:
            double drive = (-gamepad1.left_stick_y * 1.10);
            double turn = (gamepad1.right_stick_x * 1.25);
            double strafe = (gamepad1.left_stick_x);

            //Calculate each individual motor speed using the stick values:
            //range.clip calculates a value between min and max, change those values to reduce overall speed
            frontLeftPower = Range.clip(drive + turn + strafe, -1.0, 1.0);
            frontRightPower = Range.clip(drive - turn - strafe, -1.0, 1.0);
            rearLeftPower = Range.clip(drive + turn - strafe, -1.0, 1.0);
            rearRightPower = Range.clip(drive - turn + strafe, -1.0, 1.0);

            RBmotor.setPower(-rearRightPower);
            RFmotor.setPower(-frontRightPower);
            LBmotor.setPower(-rearLeftPower);
            LFmotor.setPower(-frontLeftPower);

            telemetry.addData("Front-right motor", "%5.2f", frontRightPower);
            telemetry.addData("Back-right motor", "%5.2f", rearRightPower);
            telemetry.addData("Front-left motor", "%5.2f", frontLeftPower);
            telemetry.addData("Back-left motor", "%5.2f", rearLeftPower);
            telemetry.update();
        }
        else {
            RBmotor.setPower(0);
            RFmotor.setPower(0);
            LBmotor.setPower(0);
            LFmotor.setPower(0); //Stop robot if no stick value (delete this if u want to drift lol)
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
}


















