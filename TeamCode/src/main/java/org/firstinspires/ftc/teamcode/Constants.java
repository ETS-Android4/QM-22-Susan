package org.firstinspires.ftc.teamcode;

public class Constants {
    public static double GEAR_RATIO_MULTIPLIER = 1-(0.388268398268398);
    public static final double DEFAULT_ACCELERATION_INCREMENT = 0.02;

    //Turntable:
    public static final double blueturnTablePower = 0.2;
    public static final double redturnTablePower = -0.2;

    //Controls:
    public static final float DRIVE_STICK_THRESHOLD = .0f;
    public static final float DRIVE_STICK_THRESHOLD_SQUARED = DRIVE_STICK_THRESHOLD * DRIVE_STICK_THRESHOLD;
    public static final float TRIGGER_THRESHOLD = .4f;
    public static final int BUTTON_DELAY = 250; //Delay after any toggle button is pressed before button is checked again. idk

    //Intake slides:
    //TODO: fix these values (just the non-power ones)
    public static final int slideLevelZero= 0;
    public static final int slideLevelOne= 537*5;
    public static final int slideLevelTwo= 537*10;
    public static final int slideLevelThree= 537*20;
    public static final double slidePowerUp = -1;
    public static final double slidePowerDown = 0.5;

    //Motor:
    public static final int LIFTER_MOTOR_UP = -50;
    public static final int LIFTER_MOTOR_MID = (int) (100 * 4.2);
    public static final int LIFTER_MOTOR_DOWN = 1470;

    //Wheel Diameter = 96mm
    //96mm * pi = 301.5928947446mm = circumference = 1 wheel rotation
    //537.7 Ticks per 301.5928947446mm distance traveled
    //2 ft (1 tile distance) = 609.6mm
    //609.6/301.5928947446 = 2.0212677773 (rotations per 2ft/tile)
    //2.0212677773 * 537.7 ticks = 1,086.8356838365 ticks per tile?

    public static final double TICKS_PER_ROTATION = 537.7;
    public static final double ENCODER_DRIVE_ONE_TILE = 2.0212677773 * TICKS_PER_ROTATION; //Evaluates to 1488.684491978609626 right now

    public static final double INTAKE_SPEED = 1;

//    public static final double LIFTER_MAXIMUM = .58;
//    public static final double LIFTER_UP = .61;
//    public static final double LIFTER_MID = .79;
//    public static final double LIFTER_DOWN = 1;
//    public static final double LIFTER_MINIMUM = .99;

    //Color Sensor:
    //Setting the correct WHITE_ALPHA_THRESHOLD value is key to stopping correctly. This should be set half way between the light and dark values.
    public static final double WHITE_ALPHA_THRESHOLD = 170; //

    //Distance Sensor:
    //These are in inches:
    public static final double SIDE_TO_CENTER_DISTANCE = 7.44; //inches
    public static final double FRONT_TO_CENTER_DISTANCE = 10.17;
    public static final double CENTER_TO_TOWER_DISTANCE = FRONT_TO_CENTER_DISTANCE + 61;
    public static final double SIDE_WALL_TO_TOWER_DISTANCE = 34.5;
    //in mm

    public static final double WOBBLE_MININUM_DISTANCE = 13; // in mm
    public static final double WOBBLE_MOVE_DOWN_DELAY = 1500; //in ms

    public static final int HOPPER_ZERO_RINGS = 130;
    public static final int HOPPER_ONE_RING = 121;
    public static final int HOPPER_TWO_RINGS = 100;
    public static final int HOPPER_THREE_RINGS = 86;
    public static final int[] ringDistanceArray = {HOPPER_ZERO_RINGS, HOPPER_ONE_RING, HOPPER_TWO_RINGS, HOPPER_THREE_RINGS};

    public static final double WOBBLE_2M_THRESHOLD = 325; //in mm


}
