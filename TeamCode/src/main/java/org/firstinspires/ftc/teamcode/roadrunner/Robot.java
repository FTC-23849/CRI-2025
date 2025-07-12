package org.firstinspires.ftc.teamcode.roadrunner;

public class Robot {
    public static double INTAKE_TURRET_TRANSFER = 0.84;
    public static double INTAKE_TURRET_PICKUP_LEFT = 0.57;
    public static double INTAKE_TURRET_PICKUP_RIGHT = 0.07;
    public static double INTAKE_TURRET_PICKUP_STRAIGHT = 0.32;
    public static double INTAKE_TURRET_DROP_LEFT = 0.60;
    public static double INTAKE_TURRET_DROP_RIGHT = 0.02;
    public static double INTAKE_PIVOT_TRANSFER = 0.3;
    public static double INTAKE_PIVOT_PICKUP_READY = 0.15;
    public static double INTAKE_PIVOT_PICKUP = 0.06;
    public static double INTAKE_PIVOT_DROP = 0.2;
    public static double INTAKE_WRIST_STRAIGHT = 0.4;// 0.07 works, 0.574 should, done math to test
    public static double INTAKE_WRIST_LEFT90 = 0.66;
    public static double INTAKE_WRIST_RIGHT90 = 0.07;
    public static double INTAKE_CLAW_OPEN = 0.6;
    public static double INTAKE_CLAW_CLOSE = 0.42;
    public static int INTAKE_MOTOR_RETRACT = 0;
    public static int INTAKE_MOTOR_MAX_EXTEND = 850;
    public static int INTAKE_LOWER_ENCODER_TICKS = 250;


    public static double OUTTAKE_PIVOT_TRANSFER = 0;
    public static double OUTTAKE_PIVOT_SAMPLE_SCORE = 0.8;
    public static double OUTTAKE_PIVOT_SPECIMEN_PICKUP = 1;
    public static double OUTTAKE_PIVOT_HIGH_SPECIMEN_SCORE = 0.350;
    public static double OUTTAKE_PIVOT_LOW_SPECIMEN_SCORE = 35;
    public static double OUTTAKE_TURRET_STRAIGHT = 0.49;
    public static double OUTTAKE_TURRET_LEFT90 = 0.17;
    public static double OUTTAKE_TURRET_RIGHT90 = 0.85;
    public static double OUTTAKE_WRIST_TRANSFER =  0.0;
    public static double OUTTAKE_WRIST_SPECIMEN_PICKUP = 0.1;
    public static double OUTTAKE_WRIST_SAMPLE_SCORE = 0.3;
    public static double OUTTAKE_WRIST_HIGH_SPECIMEN_SCORE = 0.35;
    public static double OUTTAKE_WRIST_LOW_SPECIMEN_SCORE = 0.4;
    public static double OUTTAKE_CLAW_OPEN = 0.45;
    public static double OUTTAKE_CLAW_CLOSE = 0.51;
    public static int OUTTAKE_MOTOR_RETRACT = 0;
    public static int OUTTAKE_MOTOR_MAX_EXTEND = 0;
    public static int OUTTAKE_MOTOR_SAMPLE_SCORE = -700;
    public static int OUTTAKE_MOTOR_HIGH_SPECIMEN_SCORE = -450;
    public static int OUTTAKE_MOTOR_LOW_SPECIMEN_SCORE = 0;
    public static double LEFT_PTO_DISENGAGE = 0.25;
    public static double LEFT_PTO_ENGAGE = 0.56; // 0.46 is loose // 0.51 is still slipping
    public static double RIGHT_PTO_DISENGAGE = 0.5;
    public static double RIGHT_PTO_ENGAGE = 0.15; // 0.3 is loose, 0.2 is still too lose




    public static class intake {

    }

    public static class outtake {

    }



}