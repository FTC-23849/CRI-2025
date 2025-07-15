package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.hardware.Servo;

public class Robot {
    public static double INTAKE_TURRET_TRANSFER = 0.84;
    public static double INTAKE_TURRET_PICKUP_LEFT = 0.57;
    public static double INTAKE_TURRET_PICKUP_RIGHT = 0.07;
    public static double INTAKE_TURRET_PICKUP_STRAIGHT = 0.32;
    public static double INTAKE_TURRET_DROP_LEFT = 0.60;
    public static double INTAKE_TURRET_DROP_RIGHT = 0.02;
    public static double INTAKE_PIVOT_TRANSFER = 0.37;
    public static double INTAKE_PIVOT_PICKUP_READY = 0.11;
    public static double INTAKE_PIVOT_PICKUP = 0.06;
    public static double INTAKE_PIVOT_DROP = 0.2;
    public static double INTAKE_PIVOT_AVOID = 0.7;
    public static double INTAKE_PIVOT_RETRACT = 0.33;
    public static double INTAKE_WRIST_STRAIGHT = 0.4;// 0.07 works, 0.574 should, done math to test
    public static double INTAKE_WRIST_LEFT90 = 0.66;
    public static double INTAKE_WRIST_RIGHT90 = 0.07;
    public static double INTAKE_CLAW_OPEN = 0.6;
    public static double INTAKE_CLAW_CLOSE = 0.44;
    public static int INTAKE_MOTOR_RETRACT = 0;
    public static int INTAKE_MOTOR_MAX_EXTEND = 850;
    public static int INTAKE_LOWER_ENCODER_TICKS = 250;


    public static double OUTTAKE_PIVOT_TRANSFER = 0;
    public static double OUTTAKE_PIVOT_SAMPLE_SCORE = 0.8;
    public static double OUTTAKE_PIVOT_SPECIMEN_PICKUP = 1;
    public static double OUTTAKE_PIVOT_HIGH_SPECIMEN_SCORE = 0.350;
    public static double OUTTAKE_PIVOT_LOW_SPECIMEN_SCORE = 0.35;
    public static double OUTTAKE_PIVOT_HANG = 0.6;
    public static double OUTTAKE_TURRET_STRAIGHT = 0.49;
    public static double OUTTAKE_TURRET_LEFT90 = 0.17;
    public static double OUTTAKE_TURRET_RIGHT90 = 0.85;
    public static double OUTTAKE_WRIST_TRANSFER = 0.85;
    public static double OUTTAKE_WRIST_SPECIMEN_PICKUP = 0.8;
    public static double OUTTAKE_WRIST_SAMPLE_SCORE = 0.5;
    public static double OUTTAKE_WRIST_HIGH_SPECIMEN_SCORE = 0.6;
    public static double OUTTAKE_WRIST_LOW_SPECIMEN_SCORE = 0.6;
    public static double OUTTAKE_WRIST_HANG = 0.5;
    public static double OUTTAKE_CLAW_OPEN = 0.4;
    public static double OUTTAKE_CLAW_CLOSE = 0.51;
    public static int OUTTAKE_MOTOR_RETRACT = 0;
    public static int OUTTAKE_MOTOR_MAX_EXTEND = 0;
    public static int OUTTAKE_MOTOR_SAMPLE_SCORE = -800;
    public static int OUTTAKE_MOTOR_HIGH_SPECIMEN_SCORE = -400;
    public static int OUTTAKE_MOTOR_LOW_SPECIMEN_SCORE = 0;
    public static double LEFT_PTO_DISENGAGE = 0.25;
    public static double LEFT_PTO_ENGAGE = 1; // 0.46 is loose // 0.51 is still slipping
    public static double RIGHT_PTO_DISENGAGE = 0.5;
    public static double RIGHT_PTO_ENGAGE = 0.0; // 0.3 is loose, 0.2 is still too lose

    //hang
    public static int OUTTAKE_MOTOR_L2_PREPARE = -500;
    public static int DRIVE_L2_HANG = 0;
    public static int OUTTAKE_MOTOR_L3_PREPARE = 0;
    public static int DRIVE_L3_HANG = 0;
    public static int DRIVE_HANG_LOWER = 0;



    public static class intake {
        //flips intake turret
        public static void intakeTurretExtend(Servo intakeTurret){
            intakeTurret.setPosition(Robot.INTAKE_TURRET_PICKUP_STRAIGHT);

        }
        //flipse everything that isnt turret
        public static void intakeNonTurretExtend(Servo intakePivot, Servo intakeWrist, Servo intakeClaw){
            intakePivot.setPosition(Robot.INTAKE_PIVOT_PICKUP_READY);
            intakeWrist.setPosition(Robot.INTAKE_WRIST_STRAIGHT);
            intakeClaw.setPosition(Robot.INTAKE_CLAW_OPEN);

        }
        //lowers intake to pickup
        public static void intakeLower(Servo intakePivot){
            intakePivot.setPosition(Robot.INTAKE_PIVOT_PICKUP);
        }
        //goes to transfer position
        public static void intakeTransfer(Servo intakeTurret, Servo intakePivot, Servo intakeWrist){
            intakeTurret.setPosition(Robot.INTAKE_TURRET_TRANSFER);
            intakePivot.setPosition(Robot.INTAKE_PIVOT_TRANSFER);
            intakeWrist.setPosition(Robot.INTAKE_WRIST_STRAIGHT);
        }
        //drops sample on left side
        public static void intakeLeftDrop(Servo intakeTurret, Servo intakePivot, Servo intakeWrist){
            intakeTurret.setPosition(Robot.INTAKE_TURRET_DROP_LEFT);
            intakePivot.setPosition(Robot.INTAKE_PIVOT_DROP);
            intakeWrist.setPosition(Robot.INTAKE_WRIST_STRAIGHT);
        }
        //drops sample on right side
        public static void intakeRightDrop(Servo intakeTurret, Servo intakePivot, Servo intakeWrist){
            intakeTurret.setPosition(Robot.INTAKE_TURRET_DROP_RIGHT);
            intakePivot.setPosition(Robot.INTAKE_PIVOT_DROP);
            intakeWrist.setPosition(Robot.INTAKE_WRIST_STRAIGHT);
        }
        //turret pickusp o left side
        public static void intakePickupLeft(Servo intakeTurret, Servo intakePivot, Servo intakeWrist){
            intakeTurret.setPosition(Robot.INTAKE_TURRET_PICKUP_LEFT);
            intakePivot.setPosition(Robot.INTAKE_PIVOT_PICKUP_READY);
            intakeWrist.setPosition(Robot.INTAKE_WRIST_LEFT90);
        }
        //turret pickups on right side
        public static void intakePickupRight(Servo intakeTurret, Servo intakePivot, Servo intakeWrist){
            intakeTurret.setPosition(Robot.INTAKE_TURRET_PICKUP_RIGHT);
            intakePivot.setPosition(Robot.INTAKE_PIVOT_PICKUP_READY);
            intakeWrist.setPosition(Robot.INTAKE_WRIST_RIGHT90);
        }
        //opens intake claw
        public static void intakeClawOpen(Servo intakeClaw){
            intakeClaw.setPosition(Robot.INTAKE_CLAW_OPEN);
        }
        //closes intake claw
        public static void intakeClawClose(Servo intakeClaw){
            intakeClaw.setPosition(Robot.INTAKE_CLAW_CLOSE);
        }

    }

    public static class outtake {
        //transfer position
        public static void outtakeTransfer(Servo outtakePivotLeft, Servo outtakePivotRight, Servo outtakeTurret, Servo outtakeWrist, Servo outtakeClaw){
            outtakePivotLeft.setPosition(Robot.OUTTAKE_PIVOT_TRANSFER);
            outtakePivotRight.setPosition(Robot.OUTTAKE_PIVOT_TRANSFER);
            outtakeTurret.setPosition(Robot.OUTTAKE_TURRET_STRAIGHT);
            outtakeWrist.setPosition(Robot.OUTTAKE_WRIST_TRANSFER);
            outtakeClaw.setPosition(Robot.OUTTAKE_CLAW_OPEN);

        }
        //goes to sample score
        public static void outtakeSampleScore(Servo outtakePivotLeft, Servo outtakePivotRight, Servo outtakeTurret, Servo outtakeWrist){
            outtakePivotLeft.setPosition(Robot.OUTTAKE_PIVOT_SAMPLE_SCORE);
            outtakePivotRight.setPosition(Robot.OUTTAKE_PIVOT_SAMPLE_SCORE);
            outtakeTurret.setPosition(Robot.OUTTAKE_TURRET_STRAIGHT);
            outtakeWrist.setPosition(Robot.OUTTAKE_WRIST_SAMPLE_SCORE);
        }
        //goes to speicmen pickup
        public static void outtakeSpecPickup(Servo outtakePivotLeft, Servo outtakePivotRight, Servo outtakeTurret, Servo outtakeWrist, Servo outtakeClaw){
            outtakePivotLeft.setPosition(Robot.OUTTAKE_PIVOT_SPECIMEN_PICKUP);
            outtakePivotRight.setPosition(Robot.OUTTAKE_PIVOT_SPECIMEN_PICKUP);
            outtakeTurret.setPosition(Robot.OUTTAKE_TURRET_STRAIGHT);
            outtakeWrist.setPosition(Robot.OUTTAKE_WRIST_SPECIMEN_PICKUP);
            outtakeClaw.setPosition(Robot.OUTTAKE_CLAW_OPEN);
        }
        //goes to score high cshamber
        public static void outtakeHighChamber(Servo outtakePivotLeft, Servo outtakePivotRight, Servo outtakeTurret, Servo outtakeWrist){
            outtakePivotLeft.setPosition(Robot.OUTTAKE_PIVOT_HIGH_SPECIMEN_SCORE);
            outtakePivotRight.setPosition(Robot.OUTTAKE_PIVOT_HIGH_SPECIMEN_SCORE);
            outtakeTurret.setPosition(Robot.OUTTAKE_TURRET_STRAIGHT);
            outtakeWrist.setPosition(Robot.OUTTAKE_WRIST_HIGH_SPECIMEN_SCORE);
        }
        //goes to score low hcamebr
        public static void outtakeLowChamber(Servo outtakePivotLeft, Servo outtakePivotRight, Servo outtakeTurret, Servo outtakeWrist){
            outtakePivotLeft.setPosition(Robot.OUTTAKE_PIVOT_LOW_SPECIMEN_SCORE);
            outtakePivotRight.setPosition(Robot.OUTTAKE_PIVOT_LOW_SPECIMEN_SCORE);
            outtakeTurret.setPosition(Robot.OUTTAKE_TURRET_STRAIGHT);
            outtakeWrist.setPosition(Robot.OUTTAKE_WRIST_LOW_SPECIMEN_SCORE);
        }
        //scofes high hcamber on left side
        public static void outtakeLeftScore(Servo outtakeTurret){
            outtakeTurret.setPosition(Robot.OUTTAKE_TURRET_LEFT90);
        }
        //scores high chamber on right side
        public static void outtakeRightScore(Servo outtakeTurret){
            outtakeTurret.setPosition(Robot.OUTTAKE_TURRET_RIGHT90);
        }
        //opens outtake claw
        public static void outtakeClawOpen(Servo outtakeClaw){
            outtakeClaw.setPosition(Robot.OUTTAKE_CLAW_OPEN);
        }
        //clsoes outtaek calw
        public static void outtakeClawClose(Servo outtakeClaw){
            outtakeClaw.setPosition(Robot.OUTTAKE_CLAW_CLOSE);
        }


    }



}