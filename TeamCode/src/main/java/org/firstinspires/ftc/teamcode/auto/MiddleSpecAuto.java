package org.firstinspires.ftc.teamcode.auto;

import static com.qualcomm.robotcore.hardware.DcMotor.ZeroPowerBehavior.BRAKE;

import androidx.annotation.NonNull;

import com.acmerobotics.dashboard.telemetry.TelemetryPacket;
import com.acmerobotics.roadrunner.Action;
import com.acmerobotics.roadrunner.Pose2d;
import com.acmerobotics.roadrunner.SequentialAction;
import com.acmerobotics.roadrunner.TrajectoryActionBuilder;
import com.acmerobotics.roadrunner.Vector2d;
import com.acmerobotics.roadrunner.ftc.Actions;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.Servo;

import org.firstinspires.ftc.teamcode.Robot;
import org.firstinspires.ftc.teamcode.roadrunner.MecanumDrive;


@com.qualcomm.robotcore.eventloop.opmode.Autonomous
public class MiddleSpecAuto extends LinearOpMode {

    DcMotorEx outtakeSlideMotor_left;
    DcMotorEx outtakeSlideMotor_right;

    DcMotorEx intakeMotor;
    Servo intakeTurret;
    Servo intakePivot;
    Servo intakeWrist;
    Servo intakeClaw;
    Servo outtakePivotLeft;
    Servo outtakePivotRight;
    Servo outtakeTurret;
    Servo outtakeWrist;
    Servo outtakeClaw;
    Servo leftPto;
    Servo rightPto;

    @Override
    public void runOpMode() {

        // Create Roadrunner Trajectories

        Pose2d startPose = new Pose2d(-30.5, -65, Math.toRadians(90));
        MecanumDrive drive = new MecanumDrive(hardwareMap, startPose);

        //map motors and servos
        outtakeSlideMotor_left = hardwareMap.get(DcMotorEx.class,"outtakeSlideMotor_left");
        outtakeSlideMotor_right = hardwareMap.get(DcMotorEx.class,"outtakeSlideMotor_right");
        outtakeSlideMotor_right.setDirection(DcMotorEx.Direction.REVERSE);

        outtakeSlideMotor_left.setZeroPowerBehavior(BRAKE);
        outtakeSlideMotor_right.setZeroPowerBehavior(BRAKE);

        outtakeSlideMotor_right.setMode(DcMotorEx.RunMode.STOP_AND_RESET_ENCODER);
        outtakeSlideMotor_left.setMode(DcMotorEx.RunMode.STOP_AND_RESET_ENCODER);

        intakeMotor = hardwareMap.get(DcMotorEx.class, "intakeMotor");
        intakeTurret = hardwareMap.get(Servo.class, "intakeTurret");
        intakePivot = hardwareMap.get(Servo.class, "intakePivot");
        intakeWrist = hardwareMap.get(Servo.class, "intakeWrist");
        intakeClaw = hardwareMap.get(Servo.class, "intakeClaw");
        outtakePivotLeft = hardwareMap.get(Servo.class, "outtakePivotLeft");
        outtakePivotRight = hardwareMap.get(Servo.class, "outtakePivotRight");
        outtakeTurret = hardwareMap.get(Servo.class, "outtakeTurret");
        outtakeWrist = hardwareMap.get(Servo.class, "outtakeWrist");
        outtakeClaw = hardwareMap.get(Servo.class, "outtakeClaw");
        leftPto = hardwareMap.get(Servo.class, "leftPto");
        rightPto = hardwareMap.get(Servo.class, "rightPto");

        intakeMotor.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        intakeMotor.setDirection(DcMotorEx.Direction.REVERSE);

        intakePivot.setPosition(Robot.INTAKE_PIVOT_TRANSFER);
        intakeWrist.setPosition(Robot.INTAKE_WRIST_STRAIGHT);
        intakeTurret.setPosition(Robot.INTAKE_TURRET_TRANSFER);
        intakeClaw.setPosition(Robot.INTAKE_CLAW_OPEN);
        outtakePivotLeft.setPosition(Robot.OUTTAKE_PIVOT_HIGH_SPECIMEN_SCORE);
        outtakePivotRight.setPosition(Robot.OUTTAKE_PIVOT_HIGH_SPECIMEN_SCORE);
        outtakeTurret.setPosition(Robot.OUTTAKE_TURRET_STRAIGHT);
        outtakeWrist.setPosition(Robot.OUTTAKE_WRIST_HIGH_SPECIMEN_SCORE);
        outtakeClaw.setPosition(Robot.OUTTAKE_CLAW_CLOSE);
        leftPto.setPosition(Robot.LEFT_PTO_DISENGAGE);
        rightPto.setPosition(Robot.RIGHT_PTO_DISENGAGE);

        waitForStart();

        if (isStopRequested()) return;

        sleep(4);

        //score preload
        TrajectoryActionBuilder goToScorePreload = drive.actionBuilder(startPose)
                .strafeTo(new Vector2d(-13, -25));

        Actions.runBlocking(new SequentialAction(
                goToScorePreload.build()
        ));

        drive.updatePoseEstimate();
        drive.localizer.update();

        TrajectoryActionBuilder scorePreload = drive.actionBuilder(startPose)
                .strafeTo(new Vector2d(-15, -6));

        Actions.runBlocking(new SequentialAction(
                scorePreload.build()
        ));

        drive.updatePoseEstimate();
        drive.localizer.update();

        //pick up first spikemark and side deposit
        TrajectoryActionBuilder collectSpec1 = drive.actionBuilder(drive.localizer.getPose())
                .strafeTo(new Vector2d(-15, -32))
                .strafeTo(new Vector2d(-7, -32));

        Actions.runBlocking(new SequentialAction(
                collectSpec1.build()
        ));

        drive.updatePoseEstimate();
        drive.localizer.update();

        TrajectoryActionBuilder depositSpec1 = drive.actionBuilder(drive.localizer.getPose())
                .strafeTo(new Vector2d(-7, -65));

        Actions.runBlocking(new SequentialAction(
                depositSpec1.build()
        ));

        drive.updatePoseEstimate();
        drive.localizer.update();

        //pick up second spike mark and side deposit
        TrajectoryActionBuilder collectSpec2 = drive.actionBuilder(drive.localizer.getPose())
                .strafeTo(new Vector2d(-7, -35));

        Actions.runBlocking(new SequentialAction(
                collectSpec2.build()
        ));

        drive.updatePoseEstimate();
        drive.localizer.update();

        TrajectoryActionBuilder depositSpec2 = drive.actionBuilder(drive.localizer.getPose())
                .strafeTo(new Vector2d(-7, -65));

        Actions.runBlocking(new SequentialAction(
                depositSpec2.build()
        ));

        drive.updatePoseEstimate();
        drive.localizer.update();

        //pick up third spike mark and side deposit
        TrajectoryActionBuilder collectSpec3 = drive.actionBuilder(drive.localizer.getPose())
                .strafeTo(new Vector2d(-7, -25));

        Actions.runBlocking(new SequentialAction(
                collectSpec3.build()
        ));

        drive.updatePoseEstimate();
        drive.localizer.update();

        TrajectoryActionBuilder depositSpec3 = drive.actionBuilder(drive.localizer.getPose())
                .strafeTo(new Vector2d(-7, -65));

        Actions.runBlocking(new SequentialAction(
                depositSpec3.build()
        ));

        drive.updatePoseEstimate();
        drive.localizer.update();


        //score second spec
        TrajectoryActionBuilder cycleSpecScore = drive.actionBuilder(drive.localizer.getPose())
                .strafeTo(new Vector2d(-14, 0));

        Actions.runBlocking(new SequentialAction(
                cycleSpecScore.build()
        ));

        drive.updatePoseEstimate();
        drive.localizer.update();


        TrajectoryActionBuilder cycleSpecCollect = drive.actionBuilder(drive.localizer.getPose())
                .strafeTo(new Vector2d(-14, 0));

        Actions.runBlocking(new SequentialAction(
                cycleSpecCollect.build()
        ));

        drive.updatePoseEstimate();
        drive.localizer.update();

        //score 3rd spec
        TrajectoryActionBuilder cycleSpecScore2 = drive.actionBuilder(drive.localizer.getPose())
                .strafeTo(new Vector2d(-14, 0));

        Actions.runBlocking(new SequentialAction(
                cycleSpecScore2.build()
        ));

        drive.updatePoseEstimate();
        drive.localizer.update();


        TrajectoryActionBuilder cycleSpecCollect2 = drive.actionBuilder(drive.localizer.getPose())
                .strafeTo(new Vector2d(-14, 0));

        Actions.runBlocking(new SequentialAction(
                cycleSpecCollect2.build()
        ));

        drive.updatePoseEstimate();
        drive.localizer.update();

        // score 4th spec

        TrajectoryActionBuilder cycleSpecScore3 = drive.actionBuilder(drive.localizer.getPose())
                .strafeTo(new Vector2d(-14, 0));

        Actions.runBlocking(new SequentialAction(
                cycleSpecScore3.build()
        ));

        drive.updatePoseEstimate();
        drive.localizer.update();


        TrajectoryActionBuilder cycleSpecCollect3 = drive.actionBuilder(drive.localizer.getPose())
                .strafeTo(new Vector2d(-14, 0));

        Actions.runBlocking(new SequentialAction(
                cycleSpecCollect3.build()
        ));





        sleep(1000);

    }

    public class setOuttakeSlides implements Action {

        DcMotorEx outtakeSlideMotor_left;
        DcMotorEx outtakeSlideMotor_right;
        int ticks;
        double p;

        public setOuttakeSlides(DcMotorEx left, DcMotorEx right, int ticks, double power){
            this.outtakeSlideMotor_left = left;
            this.outtakeSlideMotor_right = right;
            this.ticks = ticks;
            this.p = power;
        }

        @Override
        public boolean run(@NonNull TelemetryPacket telemetryPacket) {

            outtakeSlideMotor_left.setTargetPosition(-ticks);
            outtakeSlideMotor_right.setTargetPosition(-ticks);

            outtakeSlideMotor_right.setPower(-p);
            outtakeSlideMotor_left.setPower(-p);

            return outtakeSlideMotor_right.isBusy() && outtakeSlideMotor_left.isBusy();
        }
    }

    public class setIntakeSlides implements Action {

        DcMotorEx intakeMotor;
        int ticks;
        double p;

        public setIntakeSlides(DcMotorEx intake, int ticks, double power){
            this.intakeMotor = intake;
            this.ticks = ticks;
            this.p = power;
        }

        @Override
        public boolean run(@NonNull TelemetryPacket telemetryPacket) {

            intakeMotor.setTargetPosition(ticks);

            intakeMotor.setPower(p);

            return intakeMotor.isBusy();
        }
    }


}