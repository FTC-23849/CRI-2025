package org.firstinspires.ftc.teamcode.auto;

import static com.qualcomm.robotcore.hardware.DcMotor.ZeroPowerBehavior.BRAKE;

import androidx.annotation.NonNull;

import com.acmerobotics.dashboard.telemetry.TelemetryPacket;
import com.acmerobotics.roadrunner.Action;
import com.acmerobotics.roadrunner.ParallelAction;
import com.acmerobotics.roadrunner.Pose2d;
import com.acmerobotics.roadrunner.SequentialAction;
import com.acmerobotics.roadrunner.SleepAction;
import com.acmerobotics.roadrunner.TrajectoryActionBuilder;
import com.acmerobotics.roadrunner.Vector2d;
import com.acmerobotics.roadrunner.ftc.Actions;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.teamcode.Robot;
import org.firstinspires.ftc.teamcode.roadrunner.MecanumDrive;


@com.qualcomm.robotcore.eventloop.opmode.Autonomous
public class Middle7SpecAuto extends LinearOpMode {

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

    ElapsedTime timer = new ElapsedTime();

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
                .strafeToLinearHeading(new Vector2d(-12, -25), Math.toRadians(90))
                .strafeToLinearHeading(new Vector2d(-12, -6), Math.toRadians(90));

        Actions.runBlocking(new ParallelAction(
                goToScorePreload.build(),
                new setOuttakeSlides(outtakeSlideMotor_left, outtakeSlideMotor_right, -400, 1.0)
        ));

        drive.updatePoseEstimate();
        drive.localizer.update();

        TrajectoryActionBuilder scorePreload = drive.actionBuilder(drive.localizer.getPose())
                .strafeToLinearHeading(new Vector2d(-20, -6), Math.toRadians(90));

        Actions.runBlocking(new ParallelAction(
                new specLeftScore(outtakeTurret),
                scorePreload.build()
        ));

        drive.updatePoseEstimate();
        drive.localizer.update();

        TrajectoryActionBuilder collectSpike1 = drive.actionBuilder(drive.localizer.getPose())
                .strafeToLinearHeading(new Vector2d(-12, -6), Math.toRadians(90))
                .strafeToLinearHeading(new Vector2d(-12, -35.5), Math.toRadians(90))
                .strafeToLinearHeading(new Vector2d(-7, -35.5), Math.toRadians(90));

        Actions.runBlocking(new ParallelAction(
                collectSpike1.build(),
                new setOuttakeSlides(outtakeSlideMotor_left, outtakeSlideMotor_right, 0, -1.0),
                new specPickup(intakePivot, outtakePivotLeft, outtakePivotRight, outtakeTurret, outtakeWrist, outtakeClaw),
                new setIntakeSlides(intakeMotor, 200, 1.0),
                new intakeReadyRight(intakeTurret, intakePivot, intakeWrist)
        ));

        sleep(200);
        Robot.intake.intakeLower(intakePivot);
        sleep(200);
        Robot.intake.intakeClawClose(intakeClaw);
        sleep(200);

        drive.updatePoseEstimate();
        drive.localizer.update();

        TrajectoryActionBuilder collectSpec2 = drive.actionBuilder(drive.localizer.getPose())
                .strafeToLinearHeading(new Vector2d(-12, -65), Math.toRadians(90));

        Actions.runBlocking(new ParallelAction(
                collectSpec2.build(),
                new SequentialAction(
                        new intakeRightDrop(intakePivot, intakeTurret, intakeWrist),
                        new SleepAction(0.3),
                        new setIntakeSlides(intakeMotor, 0, -1.0)
                )
        ));

        drive.updatePoseEstimate();
        drive.localizer.update();

        TrajectoryActionBuilder goToScoreSpec2 = drive.actionBuilder(drive.localizer.getPose())
                .strafeToLinearHeading(new Vector2d(-12, -8), Math.toRadians(90));

        Actions.runBlocking(new SequentialAction(
                new ParallelAction(
                        new intakeClawOpen(intakeClaw),
                        new outtakeClawClose(outtakeClaw)
                ),
                new ParallelAction(
                        goToScoreSpec2.build(),
                        new setOuttakeSlides(outtakeSlideMotor_left, outtakeSlideMotor_right, -400, 1.0),
                        new specFrontScore(outtakePivotLeft, outtakePivotRight, outtakeTurret, outtakeWrist),
                        new intakeTransfer(intakePivot, intakeTurret, intakeWrist)
                )
        ));

        drive.updatePoseEstimate();
        drive.localizer.update();

        TrajectoryActionBuilder scoreSpec2 = drive.actionBuilder(drive.localizer.getPose())
                .strafeToLinearHeading(new Vector2d(-20, -8), Math.toRadians(90));

        Actions.runBlocking(new ParallelAction(
                new specLeftScore(outtakeTurret),
                scoreSpec2.build()
        ));

        drive.updatePoseEstimate();
        drive.localizer.update();

        TrajectoryActionBuilder collectSpike2 = drive.actionBuilder(drive.localizer.getPose())
                .strafeToLinearHeading(new Vector2d(-12, -8), Math.toRadians(90))
                .strafeToLinearHeading(new Vector2d(-12, -24.5), Math.toRadians(90))
                .strafeToLinearHeading(new Vector2d(-7, -24.5), Math.toRadians(90));

        drive.updatePoseEstimate();
        drive.localizer.update();

        Actions.runBlocking(new ParallelAction(
                collectSpike2.build(),
                new setOuttakeSlides(outtakeSlideMotor_left, outtakeSlideMotor_right, 0, -1.0),
                new specPickup(intakePivot, outtakePivotLeft, outtakePivotRight, outtakeTurret, outtakeWrist, outtakeClaw),
                new setIntakeSlides(intakeMotor, 200, 1.0),
                new intakeReadyRight(intakeTurret, intakePivot, intakeWrist)
        ));

        sleep(200);
        Robot.intake.intakeLower(intakePivot);
        sleep(200);
        Robot.intake.intakeClawClose(intakeClaw);
        sleep(200);

        TrajectoryActionBuilder collectSpec3 = drive.actionBuilder(drive.localizer.getPose())
                .strafeToConstantHeading(new Vector2d(-12, -65));

        Actions.runBlocking(new ParallelAction(
                collectSpec3.build(),
                new SequentialAction(
                        new intakeRightDrop(intakePivot, intakeTurret, intakeWrist),
                        new SleepAction(0.3),
                        new setIntakeSlides(intakeMotor, 0, -1.0)
                )
        ));

        drive.updatePoseEstimate();
        drive.localizer.update();

        TrajectoryActionBuilder goToScoreSpec3 = drive.actionBuilder(drive.localizer.getPose())
                .strafeToLinearHeading(new Vector2d(-12, -10), Math.toRadians(90));

        Actions.runBlocking(new SequentialAction(
                new ParallelAction(
                        new intakeClawOpen(intakeClaw),
                        new outtakeClawClose(outtakeClaw)
                ),
                new ParallelAction(
                        goToScoreSpec3.build(),
                        new setOuttakeSlides(outtakeSlideMotor_left, outtakeSlideMotor_right, -400, 1.0),
                        new specFrontScore(outtakePivotLeft, outtakePivotRight, outtakeTurret, outtakeWrist),
                        new intakeTransfer(intakePivot, intakeTurret, intakeWrist)
                )
        ));

        drive.updatePoseEstimate();
        drive.localizer.update();

        TrajectoryActionBuilder scoreSpec3 = drive.actionBuilder(drive.localizer.getPose())
                .strafeToLinearHeading(new Vector2d(-20, -10), Math.toRadians(90));

        Actions.runBlocking(new ParallelAction(
                new specLeftScore(outtakeTurret),
                scoreSpec3.build()
        ));

        drive.updatePoseEstimate();
        drive.localizer.update();

        TrajectoryActionBuilder collectSpike3 = drive.actionBuilder(drive.localizer.getPose())
                .strafeToLinearHeading(new Vector2d(-7, -15), Math.toRadians(90));

        Actions.runBlocking(new ParallelAction(
                collectSpike3.build(),
                new setOuttakeSlides(outtakeSlideMotor_left, outtakeSlideMotor_right, 0, -1.0),
                new specPickup(intakePivot, outtakePivotLeft, outtakePivotRight, outtakeTurret, outtakeWrist, outtakeClaw),
                new setIntakeSlides(intakeMotor, 200, 1.0),
                new intakeReadyRight(intakeTurret, intakePivot, intakeWrist)
        ));

        sleep(200);
        Robot.intake.intakeLower(intakePivot);
        sleep(200);
        Robot.intake.intakeClawClose(intakeClaw);
        sleep(200);

        drive.updatePoseEstimate();
        drive.localizer.update();

        TrajectoryActionBuilder collectSpec4 = drive.actionBuilder(drive.localizer.getPose())
                .strafeToLinearHeading(new Vector2d(-12, -65), Math.toRadians(90));

        Actions.runBlocking(new ParallelAction(
                collectSpec4.build(),
                new SequentialAction(
                        new intakeRightDrop(intakePivot, intakeTurret, intakeWrist),
                        new SleepAction(0.3),
                        new setIntakeSlides(intakeMotor, 0, -1.0)
                )
        ));


        drive.updatePoseEstimate();
        drive.localizer.update();

        TrajectoryActionBuilder goToScoreSpec4 = drive.actionBuilder(drive.localizer.getPose())
                .strafeToLinearHeading(new Vector2d(-12, -12), Math.toRadians(90));

        Actions.runBlocking(new SequentialAction(
                new ParallelAction(
                        new intakeClawOpen(intakeClaw),
                        new outtakeClawClose(outtakeClaw)
                ),
                new ParallelAction(
                        goToScoreSpec4.build(),
                        new setOuttakeSlides(outtakeSlideMotor_left, outtakeSlideMotor_right, -400, 1.0),
                        new specFrontScore(outtakePivotLeft, outtakePivotRight, outtakeTurret, outtakeWrist),
                        new intakeTransfer(intakePivot, intakeTurret, intakeWrist)
                )
        ));

        drive.updatePoseEstimate();
        drive.localizer.update();

        TrajectoryActionBuilder scoreSpec4 = drive.actionBuilder(drive.localizer.getPose())
                .strafeToLinearHeading(new Vector2d(-20, -12), Math.toRadians(90));

        Actions.runBlocking(new ParallelAction(
                new specLeftScore(outtakeTurret),
                scoreSpec4.build()
        ));

        drive.updatePoseEstimate();
        drive.localizer.update();

        TrajectoryActionBuilder collectSpec5 = drive.actionBuilder(drive.localizer.getPose())
                .strafeToLinearHeading(new Vector2d(-12, -12), Math.toRadians(90))
                .strafeToLinearHeading(new Vector2d(-12, -65), Math.toRadians(90));

        Actions.runBlocking(new ParallelAction(
                collectSpec5.build()
        ));

        drive.updatePoseEstimate();
        drive.localizer.update();

        TrajectoryActionBuilder goToScoreSpec5 = drive.actionBuilder(drive.localizer.getPose())
                .strafeToLinearHeading(new Vector2d(-12, -14), Math.toRadians(90));

        drive.updatePoseEstimate();
        drive.localizer.update();


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

            outtakeSlideMotor_left.setTargetPosition(ticks);
            outtakeSlideMotor_right.setTargetPosition(ticks);

            outtakeSlideMotor_right.setMode(DcMotorEx.RunMode.RUN_TO_POSITION);
            outtakeSlideMotor_left.setMode(DcMotorEx.RunMode.RUN_TO_POSITION);

            outtakeSlideMotor_right.setPower(p);
            outtakeSlideMotor_left.setPower(p);

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

            intakeMotor.setMode(DcMotor.RunMode.RUN_TO_POSITION);

            intakeMotor.setPower(p);

            return intakeMotor.isBusy();
        }
    }

    public class specFrontScore implements Action {

        Servo outtakePivotLeft;
        Servo outtakePivotRight;
        Servo outtakeTurret;
        Servo outtakeWrist;

        public specFrontScore(Servo outtakePivotLeft, Servo outtakePivotRight, Servo outtakeTurret, Servo outtakeWrist){
            this.outtakePivotLeft = outtakePivotLeft;
            this.outtakePivotRight = outtakePivotRight;
            this.outtakeTurret = outtakeTurret;
            this.outtakeWrist = outtakeWrist;
        }

        @Override
        public boolean run(@NonNull TelemetryPacket telemetryPacket) {

            Robot.outtake.outtakeHighChamber(outtakePivotLeft, outtakePivotRight, outtakeTurret, outtakeWrist);

            return false;
        }
    }

    public class intakeRightDrop implements Action {

        Servo intakePivot;
        Servo intakeTurret;
        Servo intakeWrist;

        public intakeRightDrop(Servo intakePivot, Servo intakeTurret, Servo intakeWrist){
            this.intakePivot = intakePivot;
            this.intakeTurret = intakeTurret;
            this.intakeWrist = intakeWrist;
        }

        @Override
        public boolean run(@NonNull TelemetryPacket telemetryPacket) {

            Robot.intake.intakeRightDrop(intakeTurret, intakePivot, intakeWrist);

            return false;
        }
    }

    public class intakeTransfer implements Action {

        Servo intakeTurret;
        Servo intakePivot;
        Servo intakeWrist;

        public intakeTransfer(Servo intakePivot, Servo intakeTurret, Servo intakeWrist){
            this.intakeTurret = intakeTurret;
            this.intakePivot = intakePivot;
            this.intakeWrist = intakeWrist;
        }

        @Override
        public boolean run(@NonNull TelemetryPacket telemetryPacket) {

            Robot.intake.intakeTransfer(intakeTurret, intakePivot, intakeWrist);

            return false;
        }
    }

    public class specLeftScore implements Action {

        Servo outtakeTurret;

        public specLeftScore(Servo outtakeTurret){
            this.outtakeTurret = outtakeTurret;
        }

        @Override
        public boolean run(@NonNull TelemetryPacket telemetryPacket) {

            Robot.outtake.outtakeLeftScore(outtakeTurret);

            return false;
        }
    }

    public class outtakeClawClose implements Action {

        Servo outtakeClaw;

        public outtakeClawClose(Servo outtakeClaw){
            this.outtakeClaw = outtakeClaw;
        }

        @Override
        public boolean run(@NonNull TelemetryPacket telemetryPacket) {

            Robot.outtake.outtakeClawClose(outtakeClaw);

            return false;
        }
    }

    public class waitMilliseconds implements Action {

        ElapsedTime ElapsedTime;
        int milliseconds;

        public waitMilliseconds(ElapsedTime timer, int time){
            this.ElapsedTime = timer;
            this.milliseconds = time;
        }

        @Override
        public boolean run(@NonNull TelemetryPacket telemetryPacket) {

            this.ElapsedTime.reset();

            return this.ElapsedTime.milliseconds() >= this.milliseconds;
        }
    }

    public class outtakeClawOpen implements Action {

        Servo outtakeClaw;

        public outtakeClawOpen(Servo outtakeClaw){
            this.outtakeClaw = outtakeClaw;
        }

        @Override
        public boolean run(@NonNull TelemetryPacket telemetryPacket) {

            Robot.outtake.outtakeClawOpen(outtakeClaw);

            return false;
        }
    }

    public class specPickup implements Action {

        Servo intakePivot;
        Servo outtakePivotLeft;
        Servo outtakePivotRight;
        Servo outtakeTurret;
        Servo outtakeWrist;
        Servo outtakeClaw;

        public specPickup(Servo intakePivot, Servo outtakePivotLeft, Servo outtakePivotRight, Servo outtakeTurret, Servo outtakeWrist, Servo outtakeClaw){
            this.intakePivot = intakePivot;
            this.outtakePivotLeft = outtakePivotLeft;
            this.outtakePivotRight = outtakePivotRight;
            this.outtakeTurret = outtakeTurret;
            this.outtakeWrist = outtakeWrist;
            this.outtakeClaw = outtakeClaw;
        }

        @Override
        public boolean run(@NonNull TelemetryPacket telemetryPacket) {

            Robot.outtake.outtakeSpecPickup(outtakePivotLeft, outtakePivotRight, outtakeTurret, outtakeWrist, outtakeClaw);

            return false;
        }
    }

    public class sampleScore implements Action {

        Servo outtakePivotLeft;
        Servo outtakePivotRight;
        Servo outtakeTurret;
        Servo outtakeWrist;

        public sampleScore(Servo outtakePivotLeft, Servo outtakePivotRight, Servo outtakeTurret, Servo outtakeWrist){
            this.outtakePivotLeft = outtakePivotLeft;
            this.outtakePivotRight = outtakePivotRight;
            this.outtakeTurret = outtakeTurret;
            this.outtakeWrist = outtakeWrist;
        }

        @Override
        public boolean run(@NonNull TelemetryPacket telemetryPacket) {

            Robot.outtake.outtakeSampleScore(outtakePivotLeft, outtakePivotRight, outtakeTurret, outtakeWrist);

            return false;
        }
    }

    public class outtakeTransfer implements Action {

        Servo outtakePivotLeft;
        Servo outtakePivotRight;
        Servo outtakeTurret;
        Servo outtakeWrist;
        Servo outtakeClaw;

        public outtakeTransfer(Servo outtakePivotLeft, Servo outtakePivotRight, Servo outtakeTurret, Servo outtakeWrist, Servo outtakeClaw){
            this.outtakePivotLeft = outtakePivotLeft;
            this.outtakePivotRight = outtakePivotRight;
            this.outtakeTurret = outtakeTurret;
            this.outtakeWrist = outtakeWrist;
            this.outtakeClaw = outtakeClaw;
        }

        @Override
        public boolean run(@NonNull TelemetryPacket telemetryPacket) {

            Robot.outtake.outtakeTransfer(outtakePivotLeft, outtakePivotRight, outtakeTurret, outtakeWrist, outtakeClaw);

            return false;
        }
    }

    public class intakeClawOpen implements Action {

        Servo intakeClaw;

        public intakeClawOpen(Servo intakeClaw){
            this.intakeClaw = intakeClaw;
        }

        @Override
        public boolean run(@NonNull TelemetryPacket telemetryPacket) {

            Robot.intake.intakeClawOpen(intakeClaw);

            return false;
        }
    }

    public class outtakeLeft implements Action {

        Servo outtakeTurret;

        public outtakeLeft(Servo outtakeTurret){
            this.outtakeTurret = outtakeTurret;
        }

        @Override
        public boolean run(@NonNull TelemetryPacket telemetryPacket) {

            outtakeTurret.setPosition(0.25);

            return false;
        }
    }

    public class outtakeRight implements Action {

        Servo outtakeTurret;

        public outtakeRight(Servo outtakeTurret){
            this.outtakeTurret = outtakeTurret;
        }

        @Override
        public boolean run(@NonNull TelemetryPacket telemetryPacket) {

            outtakeTurret.setPosition(0.78);

            return false;
        }
    }

    public class setWrist implements Action {

        Servo intakeWrist;
        double position;

        public setWrist(Servo intakeWrist, double pos){
            this.intakeWrist = intakeWrist;
            this.position = pos;
        }

        @Override
        public boolean run(@NonNull TelemetryPacket telemetryPacket) {

            intakeWrist.setPosition(position);

            return false;
        }
    }

    public class intakeRotateWristLeft implements Action {

        Servo intakeClaw;

        public intakeRotateWristLeft(Servo intakeClaw){
            this.intakeClaw = intakeClaw;
        }

        @Override
        public boolean run(@NonNull TelemetryPacket telemetryPacket) {

            intakeWrist.setPosition(Robot.INTAKE_WRIST_LEFT90);

            return false;
        }
    }

    public class intakeAvoid implements Action {

        Servo intakePivot;

        public intakeAvoid(Servo intakePivot){
            this.intakePivot = intakePivot;
        }

        @Override
        public boolean run(@NonNull TelemetryPacket telemetryPacket) {

            intakePivot.setPosition(Robot.INTAKE_PIVOT_AVOID);

            return false;
        }
    }

    public class intakeReady implements Action {

        Servo intakePivot;
        Servo intakeWrist;
        Servo intakeClaw;
        Servo intakeTurret;

        public intakeReady(Servo pivot, Servo wrist, Servo claw, Servo turret){
            this.intakePivot = pivot;
            this.intakeWrist = wrist;
            this.intakeClaw = claw;
            this.intakeTurret = turret;
        }

        @Override
        public boolean run(@NonNull TelemetryPacket telemetryPacket) {

            Robot.intake.intakeTurretExtend(intakeTurret);
            Robot.intake.intakeNonTurretExtend(intakePivot, intakeWrist, intakeClaw);

            return false;
        }
    }

    public class intakeReadyRight implements Action {

        Servo intakeTurret;
        Servo intakePivot;
        Servo intakeWrist;

        public intakeReadyRight(Servo turret, Servo pivot, Servo wrist){
            this.intakeTurret = turret;
            this.intakePivot = pivot;
            this.intakeWrist = wrist;
        }

        @Override
        public boolean run(@NonNull TelemetryPacket telemetryPacket) {

            Robot.intake.intakePickupRight(intakeTurret, intakePivot, intakeWrist);
            intakeWrist.setPosition(Robot.INTAKE_WRIST_STRAIGHT);

            return false;
        }
    }

}