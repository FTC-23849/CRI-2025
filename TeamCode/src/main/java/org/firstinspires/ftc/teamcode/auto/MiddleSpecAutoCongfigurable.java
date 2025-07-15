package org.firstinspires.ftc.teamcode.auto;

import static com.qualcomm.robotcore.hardware.DcMotor.ZeroPowerBehavior.BRAKE;

import androidx.annotation.NonNull;

import com.acmerobotics.dashboard.telemetry.TelemetryPacket;
import com.acmerobotics.roadrunner.Action;
import com.acmerobotics.roadrunner.ParallelAction;
import com.acmerobotics.roadrunner.Pose2d;
import com.acmerobotics.roadrunner.SequentialAction;
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
public class MiddleSpecAutoCongfigurable extends LinearOpMode {

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

    int specNumber = 2;
    String pickupSpikeMark = "no";

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

        while (!opModeIsActive() && !isStopRequested()) {
            if (gamepad1.dpad_down) {
                specNumber = 2;
                telemetry.update();
            }

            if (gamepad1.dpad_up) {
                specNumber = 4;
                telemetry.update();
            }

            if (gamepad1.dpad_right) {
                specNumber = 5;
                telemetry.update();
            }

            if (gamepad1.a) {
                pickupSpikeMark = "yes";
                telemetry.update();
            }

            if (gamepad1.b) {
                pickupSpikeMark = "no";
                telemetry.update();
            }

            telemetry.addData("number of specs: dpad_up - 4     dpad_down - 2      dpad_right - 5", specNumber);
            telemetry.addData("pick up spike mark? a - yes     b - no", pickupSpikeMark);
            telemetry.update();
        }


        waitForStart();

        if (isStopRequested()) return;

        sleep(4);

        if (specNumber == 5) {

        } else if (pickupSpikeMark == "no") {

            if (specNumber == 2) {

                //score preload
                TrajectoryActionBuilder goToScorePreload = drive.actionBuilder(startPose)
                        .strafeToConstantHeading(new Vector2d(-10, -25));

                Actions.runBlocking(new ParallelAction(
                        goToScorePreload.build(),
                        new setOuttakeSlides(outtakeSlideMotor_left, outtakeSlideMotor_right, -400, 1.0)
                ));

                drive.updatePoseEstimate();
                drive.localizer.update();

                TrajectoryActionBuilder scorePreload = drive.actionBuilder(drive.localizer.getPose())
                        .strafeToConstantHeading(new Vector2d(-21, 5));

                Actions.runBlocking(new ParallelAction(
                        new specLeftScore(outtakeTurret),
                        scorePreload.build()
                ));

                drive.updatePoseEstimate();
                drive.localizer.update();

                TrajectoryActionBuilder collectSpec2 = drive.actionBuilder(drive.localizer.getPose())
                        .strafeToConstantHeading(new Vector2d(-12, 0))
                        .strafeToConstantHeading(new Vector2d(-12, -65));

                Actions.runBlocking(new ParallelAction(
                        collectSpec2.build(),
                        new outtakeClawOpen(outtakeClaw),
                        new specPickup(intakePivot, outtakePivotLeft, outtakePivotRight, outtakeTurret, outtakeWrist, outtakeClaw),
                        new setOuttakeSlides(outtakeSlideMotor_left, outtakeSlideMotor_right, 0, -1.0)
                ));

                drive.updatePoseEstimate();
                drive.localizer.update();

                //score second spec
                TrajectoryActionBuilder goToScoreSpec2 = drive.actionBuilder(drive.localizer.getPose())
                        .strafeToConstantHeading(new Vector2d(-12, -2));

                Actions.runBlocking(new SequentialAction(
                        new outtakeClawClose(outtakeClaw),
                        new ParallelAction(
                                goToScoreSpec2.build(),
                                new setOuttakeSlides(outtakeSlideMotor_left, outtakeSlideMotor_right, -400, 1.0),
                                new specFrontScore(outtakePivotLeft, outtakePivotRight, outtakeTurret, outtakeWrist)
                        )
                ));

                drive.updatePoseEstimate();
                drive.localizer.update();

                TrajectoryActionBuilder scoreSpec2 = drive.actionBuilder(drive.localizer.getPose())
                        .strafeToConstantHeading(new Vector2d(-21, -2));

                Actions.runBlocking(new SequentialAction(
                        new ParallelAction(
                                scoreSpec2.build(),
                                new specLeftScore(outtakeTurret)
                        )
                ));

                drive.updatePoseEstimate();
                drive.localizer.update();


                TrajectoryActionBuilder park = drive.actionBuilder(drive.localizer.getPose())
                        .strafeToConstantHeading(new Vector2d(-12, -2))
                        .strafeToConstantHeading(new Vector2d(-12, -60));

                Actions.runBlocking(new ParallelAction(
                        park.build(),
                        new setOuttakeSlides(outtakeSlideMotor_left, outtakeSlideMotor_right, 0, -1.0),
                        new specPickup(intakePivot, outtakePivotLeft, outtakePivotRight, outtakeTurret, outtakeWrist, outtakeClaw)
                ));

            } else if (specNumber == 4) {

                //score preload
                TrajectoryActionBuilder goToScorePreload = drive.actionBuilder(startPose)
                        .strafeToConstantHeading(new Vector2d(-10, -25));

                Actions.runBlocking(new ParallelAction(
                        goToScorePreload.build(),
                        new setOuttakeSlides(outtakeSlideMotor_left, outtakeSlideMotor_right, -400, 1.0)
                ));

                drive.updatePoseEstimate();
                drive.localizer.update();

                TrajectoryActionBuilder scorePreload = drive.actionBuilder(drive.localizer.getPose())
                        .strafeToConstantHeading(new Vector2d(-21, 5));

                Actions.runBlocking(new ParallelAction(
                        new specLeftScore(outtakeTurret),
                        scorePreload.build()
                ));

                drive.updatePoseEstimate();
                drive.localizer.update();

                TrajectoryActionBuilder collectSpec2 = drive.actionBuilder(drive.localizer.getPose())
                        .strafeToConstantHeading(new Vector2d(-12, 0))
                        .strafeToConstantHeading(new Vector2d(-12, -65));

                Actions.runBlocking(new ParallelAction(
                        collectSpec2.build(),
                        new outtakeClawOpen(outtakeClaw),
                        new specPickup(intakePivot, outtakePivotLeft, outtakePivotRight, outtakeTurret, outtakeWrist, outtakeClaw),
                        new setOuttakeSlides(outtakeSlideMotor_left, outtakeSlideMotor_right, 0, -1.0)
                ));

                drive.updatePoseEstimate();
                drive.localizer.update();

                //score second spec
                TrajectoryActionBuilder goToScoreSpec2 = drive.actionBuilder(drive.localizer.getPose())
                        .strafeToConstantHeading(new Vector2d(-12, -2));

                Actions.runBlocking(new SequentialAction(
                        new outtakeClawClose(outtakeClaw),
                        new ParallelAction(
                                goToScoreSpec2.build(),
                                new setOuttakeSlides(outtakeSlideMotor_left, outtakeSlideMotor_right, -400, 1.0),
                                new specFrontScore(outtakePivotLeft, outtakePivotRight, outtakeTurret, outtakeWrist)
                        )
                ));

                drive.updatePoseEstimate();
                drive.localizer.update();

                TrajectoryActionBuilder scoreSpec2 = drive.actionBuilder(drive.localizer.getPose())
                        .strafeToConstantHeading(new Vector2d(-21, -2));

                Actions.runBlocking(new SequentialAction(
                        new ParallelAction(
                                scoreSpec2.build(),
                                new specLeftScore(outtakeTurret)
                        )
                ));

                drive.updatePoseEstimate();
                drive.localizer.update();

                //score spec 3

                TrajectoryActionBuilder collectSpec3 = drive.actionBuilder(drive.localizer.getPose())
                        .strafeToConstantHeading(new Vector2d(-12, -2))
                        .strafeToConstantHeading(new Vector2d(-12, -65));

                Actions.runBlocking(new ParallelAction(
                        collectSpec3.build(),
                        new outtakeClawOpen(outtakeClaw),
                        new specPickup(intakePivot, outtakePivotLeft, outtakePivotRight, outtakeTurret, outtakeWrist, outtakeClaw),
                        new setOuttakeSlides(outtakeSlideMotor_left, outtakeSlideMotor_right, 0, -1.0)
                ));

                drive.updatePoseEstimate();
                drive.localizer.update();

                //score second spec
                TrajectoryActionBuilder goToScoreSpec3 = drive.actionBuilder(drive.localizer.getPose())
                        .strafeToConstantHeading(new Vector2d(-12, -4));

                Actions.runBlocking(new SequentialAction(
                        new outtakeClawClose(outtakeClaw),
                        new ParallelAction(
                                goToScoreSpec3.build(),
                                new setOuttakeSlides(outtakeSlideMotor_left, outtakeSlideMotor_right, -400, 1.0),
                                new specFrontScore(outtakePivotLeft, outtakePivotRight, outtakeTurret, outtakeWrist)
                        )
                ));

                drive.updatePoseEstimate();
                drive.localizer.update();

                TrajectoryActionBuilder scoreSpec3 = drive.actionBuilder(drive.localizer.getPose())
                        .strafeToConstantHeading(new Vector2d(-21, -4));

                Actions.runBlocking(new SequentialAction(
                        new ParallelAction(
                                scoreSpec3.build(),
                                new specLeftScore(outtakeTurret)
                        )
                ));

                drive.updatePoseEstimate();
                drive.localizer.update();

                //score spec 4

                TrajectoryActionBuilder collectSpec4 = drive.actionBuilder(drive.localizer.getPose())
                        .strafeToConstantHeading(new Vector2d(-12, -4))
                        .strafeToConstantHeading(new Vector2d(-12, -65));

                Actions.runBlocking(new ParallelAction(
                        collectSpec4.build(),
                        new outtakeClawOpen(outtakeClaw),
                        new specPickup(intakePivot, outtakePivotLeft, outtakePivotRight, outtakeTurret, outtakeWrist, outtakeClaw),
                        new setOuttakeSlides(outtakeSlideMotor_left, outtakeSlideMotor_right, 0, -1.0)
                ));

                drive.updatePoseEstimate();
                drive.localizer.update();

                //score second spec
                TrajectoryActionBuilder goToScoreSpec4 = drive.actionBuilder(drive.localizer.getPose())
                        .strafeToConstantHeading(new Vector2d(-12, -6));

                Actions.runBlocking(new SequentialAction(
                        new outtakeClawClose(outtakeClaw),
                        new ParallelAction(
                                goToScoreSpec4.build(),
                                new specFrontScore(outtakePivotLeft, outtakePivotRight, outtakeTurret, outtakeWrist)
                        )
                ));

                drive.updatePoseEstimate();
                drive.localizer.update();

                TrajectoryActionBuilder scoreSpec4 = drive.actionBuilder(drive.localizer.getPose())
                        .strafeToConstantHeading(new Vector2d(-21, -6));

                Actions.runBlocking(new SequentialAction(
                        new ParallelAction(
                                scoreSpec4.build(),
                                new specLeftScore(outtakeTurret)
                        )
                ));

                drive.updatePoseEstimate();
                drive.localizer.update();


                TrajectoryActionBuilder park = drive.actionBuilder(drive.localizer.getPose())
                        .strafeToConstantHeading(new Vector2d(-12, -6))
                        .strafeToConstantHeading(new Vector2d(-12, -60));

                Actions.runBlocking(new ParallelAction(
                        park.build(),
                        new specPickup(intakePivot, outtakePivotLeft, outtakePivotRight, outtakeTurret, outtakeWrist, outtakeClaw)
                ));

            }
        }

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


}