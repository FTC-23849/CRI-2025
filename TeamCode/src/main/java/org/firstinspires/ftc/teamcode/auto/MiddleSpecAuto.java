package org.firstinspires.ftc.teamcode.auto;

import com.acmerobotics.roadrunner.Pose2d;
import com.acmerobotics.roadrunner.SequentialAction;
import com.acmerobotics.roadrunner.TrajectoryActionBuilder;
import com.acmerobotics.roadrunner.Vector2d;
import com.acmerobotics.roadrunner.ftc.Actions;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;

import org.firstinspires.ftc.teamcode.roadrunner.MecanumDrive;


@com.qualcomm.robotcore.eventloop.opmode.Autonomous
public class MiddleSpecAuto extends LinearOpMode {

    @Override
    public void runOpMode() {

        // Create Roadrunner Trajectories

        Pose2d startPose = new Pose2d(-7, -65, Math.toRadians(90));
        MecanumDrive drive = new MecanumDrive(hardwareMap, startPose);

        waitForStart();

        if (isStopRequested()) return;

        sleep(4);

        TrajectoryActionBuilder scorePreload = drive.actionBuilder(startPose)
                .strafeTo(new Vector2d(-13, -25))
                .strafeTo(new Vector2d(-15, -6));

        Actions.runBlocking(new SequentialAction(
                scorePreload.build()
        ));

        drive.updatePoseEstimate();
        drive.localizer.update();

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

        TrajectoryActionBuilder collectSpec2 = drive.actionBuilder(drive.localizer.getPose())
                .strafeTo(new Vector2d(-15, -32))
                .strafeTo(new Vector2d(-7, -32));

        Actions.runBlocking(new SequentialAction(
                collectSpec1.build()
        ));

        drive.updatePoseEstimate();
        drive.localizer.update();

        TrajectoryActionBuilder depositSpec2 = drive.actionBuilder(drive.localizer.getPose())
                .strafeTo(new Vector2d(-7, -65));

        Actions.runBlocking(new SequentialAction(
                depositSpec1.build()
        ));


        sleep(1000);

    }

}