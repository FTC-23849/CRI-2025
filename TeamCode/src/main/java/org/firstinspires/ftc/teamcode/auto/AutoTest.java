package org.firstinspires.ftc.teamcode.auto;

import com.acmerobotics.roadrunner.Pose2d;
import com.acmerobotics.roadrunner.ProfileAccelConstraint;
import com.acmerobotics.roadrunner.SequentialAction;
import com.acmerobotics.roadrunner.TrajectoryActionBuilder;
import com.acmerobotics.roadrunner.TranslationalVelConstraint;
import com.acmerobotics.roadrunner.Vector2d;
import com.acmerobotics.roadrunner.ftc.Actions;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;

import org.firstinspires.ftc.teamcode.MecanumDrive;


@com.qualcomm.robotcore.eventloop.opmode.Autonomous
public class AutoTest extends LinearOpMode {

    @Override
    public void runOpMode() {

        // Create Roadrunner Trajectories

        Pose2d startPose = new Pose2d(-7, -65, Math.toRadians(90));
        MecanumDrive drive = new MecanumDrive(hardwareMap, startPose);

        waitForStart();

        if (isStopRequested()) return;

        sleep(4);

        TrajectoryActionBuilder cycleOne = drive.actionBuilder(startPose)
                .strafeTo(new Vector2d(-7, 0))
                .strafeTo(new Vector2d(-14, 0))
                .strafeTo(new Vector2d(-7, 0))
                .strafeTo(new Vector2d(-7, -65.5));

        Actions.runBlocking(new SequentialAction(
                cycleOne.build()
        ));

        drive.updatePoseEstimate();
        drive.localizer.update();

        TrajectoryActionBuilder cycleTwo = drive.actionBuilder(drive.localizer.getPose())
                .strafeToLinearHeading(new Vector2d(-7, 0), Math.toRadians(180))
                .strafeTo(new Vector2d(-14, 0))
                .strafeTo(new Vector2d(-7, 0))
                .strafeToLinearHeading(new Vector2d(-7, -65), Math.toRadians(90));

        Actions.runBlocking(new SequentialAction(
                cycleTwo.build()
        ));

        sleep(1000);

    }

}