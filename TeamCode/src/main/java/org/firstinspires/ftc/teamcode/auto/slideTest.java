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

import org.firstinspires.ftc.teamcode.Robot;
import org.firstinspires.ftc.teamcode.roadrunner.MecanumDrive;


@com.qualcomm.robotcore.eventloop.opmode.Autonomous
public class slideTest extends LinearOpMode {

    DcMotorEx outtakeSlideMotor_left;
    DcMotorEx outtakeSlideMotor_right;

    @Override
    public void runOpMode() {

        //map motors and servos
        outtakeSlideMotor_left = hardwareMap.get(DcMotorEx.class,"outtakeSlideMotor_left");
        outtakeSlideMotor_right = hardwareMap.get(DcMotorEx.class,"outtakeSlideMotor_right");
        outtakeSlideMotor_right.setDirection(DcMotorEx.Direction.REVERSE);

        outtakeSlideMotor_left.setZeroPowerBehavior(BRAKE);
        outtakeSlideMotor_right.setZeroPowerBehavior(BRAKE);

        outtakeSlideMotor_right.setMode(DcMotorEx.RunMode.STOP_AND_RESET_ENCODER);
        outtakeSlideMotor_left.setMode(DcMotorEx.RunMode.STOP_AND_RESET_ENCODER);

        outtakeSlideMotor_right.setMode(DcMotorEx.RunMode.RUN_TO_POSITION);
        outtakeSlideMotor_left.setMode(DcMotorEx.RunMode.RUN_TO_POSITION);

        waitForStart();

        if (isStopRequested()) return;

        sleep(4);

        //score preload
        outtakeSlideMotor_left.setTargetPosition(-400);
        outtakeSlideMotor_right.setTargetPosition(-400);

        outtakeSlideMotor_left.setPower(1.0);
        outtakeSlideMotor_right.setPower(1.0);

        while (outtakeSlideMotor_right.isBusy() && outtakeSlideMotor_left.isBusy()) {

        }

        outtakeSlideMotor_left.setPower(0);
        outtakeSlideMotor_right.setPower(0);

        sleep(1000);

    }

}