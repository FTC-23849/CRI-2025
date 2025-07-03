/*
Copyright 2023 FIRST Tech Challenge Team FTC

Permission is hereby granted, free of charge, to any person obtaining a copy of this software and
associated documentation files (the "Software"), to deal in the Software without restriction,
including without limitation the rights to use, copy, modify, merge, publish, distribute,
sublicense, and/or sell copies of the Software, and to permit persons to whom the Software is
furnished to do so, subject to the following conditions:

The above copyright notice and this permission notice shall be included in all copies or substantial
portions of the Software.

THE SOFTWARE IS PROVIDED "AS IS", WITHOUT WARRANTY OF ANY KIND, EXPRESS OR IMPLIED, INCLUDING BUT
NOT LIMITED TO THE WARRANTIES OF MERCHANTABILITY, FITNESS FOR A PARTICULAR PURPOSE AND
NONINFRINGEMENT. IN NO EVENT SHALL THE AUTHORS OR COPYRIGHT HOLDERS BE LIABLE FOR ANY CLAIM,
DAMAGES OR OTHER LIABILITY, WHETHER IN AN ACTION OF CONTRACT, TORT OR OTHERWISE, ARISING FROM,
OUT OF OR IN CONNECTION WITH THE SOFTWARE OR THE USE OR OTHER DEALINGS IN THE SOFTWARE.
*/
package org.firstinspires.ftc.teamcode.teleop;

import static com.qualcomm.robotcore.hardware.DcMotor.ZeroPowerBehavior.BRAKE;

import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.Servo;

import org.firstinspires.ftc.teamcode.Robot;

import com.qualcomm.robotcore.util.ElapsedTime;



@TeleOp(name = "DriverControl", group = "stuff")

public class DriverControl extends OpMode {
    /* Declare OpMode members. */
    DcMotorEx leftFrontMotor;
    DcMotorEx rightFrontMotor;
    DcMotorEx leftBackMotor;
    DcMotorEx rightBackMotor;

    DcMotorEx outtakeSlideMotor_left;
    DcMotorEx outtakeSlideMotor_right;

    DcMotorEx intakeMotor;
    Servo intakeTurret;
    Servo intakePivot;
    Servo intakeWrist;
    Servo intakeClaw;


    boolean intakeRetracted = true;
    boolean intakeClawClosed = false;
    int scoringCycle = 0;
    int intakeCycle = 0;
    boolean rightBumperPressed = false;
    int backTimes = 0;
    boolean scoringSpec = true;
    boolean backPressed = false;
    int intakePickupPosition = 2;
    boolean leftBumperPressed = false;
    int intakePickupCycle = 0;
    boolean intakeRetracting = false;
    boolean readyToDrop = false;
    ElapsedTime intakeExtendTimer = new ElapsedTime();




    @Override
    public void init() {

        leftFrontMotor  = hardwareMap.get(DcMotorEx.class,"LF");
        rightFrontMotor = hardwareMap.get(DcMotorEx.class, "RF");
        leftBackMotor = hardwareMap.get(DcMotorEx.class,"LB");
        rightBackMotor = hardwareMap.get(DcMotorEx.class,"RB");
        leftFrontMotor.setDirection(DcMotorEx.Direction.REVERSE);
        leftBackMotor.setDirection(DcMotorEx.Direction.REVERSE);

        leftFrontMotor.setZeroPowerBehavior(BRAKE);
        leftBackMotor.setZeroPowerBehavior(BRAKE);
        rightFrontMotor.setZeroPowerBehavior(BRAKE);
        rightBackMotor.setZeroPowerBehavior(BRAKE);


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

        intakeMotor.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);

    }

    @Override
    public void loop() {

        telemetry.addData("intakeMotor", intakeMotor.getCurrentPosition());
        telemetry.update();

        //Drive Code
        double y = -gamepad1.left_stick_y; // Remember, Y stick is reversed!
        double x = gamepad1.left_stick_x;
        double rx = gamepad1.right_stick_x;

        double denominator = Math.max(Math.abs(y) + Math.abs(x) + Math.abs(rx), 1);
        double leftFrontPower = (y + x + rx) / denominator;
        double leftBackPower = (y - x + rx) / denominator;
        double rightFrontPower = (y - x - rx) / denominator;
        double rightBackPower = (y + x - rx) / denominator;

        leftFrontMotor.setPower(leftFrontPower);
        leftBackMotor.setPower(leftBackPower);
        rightFrontMotor.setPower(rightFrontPower);
        rightBackMotor.setPower(rightBackPower);

        //intake
        if(gamepad1.right_bumper){
            if(intakeRetracted == false){
                if(rightBumperPressed == false){
                        intakeCycle = intakeCycle + 1;
                }

                if(intakeCycle == 1){
                    intakePivot.setPosition(Robot.INTAKE_PIVOT_PICKUP);

                }
                if(intakeCycle == 2){ rightBumperPressed = true;
                    intakeClaw.setPosition(Robot.INTAKE_CLAW_CLOSE);
                    intakeClawClosed = true;

                }
                if(intakeCycle == 3){
                    intakeClaw.setPosition(Robot.INTAKE_CLAW_OPEN);
                    intakeClawClosed = false;
                    intakeCycle = 1;
                }
            }
            if(intakeRetracted == false){
                if(rightBumperPressed == false){
                    scoringCycle = scoringCycle +1;
                }
                rightBumperPressed = true;
                //outtake code here
            }

        }
        else{
            rightBumperPressed = false;
        }
        if(gamepad1.left_bumper){
            if(readyToDrop = true){
                intakeClaw.setPosition(Robot.INTAKE_CLAW_OPEN);
            }
            if(leftBumperPressed = false){
                intakePickupCycle = intakePickupCycle + 1;
            }
            leftBumperPressed = true;
            if(intakePickupPosition == 1){

                if(intakePickupCycle == 1){
                    intakeWrist.setPosition(Robot.INTAKE_WRIST_STRAIGHT);
                }
                if(intakePickupCycle == 2){
                    intakeWrist.setPosition(Robot.INTAKE_WRIST_LEFT90);
                }
            }
            if(intakePickupPosition == 2){
                if(intakePickupCycle == 1){
                    intakeWrist.setPosition(Robot.INTAKE_WRIST_LEFT90);
                }
                if(intakePickupCycle == 2){
                    intakeWrist.setPosition(Robot.INTAKE_WRIST_STRAIGHT);
                }
            }
            if(intakePickupPosition == 3){
                if(intakePickupCycle == 1){
                    intakeWrist.setPosition(Robot.INTAKE_WRIST_STRAIGHT);
                }
                if(intakePickupCycle == 2){
                    intakeWrist.setPosition(Robot.INTAKE_WRIST_RIGHT90);
                }
            }
            if(intakePickupCycle == 2){
                intakePickupCycle = 0;
            }
        }
        else{
            leftBumperPressed = false;
        }
        if(gamepad1.back){
            if(backPressed = false){
                backTimes = backTimes +1;
            }
            backPressed = true;
            if(backTimes == 1){
                scoringSpec = false;
            }
            if(backTimes == 2){
                scoringSpec = true;
                backTimes = 0;
            }
        }
        else{
            backPressed = false;
        }
        if(gamepad1.dpad_left){
            if(intakeRetracted == false && intakeClawClosed == true ){
                intakeTurret.setPosition(Robot.INTAKE_TURRET_DROP_LEFT);
                intakeWrist.setPosition(Robot.INTAKE_WRIST_STRAIGHT);
                intakePivot.setPosition(Robot.INTAKE_PIVOT_DROP);
                intakeMotor.setTargetPosition(Robot.INTAKE_MOTOR_RETRACT);
                intakeMotor.setMode(DcMotor.RunMode.RUN_TO_POSITION);
                intakeMotor.setPower(1);
                intakeRetracted = true;

            }
            if(intakeRetracted == false && intakeClawClosed == false){
                intakeTurret.setPosition(Robot.INTAKE_TURRET_PICKUP_LEFT);
                intakeWrist.setPosition(Robot.INTAKE_WRIST_LEFT90);
                intakePickupPosition = 1;
                

            }
            if(intakeRetracted == true){
                intakeTurret.setPosition(Robot.INTAKE_TURRET_DROP_LEFT);
                intakePivot.setPosition(Robot.INTAKE_PIVOT_DROP);
                readyToDrop = true;
            }
        }
        if(gamepad1.dpad_right){
            if(intakeRetracted == false && intakeClawClosed == true ){
                intakeTurret.setPosition(Robot.INTAKE_TURRET_DROP_RIGHT);
                intakeWrist.setPosition(Robot.INTAKE_WRIST_STRAIGHT);
                intakePivot.setPosition(Robot.INTAKE_PIVOT_DROP);
                intakeMotor.setTargetPosition(Robot.INTAKE_MOTOR_RETRACT);
                intakeMotor.setMode(DcMotor.RunMode.RUN_TO_POSITION);
                intakeMotor.setPower(1);
                intakeRetracted = true;

            }
            if(intakeRetracted == false && intakeClawClosed == false){
                intakeTurret.setPosition(Robot.INTAKE_TURRET_PICKUP_RIGHT);
                intakeWrist.setPosition(Robot.INTAKE_WRIST_RIGHT90);
                intakePickupPosition = 3;


            }
            if(intakeRetracted == true){
                intakeTurret.setPosition(Robot.INTAKE_TURRET_DROP_RIGHT);
                intakePivot.setPosition(Robot.INTAKE_PIVOT_DROP);
                readyToDrop = true;
            }
        }
        if(intakeExtendTimer.milliseconds()>250 && intakeExtendTimer.milliseconds() <450){
            intakeClaw.setPosition(Robot.INTAKE_CLAW_OPEN);
            intakePivot.setPosition(Robot.INTAKE_PIVOT_PICKUP_READY);
            intakeWrist.setPosition(Robot.INTAKE_WRIST_STRAIGHT);
            intakeTurret.setPosition(Robot.INTAKE_TURRET_PICKUP_STRAIGHT);
            intakePickupPosition = 2;

        }
        if(gamepad1.right_trigger>0){
            if(intakeRetracted == true){
                intakeExtendTimer.reset();
            }
            intakeMotor.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
            intakeRetracted = false;
            intakeRetracting = false;
            readyToDrop = false;
        }

        if (gamepad1.right_trigger != 0.0 && intakeMotor.getCurrentPosition() < Robot.INTAKE_MOTOR_MAX_EXTEND ) {
            intakeMotor.setPower(gamepad1.right_trigger);
        }
        else if (gamepad1.left_trigger != 0.0) {
            intakeMotor.setTargetPosition(Robot.INTAKE_MOTOR_RETRACT);
            intakeMotor.setMode(DcMotor.RunMode.RUN_TO_POSITION);
            intakeMotor.setPower(1);
            intakePivot.setPosition(Robot.INTAKE_PIVOT_TRANSFER);
            intakeWrist.setPosition(Robot.INTAKE_WRIST_STRAIGHT);
            intakeTurret.setPosition(Robot.INTAKE_TURRET_TRANSFER);
            intakeRetracted = true;
            intakeRetracting = true;

        }

        else if(intakeRetracting == false){
            intakeMotor.setPower(0);
        }

        if(intakeMotor.getCurrentPosition()<5){
            intakeRetracting = false;
        }




        //PTO Code


        //Outtake Code
        if (gamepad2.right_stick_y != 0.0) {
            outtakeSlideMotor_left.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
            outtakeSlideMotor_right.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
            outtakeSlideMotor_left.setPower(gamepad2.right_stick_y);
            outtakeSlideMotor_right.setPower(gamepad2.right_stick_y);
        }
        else{
            outtakeSlideMotor_left.setPower(0);
            outtakeSlideMotor_right.setPower(0);
        }



    }

}