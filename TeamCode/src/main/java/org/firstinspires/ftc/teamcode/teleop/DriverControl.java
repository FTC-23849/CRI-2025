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
    Servo outtakePivotLeft;
    Servo outtakePivotRight;
    Servo outtakeTurret;
    Servo outtakeWrist;
    Servo outtakeClaw;


    boolean intakeRetracted = true; // keeps track if intake is retracted
    boolean intakeClawClosed = false; //keeps track if intake claw is closed
    int scoringCycle = 0;// keeps track of which part of scoring outtake is on
    int intakeCycle = 0;//keeps track of which part of picking up intake is on ex: lowered, claw open, claw close
    boolean rightBumperPressed = false; // checks if the right bumper has been pressed when the code checks
    int backTimes = 0; // keeps track of the amount of times the back button has been pressed for toggling between spec and sample scoring
    boolean scoringSpec = true; //true: scoring spec, false: scoring sample
    boolean backPressed = false; // checks if back button is pressed before the code is run
    int intakePickupPosition = 2; // which position turret is in, 1 is left, 2 is middle, 3 is right
    boolean leftBumperPressed = false; // checks if left bumper is pressed before the code is run
    int intakePickupCycle = 0; // keeps track of which direction intake claw is facing, straight, 90 degrees
    boolean intakeRetracting = false; // keeps track of if the intake is retracting to transfer
    boolean readyToDrop = false;// keeps track of if the robot is ready to drop sampel to hp
    double lastCycleTime = 0; // time when last cycle happened
    int totalCycleTimes = 1; // total amount of cycles
    ElapsedTime intakeExtendTimer = new ElapsedTime(); // not needed, replaced with encder ticks for lowering intake after extending
    ElapsedTime runTime = new ElapsedTime(); // total time code has ran


    boolean startPressed = false;
    int startTimes = 0;
    boolean highChamber = true;
    boolean goingToHighChamber = false;
    boolean goingToLowChamber = false;
    boolean goingToSampleScore = false;
    boolean goingToTransfer = false;
    boolean goingToHang = false;
    boolean hanging = false;







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
        if(scoringSpec == true){
            telemetry.addLine("SPECIMEN");
        }
        else{
            telemetry.addLine("SAMPLE");
        }
        telemetry.addData("runTime",runTime.milliseconds());
        telemetry.addData("average cycle time",runTime.milliseconds()/totalCycleTimes);
        telemetry.addData("lastCycleMilliseconds", runTime.milliseconds()-lastCycleTime);
        telemetry.addData("lastCycleTime",lastCycleTime);
        telemetry.addData("intakeExtendTimer",intakeExtendTimer.milliseconds());
        telemetry.addData("intakeRetracted",intakeRetracted );
        telemetry.addData("intakeClawClosed",intakeClawClosed );
        telemetry.addData("scoringCycle",scoringCycle );
        telemetry.addData("intakeCycle",intakeCycle );
        telemetry.addData("rightBumperPressed",rightBumperPressed );
        telemetry.addData("backTimes",backTimes );
        telemetry.addData("scoringSpec",scoringSpec );
        telemetry.addData("backPressed",backPressed );
        telemetry.addData("intakePickupPosition",intakePickupPosition );
        telemetry.addData("leftBumperPressed", leftBumperPressed);
        telemetry.addData("intakePickupCycle",intakePickupCycle );
        telemetry.addData("intakeRetracting",intakeRetracting );
        telemetry.addData("readyToDrop", readyToDrop);
        telemetry.addData("intakeMotor", intakeMotor.getCurrentPosition());
        telemetry.update();

        if(totalCycleTimes == 1){
            intakePivot.setPosition(Robot.INTAKE_PIVOT_TRANSFER);
            intakeWrist.setPosition(Robot.INTAKE_WRIST_STRAIGHT);
            intakeTurret.setPosition(Robot.INTAKE_TURRET_TRANSFER);
        }
        totalCycleTimes = totalCycleTimes+1;


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
            //if intake is extended, cycle through intake functions(drop intake open claw close claw)
            if(intakeRetracted == false){
                if(rightBumperPressed == false){
                    //checks if its first time that the button is detected to be pressed
                        intakeCycle = intakeCycle + 1;
                }

                if(intakeCycle == 1){
                    intakePivot.setPosition(Robot.INTAKE_PIVOT_PICKUP);
                    //drop intake

                }
                if(intakeCycle == 2){ rightBumperPressed = true;
                    intakeClaw.setPosition(Robot.INTAKE_CLAW_CLOSE);
                    intakeClawClosed = true;
                    //close claw

                }
                if(intakeCycle == 3){
                    intakeClaw.setPosition(Robot.INTAKE_CLAW_OPEN);
                    intakeClawClosed = false;
                    intakeCycle = 1;
                    //open claw
                }
            }
            if(intakeRetracted == false){
                if(rightBumperPressed == false){
                    scoringCycle = scoringCycle + 1;
                    //checks if first time button is detected
                }
                rightBumperPressed = true;
                if(scoringSpec == true){
                    if(scoringCycle == 4){
                        scoringCycle = 0;
                    }
                }
                else{
                    if(scoringCycle == 3){
                        scoringCycle = 0;
                    }
                }

                //used to cycle through outtake functions
                //makes it so every tiem button is pressed only goes up by 1


                //outtake code here

            }

        }
        else{
            rightBumperPressed = false;
            //if right bumper is let go then the button is not pressed
        }
        if(gamepad1.left_bumper){
            // checks if its first time running
            if(leftBumperPressed == false){
                intakePickupCycle = intakePickupCycle + 1;
                if(readyToDrop == true){
                    intakeClaw.setPosition(Robot.INTAKE_CLAW_OPEN);
                    intakeClawClosed = false;
                    readyToDrop = false;
                    //drops sample off side then resets to allow left bumper to be used for other things
                }
            }
            leftBumperPressed = true;
            //dumb stuff that changes the direction differently depending on direction turret is facing
            if(intakeRetracted == false) {
                if (intakePickupPosition == 1) {
                    if (intakePickupCycle == 1) {
                        intakeWrist.setPosition(Robot.INTAKE_WRIST_LEFT90);
                    }
                    if (intakePickupCycle == 2) {
                        intakeWrist.setPosition(Robot.INTAKE_WRIST_STRAIGHT);

                    }
                }
                if (intakePickupPosition == 2) {
                    if (intakePickupCycle == 1) {
                        intakeWrist.setPosition(Robot.INTAKE_WRIST_LEFT90);
                    }
                    if (intakePickupCycle == 2) {
                        intakeWrist.setPosition(Robot.INTAKE_WRIST_STRAIGHT);
                    }
                }
                if (intakePickupPosition == 3) {
                    if (intakePickupCycle == 1) {
                        intakeWrist.setPosition(Robot.INTAKE_WRIST_RIGHT90);
                    }
                    if (intakePickupCycle == 2) {
                        intakeWrist.setPosition(Robot.INTAKE_WRIST_STRAIGHT);
                    }
                }
            }
            if(intakePickupCycle == 2){
                intakePickupCycle = 0;
            }

        }
        else{
            //resets left bumper pressed after let bumper is let go
            leftBumperPressed = false;
        }
        // toggle between spec and sample
        if(gamepad1.back){
            //checks if first time running
            if(backPressed == false){

                backTimes = backTimes + 1;
                //does a change first time only
            }
            backPressed = true;
            if(backTimes == 1){
                scoringSpec = false;
                //setting to sample, defaults to spec
            }
            if(backTimes == 2){
                scoringSpec = true;
                //setting to spec, again
                backTimes = 0;
            }
        }
        else{
            //setting back pressed to false again after back button is let go
            backPressed = false;
        }
        if(gamepad1.start){
            //checks if first time running
            if(startPressed == false){

                startTimes = startTimes + 1;
                //does a change first time only
            }
            startPressed = true;
            if(startTimes == 1){
                highChamber = false;
                //setting to sample, defaults to spec
            }
            if(startTimes == 2){
                highChamber = true;
                //setting to spec, again
                startTimes = 0;
            }
        }
        else{
            //setting back pressed to false again after back button is let go
            startPressed = false;
        }
        if(gamepad1.dpad_left){
            //retracts intake and prepares to drop off side if intake is extended and intake claw is closed
            if(intakeRetracted == false && intakeClawClosed == true ){
                intakeTurret.setPosition(Robot.INTAKE_TURRET_DROP_LEFT);
                intakeWrist.setPosition(Robot.INTAKE_WRIST_LEFT90);
                intakePivot.setPosition(Robot.INTAKE_PIVOT_DROP);
                intakeMotor.setTargetPosition(Robot.INTAKE_MOTOR_RETRACT);
                intakeMotor.setMode(DcMotor.RunMode.RUN_TO_POSITION);
                intakeMotor.setPower(1);
                intakeRetracted = true;
                readyToDrop = true;
                intakeRetracting = true;

            }
            //moves turret to side to pick up if intake is extended but has not picked up sample yet
            if(intakeRetracted == false && intakeClawClosed == false){
                intakeTurret.setPosition(Robot.INTAKE_TURRET_PICKUP_LEFT);
                intakeWrist.setPosition(Robot.INTAKE_WRIST_LEFT90);
                intakePickupPosition = 1;
                

            }
            //drops off side when intake is retracted, somewhat useless
            if(intakeRetracted == true){
                intakeTurret.setPosition(Robot.INTAKE_TURRET_DROP_LEFT);
                intakePivot.setPosition(Robot.INTAKE_PIVOT_DROP);
                intakeWrist.setPosition(Robot.INTAKE_WRIST_LEFT90);
                readyToDrop = true;
            }
        }
        if(gamepad1.dpad_right){
            //retracts intake and drops off side when itnake is extended and the claw is closed
            if(intakeRetracted == false && intakeClawClosed == true ){
                intakeTurret.setPosition(Robot.INTAKE_TURRET_DROP_RIGHT);
                intakeWrist.setPosition(Robot.INTAKE_WRIST_RIGHT90);
                intakePivot.setPosition(Robot.INTAKE_PIVOT_DROP);
                intakeMotor.setTargetPosition(Robot.INTAKE_MOTOR_RETRACT);
                intakeMotor.setMode(DcMotor.RunMode.RUN_TO_POSITION);
                intakeMotor.setPower(1);
                intakeRetracted = true;
                readyToDrop = true;
                intakeRetracting = true;

            }
            //makes turret pick up from side when intake is extended but claw is not closed
            if(intakeRetracted == false && intakeClawClosed == false){
                intakeTurret.setPosition(Robot.INTAKE_TURRET_PICKUP_RIGHT);
                intakeWrist.setPosition(Robot.INTAKE_WRIST_RIGHT90);
                intakePickupPosition = 3;


            }
            //moves turret to side to drop sample when intake is retracted
            if(intakeRetracted == true){
                intakeTurret.setPosition(Robot.INTAKE_TURRET_DROP_RIGHT);
                intakePivot.setPosition(Robot.INTAKE_PIVOT_DROP);
                intakeWrist.setPosition(Robot.INTAKE_WRIST_RIGHT90);
                readyToDrop = true;
            }
        }
        if(gamepad1.dpad_up){
            // makes turret pick up in a straight line
            if(intakeRetracted == false && intakeClawClosed == false){
                intakeTurret.setPosition(Robot.INTAKE_TURRET_PICKUP_STRAIGHT);
                intakeWrist.setPosition(Robot.INTAKE_WRIST_STRAIGHT);
                intakePickupPosition = 2;
            }

        }
         // the && gamepad1.a == false makes sure nothing happens when intake motors are adjusted
        // drops intake if intake is far enough away so intake doesnt hit robot
        if(intakeMotor.getCurrentPosition() > Robot.INTAKE_LOWER_ENCODER_TICKS && gamepad1.right_trigger > 0 && gamepad1.a == false ){
            intakeClaw.setPosition(Robot.INTAKE_CLAW_OPEN);
            intakePivot.setPosition(Robot.INTAKE_PIVOT_PICKUP_READY);
            intakeWrist.setPosition(Robot.INTAKE_WRIST_STRAIGHT);
            intakeTurret.setPosition(Robot.INTAKE_TURRET_PICKUP_STRAIGHT);
            intakeClawClosed = false;
            intakeRetracted = false;
            intakeCycle = 0;
            intakePickupPosition = 2;

        }

        if(gamepad1.right_trigger > 0 && gamepad1.a == false){
            // just sets some things
            intakeMotor.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
            intakeRetracting = false;
            readyToDrop = false;
            intakeTurret.setPosition(Robot.INTAKE_TURRET_PICKUP_STRAIGHT);
        }
        // powers intake motors if there less then the max extension
        if (gamepad1.right_trigger != 0.0 && intakeMotor.getCurrentPosition() < Robot.INTAKE_MOTOR_MAX_EXTEND && gamepad1.a == false) {
            intakeMotor.setPower(gamepad1.right_trigger);
        }
        //retracts intake
        else if (gamepad1.left_trigger != 0.0 && gamepad1.a == false) {
            intakeMotor.setTargetPosition(Robot.INTAKE_MOTOR_RETRACT);
            intakeMotor.setMode(DcMotor.RunMode.RUN_TO_POSITION);
            intakeMotor.setPower(1);
            intakePivot.setPosition(Robot.INTAKE_PIVOT_TRANSFER);
            intakeWrist.setPosition(Robot.INTAKE_WRIST_STRAIGHT);
            intakeTurret.setPosition(Robot.INTAKE_TURRET_TRANSFER);
            intakeRetracted = true;
            intakeRetracting = true;

        }
        //manual controls of intake motor
        else if(gamepad1.right_trigger > 0 && gamepad1.a){
            intakeMotor.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
            intakeMotor.setPower(gamepad1.right_trigger);
        }
        else if(gamepad1.left_trigger > 0 && gamepad1.a){
            intakeMotor.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
            intakeMotor.setPower(-gamepad1.left_trigger);
        }
        else if(gamepad1.a){
            intakeMotor.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        }
        //sets power only to 0 is intake isnt retracting and no buttons are being pressed
        else if(intakeRetracting == false){
            intakeMotor.setPower(0);
        }
        //makes intake retracting false if intake is close enough, maybe adjust value to be even smaller due to drift, possibly get rid of this
        if(intakeMotor.getCurrentPosition() < 5){
            intakeRetracting = false;
        }




        //PTO Code


        //Outtake Code
        if(gamepad1.right_bumper && intakeRetracted && highChamber == true && scoringCycle == 1){
            outtakePivotLeft.setPosition(Robot.OUTTAKE_PIVOT_SPECIMEN_PICKUP);
            outtakePivotRight.setPosition(Robot.OUTTAKE_PIVOT_SPECIMEN_PICKUP);
            outtakeClaw.setPosition(Robot.OUTTAKE_CLAW_OPEN);

        }




    }

}