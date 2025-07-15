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
import static com.qualcomm.robotcore.hardware.DcMotor.ZeroPowerBehavior.FLOAT;

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
    Servo leftPto;
    Servo rightPto;
    Servo displayLight;


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

    boolean intakeAvoid = false; //checking if intake avoids outtake when pivoting
    boolean ptoEngaged = false;
    ElapsedTime intakeAvoidTimer = new ElapsedTime(); // timer for intake to avoid outtake whne it pivots
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
    boolean outtakeRetracted = true;
    boolean transferring = false;
    ElapsedTime transferTimer = new ElapsedTime();
    int hangCycle = 0;
    ElapsedTime ptoDelay = new ElapsedTime();
    ElapsedTime hangHold = new ElapsedTime();
    ElapsedTime hangDrop = new ElapsedTime();
    ElapsedTime hangTimer = new ElapsedTime();
    boolean initalized = false;







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
        outtakePivotLeft = hardwareMap.get(Servo.class, "outtakePivotLeft");
        outtakePivotRight = hardwareMap.get(Servo.class, "outtakePivotRight");
        outtakeTurret = hardwareMap.get(Servo.class, "outtakeTurret");
        outtakeWrist = hardwareMap.get(Servo.class, "outtakeWrist");
        outtakeClaw = hardwareMap.get(Servo.class, "outtakeClaw");
        leftPto = hardwareMap.get(Servo.class, "leftPto");
        rightPto = hardwareMap.get(Servo.class, "rightPto");
        displayLight = hardwareMap.get(Servo.class, "displayLight");

        intakeMotor.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        intakeMotor.setDirection(DcMotorEx.Direction.REVERSE);

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
        telemetry.addData("left slide motor", outtakeSlideMotor_left.getCurrentPosition());
        telemetry.addData("intakeAvoid",intakeAvoid);
        telemetry.addData("ptoEngaged",ptoEngaged);
        telemetry.addData("intakeAvoidTimer",intakeAvoidTimer.milliseconds());
        telemetry.addData("startPressed",startPressed);
        telemetry.addData("startTimes",startTimes);
        telemetry.addData("highChamber",highChamber);
        telemetry.addData("goingToHighChamber",goingToHighChamber);
        telemetry.addData("goingToLowChamber",goingToLowChamber);
        telemetry.addData("goingToSampleScore",goingToSampleScore);
        telemetry.addData("goingToTransfer",goingToTransfer);
        telemetry.addData("goingToHang",goingToHang);
        telemetry.addData("hanging",hanging);
        telemetry.addData("outtakeRetracted",outtakeRetracted);
        telemetry.addData("transferring",transferring);
        telemetry.addData("transferTimer",transferTimer.milliseconds());
        telemetry.addData("hangCycle",hangCycle);
        telemetry.addData("ptoDelay",ptoDelay.milliseconds());
        telemetry.addData("hangHold",hangHold);
        telemetry.addData("initalized", initalized);
        telemetry.addData("lf", leftFrontMotor.getCurrentPosition());
        telemetry.addData("rf", rightFrontMotor.getCurrentPosition());
        telemetry.addData("lb", rightFrontMotor.getCurrentPosition());
        telemetry.addData("rb", rightBackMotor.getCurrentPosition());
        telemetry.addData("slideMotorLeft", outtakeSlideMotor_left.getCurrentPosition());
        telemetry.addData("hangTimer", hangTimer.milliseconds());
        telemetry.update();


        if(totalCycleTimes == 1){
            intakePivot.setPosition(Robot.INTAKE_PIVOT_AVOID);
            intakeWrist.setPosition(Robot.INTAKE_WRIST_STRAIGHT);
            intakeTurret.setPosition(Robot.INTAKE_TURRET_TRANSFER);
            intakeClaw.setPosition(Robot.INTAKE_CLAW_OPEN);
            outtakePivotLeft.setPosition(Robot.OUTTAKE_PIVOT_TRANSFER);
            outtakePivotRight.setPosition(Robot.OUTTAKE_PIVOT_TRANSFER);
            outtakeTurret.setPosition(Robot.OUTTAKE_TURRET_STRAIGHT);
            outtakeWrist.setPosition(Robot.OUTTAKE_WRIST_TRANSFER);
            outtakeClaw.setPosition(Robot.OUTTAKE_CLAW_OPEN);
            leftPto.setPosition(Robot.LEFT_PTO_DISENGAGE);
            rightPto.setPosition(Robot.RIGHT_PTO_DISENGAGE);
        }
        if(runTime.milliseconds() > 5000 && initalized == false){
            intakePivot.setPosition(Robot.INTAKE_PIVOT_TRANSFER);
            initalized = true;
        }
        totalCycleTimes = totalCycleTimes+1;
        if(gamepad1.left_stick_button) {
            if (scoringSpec == false) {
                displayLight.setPosition(0.388);
            } else if (highChamber == true) {
                displayLight.setPosition(0.5);
            } else if (highChamber == false) {
                displayLight.setPosition(0.611);
            } else {
                displayLight.setPosition(1);
            }
        }
        else{
            displayLight.setPosition(0);
        }


        //Drive Code
        if (hanging == false) {


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
        }



        //intake
        if(gamepad1.right_bumper){
            //if intake is extended, cycle through intake functions(drop intake open claw close claw)
            if(intakeRetracted == false){
                if(rightBumperPressed == false){
                    //checks if its first time that the button is detected to be pressed
                    intakeCycle = intakeCycle + 1;
                }
                rightBumperPressed = true;

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
            if(intakeRetracted == true){
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
            if(outtakeRetracted == false){
                goingToSampleScore = false;
                outtakeSlideMotor_left.setTargetPosition(Robot.OUTTAKE_MOTOR_RETRACT);
                outtakeSlideMotor_right.setTargetPosition(Robot.OUTTAKE_MOTOR_RETRACT);
                outtakeSlideMotor_left.setMode(DcMotor.RunMode.RUN_TO_POSITION);
                outtakeSlideMotor_right.setMode(DcMotor.RunMode.RUN_TO_POSITION);
                outtakeSlideMotor_left.setPower(1);
                outtakeSlideMotor_right.setPower(1);
                goingToTransfer = true;
                outtakePivotLeft.setPosition(Robot.OUTTAKE_PIVOT_TRANSFER);
                outtakePivotRight.setPosition(Robot.OUTTAKE_PIVOT_TRANSFER);
                outtakeTurret.setPosition(Robot.OUTTAKE_TURRET_STRAIGHT);
                outtakeWrist.setPosition(Robot.OUTTAKE_WRIST_TRANSFER);
                outtakeClaw.setPosition(Robot.OUTTAKE_CLAW_OPEN);
                scoringCycle = 0;
            }

        }
        else{
            //resets left bumper pressed after let bumper is let go
            leftBumperPressed = false;
        }
        // toggle between spec and sample
        if(gamepad1.back){
            scoringCycle = 0;
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
            if(intakeRetracted == true && outtakeRetracted){
                intakeTurret.setPosition(Robot.INTAKE_TURRET_DROP_LEFT);
                intakePivot.setPosition(Robot.INTAKE_PIVOT_DROP);
                intakeWrist.setPosition(Robot.INTAKE_WRIST_LEFT90);
                readyToDrop = true;
            }
            if(outtakeRetracted == false){
                outtakeTurret.setPosition(Robot.OUTTAKE_TURRET_LEFT90);
            }
        }
        if(gamepad1.dpad_right){
            //retracts intake and drops off side when itnake is extended and the claw is closed
            if(intakeRetracted == false && intakeClawClosed == true){
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
            if(intakeRetracted == true && outtakeRetracted){
                intakeTurret.setPosition(Robot.INTAKE_TURRET_DROP_RIGHT);
                intakePivot.setPosition(Robot.INTAKE_PIVOT_DROP);
                intakeWrist.setPosition(Robot.INTAKE_WRIST_RIGHT90);
                readyToDrop = true;
            }
            if(outtakeRetracted == false){
                outtakeTurret.setPosition(Robot.OUTTAKE_TURRET_RIGHT90);
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
        if(intakeMotor.getCurrentPosition() > Robot.INTAKE_LOWER_ENCODER_TICKS && gamepad1.right_trigger > 0 && gamepad1.a == false && gamepad1.b == false ){
            intakeClaw.setPosition(Robot.INTAKE_CLAW_OPEN);
            intakePivot.setPosition(Robot.INTAKE_PIVOT_PICKUP_READY);
            intakeWrist.setPosition(Robot.INTAKE_WRIST_STRAIGHT);
            intakeTurret.setPosition(Robot.INTAKE_TURRET_PICKUP_STRAIGHT);
            intakeClawClosed = false;
            intakeRetracted = false;
            intakeCycle = 0;
            intakePickupPosition = 2;

        }

        if(gamepad1.right_trigger > 0 && gamepad1.a == false && gamepad1.b == false){
            // just sets some things
            intakeMotor.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
            intakeRetracting = false;
            readyToDrop = false;
            intakeTurret.setPosition(Robot.INTAKE_TURRET_PICKUP_STRAIGHT);
        }
        // powers intake motors if there less then the max extension
        if (gamepad1.right_trigger != 0.0 && intakeMotor.getCurrentPosition() < Robot.INTAKE_MOTOR_MAX_EXTEND && gamepad1.a == false && gamepad1.b == false) {
            intakeMotor.setPower(gamepad1.right_trigger);
        }
        //retracts intake
        else if (gamepad1.left_trigger > 0.1 && gamepad1.a == false) {
            intakeMotor.setTargetPosition(Robot.INTAKE_MOTOR_RETRACT);
            intakeMotor.setMode(DcMotor.RunMode.RUN_TO_POSITION);
            intakeMotor.setPower(1);
            intakePivot.setPosition(Robot.INTAKE_PIVOT_TRANSFER);
            intakeWrist.setPosition(Robot.INTAKE_WRIST_STRAIGHT);
            intakeTurret.setPosition(Robot.INTAKE_TURRET_TRANSFER);
            intakeRetracted = true;
            intakeRetracting = true;
            if(scoringSpec == false){
                transferring = true;
            }

        }
        //manual controls of intake motor
        else if(gamepad1.right_trigger > 0 && gamepad1.a && gamepad1.b == false){
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
        //auto transfer
        if(transferring && intakeMotor.getCurrentPosition() < 5){
            transferTimer.reset();
            transferring = false;
        }
        if(transferTimer.milliseconds() > 200 && transferTimer.milliseconds() < 300){
            outtakeClaw.setPosition(Robot.OUTTAKE_CLAW_CLOSE);
        }
        if(transferTimer.milliseconds() > 300 && transferTimer.milliseconds() < 400){
            intakeClaw.setPosition(Robot.INTAKE_CLAW_OPEN);
        }
        //goes to score sample
        if(transferTimer.milliseconds() > 300 && transferTimer.milliseconds() < 400){
            scoringCycle = 1;
            outtakeSlideMotor_left.setTargetPosition(Robot.OUTTAKE_MOTOR_SAMPLE_SCORE);
            outtakeSlideMotor_right.setTargetPosition(Robot.OUTTAKE_MOTOR_SAMPLE_SCORE);
            outtakeSlideMotor_left.setMode(DcMotor.RunMode.RUN_TO_POSITION);
            outtakeSlideMotor_right.setMode(DcMotor.RunMode.RUN_TO_POSITION);
            outtakeSlideMotor_left.setPower(1);
            outtakeSlideMotor_right.setPower(1);
            goingToSampleScore = true;
            outtakePivotLeft.setPosition(Robot.OUTTAKE_PIVOT_SAMPLE_SCORE);
            outtakePivotRight.setPosition(Robot.OUTTAKE_PIVOT_SAMPLE_SCORE);
            outtakeTurret.setPosition(Robot.OUTTAKE_TURRET_STRAIGHT);
            outtakeWrist.setPosition(Robot.OUTTAKE_WRIST_SAMPLE_SCORE);
            intakeAvoid = true;
            intakeAvoidTimer.reset();
        }




        //PTO Code
        //gamepad2 testing
        if(gamepad2.a){
            leftPto.setPosition(Robot.LEFT_PTO_ENGAGE);
            rightPto.setPosition(Robot.RIGHT_PTO_ENGAGE);
            ptoEngaged = true;
        }
        if(gamepad2.b){
            leftPto.setPosition(Robot.LEFT_PTO_DISENGAGE);
            rightPto.setPosition(Robot.RIGHT_PTO_DISENGAGE);
            ptoEngaged = false;
        }




        //Outtake Code
        //controlling outtake with gamepad 2
        if(gamepad2.right_stick_y != 0){
            goingToHighChamber = false;
            goingToLowChamber = false;
            goingToSampleScore = false;
            goingToTransfer = false;
            goingToHang = false;
            hanging = false;
            outtakeSlideMotor_left.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
            outtakeSlideMotor_right.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
            outtakeSlideMotor_left.setPower(gamepad2.right_stick_y);
            outtakeSlideMotor_right.setPower(gamepad2.right_stick_y);
            //setting power to zero only if nothing is happening

        }
        else if((gamepad1.right_trigger > 0.1 || gamepad1.left_trigger > 0.1) && gamepad1.b){
            outtakeSlideMotor_left.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
            outtakeSlideMotor_right.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
            outtakeSlideMotor_left.setPower(gamepad1.right_trigger - gamepad1.left_trigger);
            outtakeSlideMotor_right.setPower(gamepad1.right_trigger - gamepad1.left_trigger);


        }
        else if(gamepad1.b){
            outtakeSlideMotor_left.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
            outtakeSlideMotor_right.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        }
        else if (!goingToHighChamber && !goingToLowChamber && !goingToSampleScore && !goingToTransfer && !goingToHang && !hanging) {
            outtakeSlideMotor_left.setPower(0);
            outtakeSlideMotor_right.setPower(0);
        }
        //flips outtake arm to pick up sample
        if(gamepad1.right_bumper && intakeRetracted && scoringSpec && scoringCycle == 1){
//            outtakePivotLeft.setPosition(Robot.OUTTAKE_PIVOT_SPECIMEN_PICKUP);
//            outtakePivotRight.setPosition(Robot.OUTTAKE_PIVOT_SPECIMEN_PICKUP);
//            outtakeTurret.setPosition(Robot.OUTTAKE_TURRET_STRAIGHT);
//            outtakeWrist.setPosition(Robot.OUTTAKE_WRIST_SPECIMEN_PICKUP);
//            outtakeClaw.setPosition(Robot.OUTTAKE_CLAW_OPEN);
//            outtakeSlideMotor_left.setTargetPosition(Robot.OUTTAKE_MOTOR_RETRACT);
//            outtakeSlideMotor_right.setTargetPosition(Robot.OUTTAKE_MOTOR_RETRACT);
//            outtakeSlideMotor_left.setMode(DcMotor.RunMode.RUN_TO_POSITION);
//            outtakeSlideMotor_right.setMode(DcMotor.RunMode.RUN_TO_POSITION);
//            outtakeSlideMotor_left.setPower(1);
//            outtakeSlideMotor_right.setPower(1);
//            outtakeRetracted = true;
//            goingToTransfer = true;
//            goingToLowChamber = false;
//            goingToHighChamber = false;
            intakeAvoid = true;
            intakeAvoidTimer.reset();
        }
        if(scoringCycle == 1 && scoringSpec && intakeAvoidTimer.milliseconds() > 250 && intakeAvoidTimer.milliseconds() < 500){
            outtakePivotLeft.setPosition(Robot.OUTTAKE_PIVOT_SPECIMEN_PICKUP);
            outtakePivotRight.setPosition(Robot.OUTTAKE_PIVOT_SPECIMEN_PICKUP);
            outtakeTurret.setPosition(Robot.OUTTAKE_TURRET_STRAIGHT);
            outtakeWrist.setPosition(Robot.OUTTAKE_WRIST_SPECIMEN_PICKUP);
            outtakeClaw.setPosition(Robot.OUTTAKE_CLAW_OPEN);
            outtakeSlideMotor_left.setTargetPosition(Robot.OUTTAKE_MOTOR_RETRACT);
            outtakeSlideMotor_right.setTargetPosition(Robot.OUTTAKE_MOTOR_RETRACT);
            outtakeSlideMotor_left.setMode(DcMotor.RunMode.RUN_TO_POSITION);
            outtakeSlideMotor_right.setMode(DcMotor.RunMode.RUN_TO_POSITION);
            outtakeSlideMotor_left.setPower(1);
            outtakeSlideMotor_right.setPower(1);
            outtakeRetracted = true;
            goingToTransfer = true;
            goingToLowChamber = false;
            goingToHighChamber = false;

        }
        //doesnt continously power motors when they are all the way down
        if((outtakeSlideMotor_right.getCurrentPosition() >-5 || outtakeSlideMotor_left.getCurrentPosition() > -5) && goingToTransfer == true){
            goingToTransfer = false;
        }
        //closes outtake claw
        if (gamepad1.right_bumper && intakeRetracted && scoringSpec && scoringCycle == 2) {
            outtakeClaw.setPosition(Robot.OUTTAKE_CLAW_CLOSE);
        }
        //goes to score spec straight position for high chamber
        if(gamepad1.right_bumper && intakeRetracted && highChamber == true && scoringSpec && scoringCycle == 3){
            outtakeSlideMotor_left.setTargetPosition(Robot.OUTTAKE_MOTOR_HIGH_SPECIMEN_SCORE);
            outtakeSlideMotor_right.setTargetPosition(Robot.OUTTAKE_MOTOR_HIGH_SPECIMEN_SCORE);
            outtakeSlideMotor_left.setMode(DcMotor.RunMode.RUN_TO_POSITION);
            outtakeSlideMotor_right.setMode(DcMotor.RunMode.RUN_TO_POSITION);
            outtakeSlideMotor_left.setPower(1);
            outtakeSlideMotor_right.setPower(1);
            outtakeRetracted = false;
            outtakePivotLeft.setPosition(Robot.OUTTAKE_PIVOT_HIGH_SPECIMEN_SCORE);
            outtakePivotRight.setPosition(Robot.OUTTAKE_PIVOT_HIGH_SPECIMEN_SCORE);
            outtakeTurret.setPosition(Robot.OUTTAKE_TURRET_STRAIGHT);
            outtakeWrist.setPosition(Robot.OUTTAKE_WRIST_HIGH_SPECIMEN_SCORE);
            goingToHighChamber = true;
        }
        //goes to score spec in straight for low chamber(low chamber fadeeee)
        if(gamepad1.right_bumper && intakeRetracted && highChamber == false && scoringSpec && scoringCycle == 3){
            outtakeSlideMotor_left.setTargetPosition(Robot.OUTTAKE_MOTOR_LOW_SPECIMEN_SCORE);
            outtakeSlideMotor_right.setTargetPosition(Robot.OUTTAKE_MOTOR_LOW_SPECIMEN_SCORE);
            outtakeSlideMotor_left.setMode(DcMotor.RunMode.RUN_TO_POSITION);
            outtakeSlideMotor_right.setMode(DcMotor.RunMode.RUN_TO_POSITION);
            outtakeSlideMotor_left.setPower(1);
            outtakeSlideMotor_right.setPower(1);
            outtakeRetracted = false;
            outtakePivotLeft.setPosition(Robot.OUTTAKE_PIVOT_LOW_SPECIMEN_SCORE);
            outtakePivotRight.setPosition(Robot.OUTTAKE_PIVOT_LOW_SPECIMEN_SCORE);
            outtakeTurret.setPosition(Robot.OUTTAKE_TURRET_STRAIGHT);
            outtakeWrist.setPosition(Robot.OUTTAKE_WRIST_LOW_SPECIMEN_SCORE);
            goingToLowChamber = true;
        }
        //releases outtake claw
        if(gamepad1.right_bumper && intakeRetracted && scoringSpec && scoringCycle == 0){
            outtakeClaw.setPosition(Robot.OUTTAKE_CLAW_OPEN);
        }
        //sample scoring
        if(gamepad1.right_bumper && intakeRetracted && !scoringSpec && scoringCycle == 1){
            outtakeSlideMotor_left.setTargetPosition(Robot.OUTTAKE_MOTOR_SAMPLE_SCORE);
            outtakeSlideMotor_right.setTargetPosition(Robot.OUTTAKE_MOTOR_SAMPLE_SCORE);
            outtakeSlideMotor_left.setMode(DcMotor.RunMode.RUN_TO_POSITION);
            outtakeSlideMotor_right.setMode(DcMotor.RunMode.RUN_TO_POSITION);
            outtakeSlideMotor_left.setPower(1);
            outtakeSlideMotor_right.setPower(1);
            goingToSampleScore = true;
            intakeAvoid = true;
            intakeAvoidTimer.reset();
        }
        if(gamepad1.right_bumper && intakeRetracted && !scoringSpec && scoringCycle == 2){
            outtakeClaw.setPosition(Robot.OUTTAKE_CLAW_OPEN);
        }
        if(outtakeSlideMotor_left.getCurrentPosition()<-200 && outtakeSlideMotor_left.getCurrentPosition()>-300 && goingToSampleScore){
            outtakePivotLeft.setPosition(Robot.OUTTAKE_PIVOT_SAMPLE_SCORE);
            outtakePivotRight.setPosition(Robot.OUTTAKE_PIVOT_SAMPLE_SCORE);
            outtakeTurret.setPosition(Robot.OUTTAKE_TURRET_STRAIGHT);
            outtakeWrist.setPosition(Robot.OUTTAKE_WRIST_SAMPLE_SCORE);
        }
        if((gamepad1.right_bumper && intakeRetracted && !scoringSpec && scoringCycle == 0) || (outtakeRetracted && gamepad1.right_trigger > 0.1 && gamepad1.a == false && gamepad1.b == false)){
            goingToSampleScore = false;
            outtakeSlideMotor_left.setTargetPosition(Robot.OUTTAKE_MOTOR_RETRACT);
            outtakeSlideMotor_right.setTargetPosition(Robot.OUTTAKE_MOTOR_RETRACT);
            outtakeSlideMotor_left.setMode(DcMotor.RunMode.RUN_TO_POSITION);
            outtakeSlideMotor_right.setMode(DcMotor.RunMode.RUN_TO_POSITION);
            outtakeSlideMotor_left.setPower(1);
            outtakeSlideMotor_right.setPower(1);
            goingToTransfer = true;
            outtakePivotLeft.setPosition(Robot.OUTTAKE_PIVOT_TRANSFER);
            outtakePivotRight.setPosition(Robot.OUTTAKE_PIVOT_TRANSFER);
            outtakeTurret.setPosition(Robot.OUTTAKE_TURRET_STRAIGHT);
            outtakeWrist.setPosition(Robot.OUTTAKE_WRIST_TRANSFER);
            outtakeClaw.setPosition(Robot.OUTTAKE_CLAW_OPEN);
        }

        if(intakeAvoid && intakeAvoidTimer.milliseconds() < 500){
            intakePivot.setPosition(Robot.INTAKE_PIVOT_AVOID);
        }
        if(intakeAvoidTimer.milliseconds() > 500 && intakeAvoid){
            intakePivot.setPosition(Robot.INTAKE_PIVOT_RETRACT);
            intakeAvoid = false;
        }
        if(gamepad2.y){
            outtakePivotLeft.setPosition(Robot.OUTTAKE_PIVOT_HANG);
            outtakePivotRight.setPosition(Robot.OUTTAKE_PIVOT_HANG);
        }
        //raises slides for hang
        if(gamepad1.right_stick_button && gamepad1.left_stick_button){
            hanging = true;
            outtakeSlideMotor_left.setTargetPosition(Robot.OUTTAKE_MOTOR_L2_PREPARE);
            outtakeSlideMotor_right.setTargetPosition(Robot.OUTTAKE_MOTOR_L2_PREPARE);
            outtakeSlideMotor_left.setMode(DcMotor.RunMode.RUN_TO_POSITION);
            outtakeSlideMotor_right.setMode(DcMotor.RunMode.RUN_TO_POSITION);
            outtakeSlideMotor_left.setPower(0.2);
            outtakeSlideMotor_right.setPower(0.2);
            outtakePivotLeft.setPosition(Robot.OUTTAKE_PIVOT_HANG);
            outtakePivotRight.setPosition(Robot.OUTTAKE_PIVOT_HANG);
            outtakeTurret.setPosition(Robot.OUTTAKE_TURRET_STRAIGHT);
            outtakeWrist.setPosition(Robot.OUTTAKE_WRIST_HANG);
            intakePivot.setPosition(Robot.INTAKE_PIVOT_AVOID);
            hangCycle = 1;
        }
        //lets hook drop on l2 bar
        if(hangCycle == 1 && (outtakeSlideMotor_left.getCurrentPosition() < (Robot.OUTTAKE_MOTOR_L2_PREPARE + 50))/* && hangTimer.milliseconds() > 1000*/){
            hangTimer.reset();
            outtakeSlideMotor_left.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
            outtakeSlideMotor_right.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
            outtakeSlideMotor_left.setZeroPowerBehavior(FLOAT);
            outtakeSlideMotor_right.setZeroPowerBehavior(FLOAT);
            outtakeSlideMotor_left.setPower(0);
            outtakeSlideMotor_left.setPower(0);
            hangCycle = 2;
        }
        //engages pto
        if(hangCycle == 2 && hangTimer.milliseconds() > 1000 && hangTimer.milliseconds() < 1200){
            rightPto.setPosition(Robot.RIGHT_PTO_ENGAGE);
            leftPto.setPosition(Robot.LEFT_PTO_ENGAGE);
            hangTimer.reset();
            hangCycle = 3;
        }
        //goes to l2 hang
        if(hangCycle == 3 && hangTimer.milliseconds() > 500 && hangTimer.milliseconds() < 750){
            leftFrontMotor.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
            leftBackMotor.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
            rightFrontMotor.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
            rightBackMotor.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
            leftFrontMotor.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
            leftBackMotor.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
            rightFrontMotor.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
            rightBackMotor.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
//            leftFrontMotor.setTargetPosition(Robot.DRIVE_L2_HANG);
//            leftBackMotor.setTargetPosition((Robot.DRIVE_L2_HANG));
//            rightFrontMotor.setTargetPosition(Robot.DRIVE_L2_HANG);
//            rightBackMotor.setTargetPosition(Robot.DRIVE_L2_HANG);
//            leftFrontMotor.setMode(DcMotor.RunMode.RUN_TO_POSITION);
//            leftBackMotor.setMode(DcMotor.RunMode.RUN_TO_POSITION);
//            leftBackMotor.setMode(DcMotor.RunMode.RUN_TO_POSITION);
//            leftBackMotor.setMode(DcMotor.RunMode.RUN_TO_POSITION);
            leftFrontMotor.setPower(1);
            leftBackMotor.setPower(1);
            rightFrontMotor.setPower(1);
            rightBackMotor.setPower(1);
            hangCycle = 4;
        }
        //lets bot drop slowly
        if(hangCycle ==4 && outtakeSlideMotor_left.getCurrentPosition() > Robot.DRIVE_L2_HANG+15){
            leftFrontMotor.setZeroPowerBehavior(FLOAT);
            leftBackMotor.setZeroPowerBehavior(FLOAT);
            rightFrontMotor.setZeroPowerBehavior(FLOAT);
            rightBackMotor.setZeroPowerBehavior(FLOAT);
            leftFrontMotor.setPower(-0.2);
            leftBackMotor.setPower(-0.2);
            rightFrontMotor.setPower(-0.2);
            rightBackMotor.setPower(-0.2);
            hangTimer.reset();
            hangCycle = 5;

        }
        /*if(hangCycle == 5 && hangTimer.milliseconds() > 500 && hangTimer.milliseconds() < 750){
            rightPto.setPosition(Robot.RIGHT_PTO_DISENGAGE);
            leftPto.setPosition(Robot.LEFT_PTO_DISENGAGE);
            outtakeSlideMotor_left.setTargetPosition(Robot.OUTTAKE_MOTOR_L3_PREPARE);
            outtakeSlideMotor_right.setTargetPosition(Robot.OUTTAKE_MOTOR_L3_PREPARE);
            outtakeSlideMotor_left.setMode(DcMotor.RunMode.RUN_TO_POSITION);
            outtakeSlideMotor_right.setMode(DcMotor.RunMode.RUN_TO_POSITION);
            outtakeSlideMotor_left.setPower(1);
            outtakeSlideMotor_right.setPower(1);
            leftFrontMotor.setPower(0);
            leftBackMotor.setPower(0);
            rightFrontMotor.setPower(0);
            rightBackMotor.setPower(0);
            hangCycle = 6;
        }
        if(hangCycle == 6 && (outtakeSlideMotor_left.getCurrentPosition() < Robot.OUTTAKE_MOTOR_L3_PREPARE + 50)){
            rightPto.setPosition(Robot.RIGHT_PTO_ENGAGE);
            leftPto.setPosition(Robot.LEFT_PTO_ENGAGE);
            outtakeSlideMotor_left.setPower(0);
            outtakeSlideMotor_left.setPower(0);
            hangTimer.reset();
            hangCycle = 7;
        }
        if(hangCycle == 7 && hangTimer.milliseconds() > 250 && hangTimer.milliseconds() < 500){
//            leftFrontMotor.setTargetPosition(Robot.DRIVE_L3_HANG);
//            leftBackMotor.setTargetPosition((Robot.DRIVE_L3_HANG));
//            rightFrontMotor.setTargetPosition(Robot.DRIVE_L3_HANG);
//            rightBackMotor.setTargetPosition(Robot.DRIVE_L3_HANG);
//            leftFrontMotor.setMode(DcMotor.RunMode.RUN_TO_POSITION);
//            leftBackMotor.setMode(DcMotor.RunMode.RUN_TO_POSITION);
//            leftBackMotor.setMode(DcMotor.RunMode.RUN_TO_POSITION);
//            leftBackMotor.setMode(DcMotor.RunMode.RUN_TO_POSITION);
            leftFrontMotor.setPower(1);
            leftBackMotor.setPower(1);
            rightFrontMotor.setPower(1);
            rightBackMotor.setPower(1);
            hangCycle = 8;
        }
        if(hangCycle == 8 && outtakeSlideMotor_left.getCurrentPosition() > Robot.DRIVE_L3_HANG + 15){
            leftFrontMotor.setPower(0);
            leftBackMotor.setPower(0);
            rightFrontMotor.setPower(0);
            rightBackMotor.setPower(0);
            hangCycle = 9;
        }
        if(hangCycle == 9 && hangTimer.milliseconds() < 10000){
            leftFrontMotor.setPower(0);
            leftBackMotor.setPower(0);
            rightFrontMotor.setPower(0);
            rightBackMotor.setPower(0);
        }
        if(hangCycle == 9 && hangTimer.milliseconds() > 10000 && hangTimer.milliseconds() < 15000){
            leftFrontMotor.setPower(Robot.DRIVE_HANG_LOWER);
            leftBackMotor.setPower(Robot.DRIVE_HANG_LOWER);
            rightFrontMotor.setPower(Robot.DRIVE_HANG_LOWER);
            rightBackMotor.setPower(Robot.DRIVE_HANG_LOWER);
        }
        if(hangCycle == 9 && hangTimer.milliseconds() > 15000 && hangTimer.milliseconds() < 16000){
            leftFrontMotor.setPower(0);
            leftBackMotor.setPower(0);
            rightFrontMotor.setPower(0);
            rightBackMotor.setPower(0);
            }
         */






    }

}