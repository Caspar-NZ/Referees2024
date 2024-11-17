package org.firstinspires.ftc.teamcode.teleop;

import com.qualcomm.hardware.lynx.LynxModule;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.Gamepad;

import org.firstinspires.ftc.teamcode.functions.intake;
import org.firstinspires.ftc.teamcode.functions.outtake;
import org.firstinspires.ftc.teamcode.functions.horiSlides;
import org.firstinspires.ftc.teamcode.functions.vertSlide;

import java.util.List;

@TeleOp
public class teleOp extends LinearOpMode {

    private static final double MAX_INPUT_SCALING = 200;
    private boolean horiSlideIsRunningToPos = false;
    private double horiSlidesTarget;

    private boolean vertSlideIsRunningToPos = false;

    private double vertSlidesTarget;
    Gamepad currentGamepad1 = new Gamepad();
    Gamepad currentGamepad2 = new Gamepad();

    Gamepad previousGamepad1 = new Gamepad();
    Gamepad previousGamepad2 = new Gamepad();

    public String Alliance = "Red";
    public String rejectAlliance = "Blue";
    public String otherReject = "Yellow";
    public boolean ejecting = false;
    public double blockDelay;
    public boolean runDelay = false;
    public boolean runningHomeHori = false;
    public double dropInBasketTimer;
    public boolean dropInBasketDelay = false;
    public boolean slightDelayToRotate = false;
    public double roationDelayTimer;
    public boolean intakeRotationUp = true;
    public boolean specimenInClaw;
    public boolean sampleInBasket;
    public boolean deliveringSpecimen = false;
    public double specimenDelivTimer;
    public boolean specimenRunTimer = false;
    public double sampleDelivTimer;
    public boolean specimenHooked = false;
    public boolean justDroppedSample = false;
    public boolean readyToGoToDelivery = false;
    public boolean readyToDeliver = false;

    public String previosIntakeResult;

    public String target;
    @Override
    public void runOpMode() throws InterruptedException {
        List<LynxModule> allHubs = hardwareMap.getAll(LynxModule.class);

        for (LynxModule hub : allHubs) {
            hub.setBulkCachingMode(LynxModule.BulkCachingMode.MANUAL);
        }


        // Initialize drive motors
        DcMotor leftFront = hardwareMap.dcMotor.get("leftFront");
        DcMotor leftBack = hardwareMap.dcMotor.get("leftBack");
        DcMotor rightFront = hardwareMap.dcMotor.get("rightFront");
        DcMotor rightBack = hardwareMap.dcMotor.get("rightBack");

        leftFront.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        leftBack.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        rightBack.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        rightFront.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);

        leftFront.setDirection(DcMotorSimple.Direction.REVERSE);
        leftBack.setDirection(DcMotorSimple.Direction.REVERSE);

        // Initialize outtake
        outtake outtake = new outtake(hardwareMap);
        outtake.clawOpen(false);
        outtake.hookAtIntake(true);
        outtake.bucketAtIntakePos(true);
        outtake.update();

        // Initialize intake
        intake intake = new intake(hardwareMap);
        intake.intakeRotatedUp(true);
        intakeRotationUp = true;
        intake.open(false);
        intake.update();

        // Initialize horizontal slides
        horiSlides horizontalSlides = new horiSlides(hardwareMap);

        vertSlide verticalSlides = new vertSlide(hardwareMap);

        while (!isStarted() && !isStopRequested()) {
            previousGamepad1.copy(currentGamepad1);
            previousGamepad2.copy(currentGamepad2);

            currentGamepad1.copy(gamepad1);
            currentGamepad2.copy(gamepad2);

            if ((gamepad2.start && !previousGamepad2.start) || (gamepad1.start && !previousGamepad1.start)){
                if (Alliance.equals("Red")){
                    Alliance = "Blue";
                    rejectAlliance = "Red";
                } else {
                    Alliance = "Red";
                    rejectAlliance = "Blue";
                }
            }
            telemetry.addData("Alliance" , Alliance);
            telemetry.update();
        }


        if (isStopRequested()) return;

        while (opModeIsActive()) {
            previousGamepad1.copy(currentGamepad1);
            previousGamepad2.copy(currentGamepad2);

            currentGamepad1.copy(gamepad1);
            currentGamepad2.copy(gamepad2);


            ////////////////////////////// Drive code ///////////////////////////////////
            double y = -gamepad1.left_stick_y; // Remember, Y stick value is reversed
            double x = gamepad1.left_stick_x * 1.1; // Counteract imperfect strafing
            double rx = gamepad1.right_stick_x;

            double denominator = Math.max(Math.abs(y) + Math.abs(x) + Math.abs(rx), 1);
            double leftFrontPower = (y + x + rx) / denominator;
            double leftBackPower = (y - x + rx) / denominator;
            double rightFrontPower = (y - x - rx) / denominator;
            double rightBackPower = (y + x - rx) / denominator;

            leftFront.setPower(leftFrontPower);
            leftBack.setPower(leftBackPower);
            rightFront.setPower(rightFrontPower);
            rightBack.setPower(rightBackPower);


            ////////////////////////////// Intake controls ///////////////////////////////////
            if ((Math.abs(gamepad2.right_stick_y) > 0.02) && !runningHomeHori || gamepad2.right_stick_button) {
                horiSlideIsRunningToPos = false;
                runningHomeHori=false;
            }
            if (horiSlideIsRunningToPos || runningHomeHori) {
                horizontalSlides.setPosition(horiSlidesTarget);
            } else {
                /////hori manual control
                double currentPosition = horizontalSlides.getCurrentPosition();
                double goingOutInput = Math.max(-gamepad2.right_stick_y, gamepad1.right_trigger);
                double goingInInput = Math.max(gamepad2.right_stick_y, gamepad1.left_trigger);
                double movementInput = (goingInInput > 0) ? -goingInInput : goingOutInput;
                double newTargetPosition = currentPosition + (movementInput * MAX_INPUT_SCALING);
                newTargetPosition = Math.max(horiSlides.MIN_POSITION, Math.min(newTargetPosition, horiSlides.MAX_POSITION));
                horizontalSlides.setPosition(newTargetPosition);
            }

            if ((gamepad1.start && !previousGamepad1.start) || (gamepad2.start && !previousGamepad2.start)){
                if (otherReject.equals("Yellow")){
                    otherReject = Alliance;
                } else {
                    otherReject = "Yellow";
                }
            }

            if (otherReject != "Yellow") { //target is yellow
                gamepad1.setLedColor(255, 255, 0, Gamepad.LED_DURATION_CONTINUOUS);
                gamepad2.setLedColor(255, 255, 0, Gamepad.LED_DURATION_CONTINUOUS);
                target = "Yellow";
            } else if (Alliance.equals("Red")){ //target is red
                gamepad1.setLedColor(255, 0, 0, Gamepad.LED_DURATION_CONTINUOUS);
                gamepad2.setLedColor(255, 0, 0, Gamepad.LED_DURATION_CONTINUOUS);
                target = "Red";
            } else { //target is blue
                gamepad1.setLedColor(0, 0, 255, Gamepad.LED_DURATION_CONTINUOUS);
                gamepad2.setLedColor(0, 0, 255, Gamepad.LED_DURATION_CONTINUOUS);
                target = "Blue";
            }

            if (!intakeRotationUp) {
                if ((intake.getDetectedColor().equals(rejectAlliance)&&(previosIntakeResult.equals(rejectAlliance))) || (intake.getDetectedColor().equals(otherReject)&&(previosIntakeResult.equals(otherReject)))) {
                    intake.open(true);  // Open block to eject
                    intake.setSpeed(-1.0, 1.0);
                    blockDelay = getRuntime();
                    runDelay = true;
                } else if ((blockDelay + 0.05) < getRuntime() && runDelay) {
                    intake.open(false);
                    runDelay = false;
                } else if (intake.getDetectedColor() == target && previosIntakeResult == target) {
                    intake.open(false);
                    intake.setSpeed(0.0, 0.0);
                    horizontalSlides.setPosition(0);
                    intake.intakeRotatedUp(true);
                    runningHomeHori = true;
                } else {
                    intake.open(false);
                    intake.setSpeed(-gamepad2.left_trigger, gamepad2.right_trigger);
                }
            }
            previosIntakeResult = intake.getDetectedColor();

            if (gamepad1.right_trigger> 0.1){
                intake.setSpeed(-1,1);
            }

            if (runningHomeHori && horizontalSlides.getCurrentPosition() < 50) {
                dropInBasketTimer = getRuntime();
                dropInBasketDelay = true;
                runningHomeHori=false;
                intake.setSpeed(0.7, -0.7);
                if (otherReject != "Yellow"){
                    sampleInBasket=true;
                }
            }
            if (dropInBasketDelay && (dropInBasketTimer + 0.6) < getRuntime()) {
                intake.setSpeed(0.0, 0.0);
                dropInBasketDelay = false;
            }

            if ((gamepad1.a || gamepad2.right_bumper) && !slightDelayToRotate){
                roationDelayTimer = getRuntime();
                slightDelayToRotate = true;
                if (intakeRotationUp){
                    intakeRotationUp = false;
                    intake.intakeRotatedUp(false);
                    intake.setSpeed(1.0, -1.0);
                } else {
                    intakeRotationUp = true;
                    intake.intakeRotatedUp(true);
                    intake.setSpeed(0.0, 0.0);
                }
            }
            if (slightDelayToRotate && !intakeRotationUp){
                intake.setSpeed(1.0, -1.0);
            }
            if (slightDelayToRotate && (roationDelayTimer + 0.4) < getRuntime()){
                slightDelayToRotate = false;
            }


            ////////////////////////////// Outtake controls ///////////////////////////////////
            if (Math.abs(gamepad2.left_stick_y) > 0.02) {
                vertSlideIsRunningToPos = false;
            } else {
                vertSlideIsRunningToPos = true;
            }
            if (vertSlideIsRunningToPos){
                verticalSlides.setPosition(vertSlidesTarget);
            } else{
                /////vert manual control
                double currentPositionvert = verticalSlides.getCurrentPosition();
                vertSlidesTarget = currentPositionvert + -gamepad2.left_stick_y * MAX_INPUT_SCALING;
                vertSlidesTarget = Math.max(vertSlide.MIN_POSITION, Math.min(vertSlidesTarget, vertSlide.MAX_POSITION));
                verticalSlides.setPosition(vertSlidesTarget);
            }

            ////////////////////Outake Yellow handling Logic ////////////////
            if (otherReject != "Yellow"){
                if ((gamepad1.dpad_up && !previousGamepad1.dpad_up) || (gamepad2.dpad_up && !previousGamepad2.dpad_up)){
                    outtake.hookAtIntake(true);
                    vertSlideIsRunningToPos = true;
                    vertSlidesTarget = verticalSlides.MAX_POSITION;
                }
            }

            ////////////////////Outake Alliance handling Logic ////////////////
            if (otherReject == "Yellow"){
                if ((!gamepad1.left_bumper && previousGamepad1.left_bumper)){
                    outtake.clawOpen(false);
                    specimenInClaw = true;
                    vertSlideIsRunningToPos = true;
                    vertSlidesTarget = verticalSlides.MIN_POSITION +150;
                    readyToGoToDelivery = true;
                }
                if (gamepad1.right_bumper && previousGamepad1.right_bumper){
                    outtake.hookAtIntake(true);
                    outtake.clawOpen(true);
                    specimenInClaw = false;
                    vertSlideIsRunningToPos = true;
                    vertSlidesTarget = verticalSlides.MIN_POSITION;
                    readyToGoToDelivery = false;
                }

                if ((gamepad1.dpad_up && !previousGamepad1.dpad_up)||(gamepad2.dpad_up && !previousGamepad2.dpad_up)){
                    if (readyToGoToDelivery){
                        outtake.hookAtIntake(false);
                        vertSlideIsRunningToPos = true;
                        vertSlidesTarget = verticalSlides.MIN_POSITION +450;
                        readyToGoToDelivery = false;
                        readyToDeliver = true;
                        sampleDelivTimer = getRuntime();
                    } else if (readyToDeliver && sampleDelivTimer + 1.0 < getRuntime()){
                        vertSlideIsRunningToPos = true;
                        vertSlidesTarget = verticalSlides.MIN_POSITION +575;
                        deliveringSpecimen = true;
                        readyToDeliver = false;
                        specimenHooked = true;
                    }
                }
                if (((gamepad1.dpad_down && !previousGamepad1.dpad_down) || (gamepad2.dpad_down && !previousGamepad2.dpad_down))&& specimenHooked){
                    outtake.clawOpen(true);
                    specimenHooked = false;
                    specimenDelivTimer = getRuntime();
                    specimenRunTimer = true;
                }
                if (specimenRunTimer && specimenDelivTimer +0.2 <getRuntime()){
                    vertSlideIsRunningToPos = true;
                    vertSlidesTarget = verticalSlides.MIN_POSITION +200;
                }
                if (specimenRunTimer && specimenDelivTimer +0.5 <getRuntime()){
                    outtake.clawOpen(false);
                }
                if (specimenRunTimer && specimenDelivTimer +0.7 <getRuntime()){
                    outtake.clawOpen(false);
                    outtake.hookAtIntake(true);
                    vertSlideIsRunningToPos = true;
                    vertSlidesTarget = verticalSlides.MIN_POSITION;
                }
                if (specimenRunTimer && specimenDelivTimer +0.9 <getRuntime()){
                    outtake.clawOpen(true);
                    specimenRunTimer = false;
                }


            }



            if (gamepad1.dpad_right || gamepad2.dpad_right){
                outtake.bucketAtIntakePos(false);
            }
            if (gamepad1.dpad_left || gamepad2.dpad_left){
                outtake.bucketAtIntakePos(true);
            }




            intake.update();
            horizontalSlides.update();
            verticalSlides.update();
            outtake.update();

            telemetry.addData("Sample in bucket", sampleInBasket);
            telemetry.addData("Vert current", verticalSlides.getCurrentPosition());
            telemetry.addData("Vert min pos", verticalSlides.MIN_POSITION);
            telemetry.addData("Vert target", vertSlidesTarget);
            telemetry.addData("Current intake", intake.getDetectedColor());
            telemetry.addData("Target colour", target);
            telemetry.update();

            for (LynxModule hub : allHubs) {
                hub.clearBulkCache();
            }
        }
    }
}
