package org.firstinspires.ftc.teamcode.autos;

import androidx.annotation.NonNull;
import com.acmerobotics.dashboard.config.Config;
import com.acmerobotics.dashboard.telemetry.TelemetryPacket;
import com.acmerobotics.roadrunner.Action;
import com.acmerobotics.roadrunner.ParallelAction;
import com.acmerobotics.roadrunner.Pose2d;
import com.acmerobotics.roadrunner.SequentialAction;
import com.acmerobotics.roadrunner.TrajectoryActionBuilder;
import com.acmerobotics.roadrunner.Vector2d;
import com.acmerobotics.roadrunner.ftc.Actions;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import org.firstinspires.ftc.teamcode.MecanumDrive;
import org.firstinspires.ftc.teamcode.functions.intake;
import org.firstinspires.ftc.teamcode.functions.outtake;
import org.firstinspires.ftc.teamcode.functions.horiSlides;
import org.firstinspires.ftc.teamcode.functions.vertSlide;

import java.util.concurrent.Executors;
import java.util.concurrent.ScheduledExecutorService;
import java.util.concurrent.TimeUnit;

@Config
@Autonomous(name = "FiveSpecimen", group = "Autonomous")
public class FiveSpecimen extends LinearOpMode {

    private Thread asyncUpdatesThread;
    private volatile boolean asyncThread = true;

    // Scheduled executor to handle delayed tasks
    private ScheduledExecutorService scheduler;

    @Override
    public void runOpMode() {

        // Initialize subsystems
        outtake outtakeSystem = new outtake(hardwareMap);
        intake intakeSystem = new intake(hardwareMap);
        horiSlides horizontalSlides = new horiSlides(hardwareMap);
        vertSlide verticalSlides = new vertSlide(hardwareMap);

        // Initialize the scheduler for delayed tasks
        scheduler = Executors.newScheduledThreadPool(1);

        // Define the runnable for async updates
        Runnable asyncUpdates = new Runnable() {
            @Override
            public void run() {
                while (asyncThread && !Thread.currentThread().isInterrupted()) {
                    horizontalSlides.update();
                    verticalSlides.update();
                    outtakeSystem.update();
                    intakeSystem.update();

                    // Add slight delay to avoid rapid updates causing jitter
                    try {
                        Thread.sleep(5); // Sleep for 20 ms to give hardware time to respond
                    } catch (InterruptedException e) {
                        Thread.currentThread().interrupt(); // Ensure the thread exits if interrupted
                    }
                }
            }
        };

        // Initialize but do not start the thread yet
        asyncUpdatesThread = new Thread(asyncUpdates);

        // Set the initial pose and initialize the drive system
        Pose2d initialPose = new Pose2d(-4, 62, Math.toRadians(270));
        MecanumDrive drive = new MecanumDrive(hardwareMap, initialPose);

        ////////////////////////////////  section 1    ///////////////////////////////////////
        TrajectoryActionBuilder trajectory1 = drive.actionBuilder(initialPose)
                .strafeTo(new Vector2d(-2, 28));

        Action trajectory1WithParallel = new ParallelAction(
                trajectory1.build(),
                (telemetryPacket) -> { // Run some action
                    delayedRun(() -> verticalSlides.setPosition(verticalSlides.MIN_POSITION + 450), 0);
                    return false;
                }
        );

        ////////////////////////////////  section 2    ///////////////////////////////////////

        TrajectoryActionBuilder trajectory2 = drive.actionBuilder(new Pose2d(-2, 28, Math.toRadians(270)))
                .setReversed(true)
                .waitSeconds(0.5)
                .strafeToLinearHeading(new Vector2d(-37, 50), Math.toRadians(248));

        Action trajectory2WithParallel = new ParallelAction(
                trajectory2.build(),
                (telemetryPacket) -> { // Run some action
                    delayedRun(() -> verticalSlides.setPosition(verticalSlides.MIN_POSITION + 650), 0);
                    delayedRun(() -> outtakeSystem.clawOpen(true),500);
                    delayedRun(() -> verticalSlides.setPosition(verticalSlides.MIN_POSITION), 650);
                    delayedRun(() -> intakeSystem.intakeRotatedUp(false), 1400);
                    delayedRun(() -> horizontalSlides.setPosition(600), 1500);
                    delayedRun(() -> intakeSystem.setSpeed(-1.0,1.0),1400);

                    return false;
                }


        );

        ////////////////////////////////  section 3    ///////////////////////////////////////

        TrajectoryActionBuilder trajectory3 = drive.actionBuilder(new Pose2d(-37, 50, Math.toRadians(220)))
                .waitSeconds(0.5)
                .setReversed(true)
                .strafeToLinearHeading(new Vector2d(-34, 60.75), Math.toRadians(270));

        Action trajectory3WithParallel = new ParallelAction(
                trajectory3.build(),
                (telemetryPacket) -> { // Run some action
                    delayedRun(() -> horizontalSlides.setPosition(1150),0);
                    delayedRun(() -> outtakeSystem.clawOpen(false), 0);
                    delayedRun(() -> intakeSystem.open(false), 0);
                    delayedRun(() -> outtakeSystem.hookAtIntake(true),100);
                    delayedRun(() -> outtakeSystem.clawOpen(true),400);
                    delayedRun(() -> intakeSystem.setSpeed(0,0),450);
                    delayedRun(() -> intakeSystem.intakeRotatedUp(true), 450);
                    delayedRun(() -> horizontalSlides.setPosition(25), 450);
                    delayedRun(() -> intakeSystem.setSpeed(0.7,-0.45),1400);
                    delayedRun(() -> intakeSystem.setSpeed(0,0),2000);
                    delayedRun(() -> outtakeSystem.bucketAtIntakePos(false), 1700);

                    return false;
                }
        );

        ////////////////////////////////  section 4    ///////////////////////////////////////

        TrajectoryActionBuilder trajectory4 = drive.actionBuilder(new Pose2d(-34, 60.75, Math.toRadians(270)))
                .setReversed(false)
                .waitSeconds(0.55)
                .strafeToLinearHeading(new Vector2d(3.5, 27), Math.toRadians(270));

        Action trajectory4WithParallel = new ParallelAction(
                trajectory4.build(),
                (telemetryPacket) -> { // Run some action
                    delayedRun(() -> outtakeSystem.clawOpen(false),0);
                    delayedRun(() -> verticalSlides.setPosition(verticalSlides.MIN_POSITION + 450), 500);
                    delayedRun(() -> outtakeSystem.hookAtIntake(false),550);
                    delayedRun(() -> outtakeSystem.bucketAtIntakePos(true),800);


                    return false;
                }
        );

        ////////////////////////////////  section 5    ///////////////////////////////////////

        TrajectoryActionBuilder trajectory5 = drive.actionBuilder(new Pose2d(3.5, 27, Math.toRadians(270)))
                .setReversed(true)
                //.waitSeconds(0.5)
                .strafeToLinearHeading(new Vector2d(0, 28), Math.toRadians(270))
                //.waitSeconds(0.5)
                .strafeToLinearHeading(new Vector2d(0, 34), Math.toRadians(270))
                .strafeToLinearHeading(new Vector2d(-47, 50), Math.toRadians(250));

        Action trajectory5WithParallel = new ParallelAction(
                trajectory5.build(),
                (telemetryPacket) -> { // Run some action
                    delayedRun(() -> verticalSlides.setPosition(verticalSlides.MIN_POSITION + 700), 0);
                    delayedRun(() -> outtakeSystem.clawOpen(true),500);
                    delayedRun(() -> verticalSlides.setPosition(verticalSlides.MIN_POSITION), 650);
                    delayedRun(() -> intakeSystem.intakeRotatedUp(false), 2400);
                    delayedRun(() -> horizontalSlides.setPosition(600), 2500);
                    delayedRun(() -> intakeSystem.setSpeed(-1.0,1.0),2500);

                    return false;
                }


        );

        ////////////////////////////////  section 6    ///////////////////////////////////////

        TrajectoryActionBuilder trajectory6 = drive.actionBuilder(new Pose2d(-47, 50, Math.toRadians(250)))
                .waitSeconds(0.5)
                .setReversed(true)
                .strafeToLinearHeading(new Vector2d(-34, 56), Math.toRadians(270))
                .strafeToLinearHeading(new Vector2d(-34, 60.75), Math.toRadians(270));

        Action trajectory6WithParallel = new ParallelAction(
                trajectory6.build(),
                (telemetryPacket) -> { // Run some action
                    delayedRun(() -> horizontalSlides.setPosition(1150),0);
                    delayedRun(() -> outtakeSystem.clawOpen(false), 0);
                    delayedRun(() -> intakeSystem.open(false), 0);
                    delayedRun(() -> outtakeSystem.hookAtIntake(true),100);
                    delayedRun(() -> outtakeSystem.clawOpen(true),400);
                    delayedRun(() -> intakeSystem.setSpeed(0,0),450);
                    delayedRun(() -> intakeSystem.intakeRotatedUp(true), 450);
                    delayedRun(() -> horizontalSlides.setPosition(25), 450);
                    delayedRun(() -> intakeSystem.setSpeed(0.7,-0.45),1400);
                    delayedRun(() -> intakeSystem.setSpeed(0,0),2000);
                    delayedRun(() -> outtakeSystem.bucketAtIntakePos(false), 1700);

                    return false;
                }
        );

        ////////////////////////////////  section 7    ///////////////////////////////////////

        TrajectoryActionBuilder trajectory7 = drive.actionBuilder(new Pose2d(-34, 60.75, Math.toRadians(270)))
                .setReversed(false)
                .waitSeconds(0.55)
                .strafeToLinearHeading(new Vector2d(8, 27), Math.toRadians(270));

        Action trajectory7WithParallel = new ParallelAction(
                trajectory7.build(),
                (telemetryPacket) -> { // Run some action
                    delayedRun(() -> outtakeSystem.clawOpen(false),0);
                    delayedRun(() -> verticalSlides.setPosition(verticalSlides.MIN_POSITION + 450), 500);
                    delayedRun(() -> outtakeSystem.hookAtIntake(false),550);
                    delayedRun(() -> outtakeSystem.bucketAtIntakePos(true),800);
                    return false;
                }
        );

        ////////////////////////////////  section 8    ///////////////////////////////////////

        TrajectoryActionBuilder trajectory8 = drive.actionBuilder(new Pose2d(8, 27, Math.toRadians(270)))
                .setReversed(true)
                .strafeToLinearHeading(new Vector2d(0, 27), Math.toRadians(270))
                .waitSeconds(0.5)
                .strafeToLinearHeading(new Vector2d(0, 34), Math.toRadians(270))
                .strafeToLinearHeading(new Vector2d(-44, 34), Math.toRadians(200));

        Action trajectory8WithParallel = new ParallelAction(
                trajectory8.build(),
                (telemetryPacket) -> { // Run some action
                    delayedRun(() -> verticalSlides.setPosition(verticalSlides.MIN_POSITION + 700), 0);
                    delayedRun(() -> outtakeSystem.clawOpen(true),500);
                    delayedRun(() -> verticalSlides.setPosition(verticalSlides.MIN_POSITION), 650);
                    delayedRun(() -> intakeSystem.intakeRotatedUp(false), 3400);
                    delayedRun(() -> horizontalSlides.setPosition(400), 3600);
                    delayedRun(() -> intakeSystem.setSpeed(-1.0,1.0),3600);

                    return false;
                }


        );

        Action tryingToCollect = new Action() {

            @Override
            public boolean run(@NonNull TelemetryPacket telemetryPacket) {
                long startTime = System.currentTimeMillis();
                boolean collected = false;
                String lastColour = "NA";
                int horizontalPos = 400;
                boolean horiatmax = false;
                delayedRun(() -> intakeSystem.setSpeed(-1.0,1.0),0);
                while ((System.currentTimeMillis() - startTime < 10000) && !collected && !horiatmax) { // && !collected && !isStopRequested() && !horiatmax
                    // Increment the horizontal slides by 10 each loop
                    horizontalPos += 10;
                    if (horizontalPos >=1200){
                        horizontalPos = 1200;
                        horiatmax = true;
                    }
                    horizontalSlides.setPosition(horizontalPos);
                    horizontalSlides.update();

                    // Check if the item has been collected
                    if (!"NA".equals(intakeSystem.getDetectedColor()) && !"NA".equals(lastColour)) {
                        collected = true;
                        telemetry.addData("Collection Status", "Item Collected!");
                    }
                    lastColour = intakeSystem.getDetectedColor();
                    // Add slight delay to avoid rapid looping causing jitter

                }
                if (collected) {
                    telemetry.addData("collected", "yes");
                }
                if (horiatmax) {
                    telemetry.addData("max", "yes");
                }

                if (!collected) {
                    telemetry.addData("Collection Status", "Failed to collect within time limit.");
                }
                telemetry.update();
                delayedRun(() -> intakeSystem.setSpeed(0,0),0);
                delayedRun(() -> intakeSystem.intakeRotatedUp(true), 0);
                delayedRun(() -> horizontalSlides.setPosition(25), 0);
                delayedRun(() -> outtakeSystem.bucketAtIntakePos(true), 0);


                return false;
            }



        };

        ////////////////////////////////  section 9    ///////////////////////////////////////

        TrajectoryActionBuilder trajectory9 = drive.actionBuilder(new Pose2d(-60, 45, Math.toRadians(250)))
                .waitSeconds(0.8)
                .setReversed(true)
                .strafeToLinearHeading(new Vector2d(-34, 56), Math.toRadians(270))
                .strafeToLinearHeading(new Vector2d(-34, 60.75), Math.toRadians(270));

        Action trajectory9WithParallel = new ParallelAction(
                trajectory9.build(),
                (telemetryPacket) -> { // Run some action
                    //delayedRun(() -> horizontalSlides.setPosition(1150),0);
                    delayedRun(() -> horizontalSlides.setPosition(25),0);
                    delayedRun(() -> outtakeSystem.clawOpen(false), 0);
                    delayedRun(() -> intakeSystem.open(false), 0);
                    delayedRun(() -> outtakeSystem.hookAtIntake(true),100);
                    delayedRun(() -> outtakeSystem.clawOpen(true),400);

                    delayedRun(() -> intakeSystem.setSpeed(0.7,-0.45),1000);
                    delayedRun(() -> intakeSystem.setSpeed(0,0),1300);
                    delayedRun(() -> outtakeSystem.bucketAtIntakePos(false), 1300);

                    return false;
                }
        );

        // Build actions to run trajectories sequentially
        Action runTrajectories = new SequentialAction(
                trajectory1WithParallel,
                trajectory2WithParallel,
                trajectory3WithParallel,
                trajectory4WithParallel,
                trajectory5WithParallel,
                trajectory6WithParallel,
                trajectory7WithParallel,
                trajectory8WithParallel,
                tryingToCollect,
                trajectory9WithParallel
        );

        // Prepare robot state before starting the auto sequence
        intakeSystem.intakeRotatedUp(true);
        outtakeSystem.clawOpen(false);
        outtakeSystem.bucketAtIntakePos(true);
        outtakeSystem.hookAtIntake(false);

        // Start the async updates thread
        asyncThread = true;
        asyncUpdatesThread.start();

        // Notify that the robot is ready to start
        telemetry.addData("Status", "Waiting for start");
        telemetry.update();
        while(!isStopRequested() && !opModeIsActive()) {
            if (gamepad1.x){
                outtakeSystem.clawOpen(true);
            }
            if (gamepad1.b){
                outtakeSystem.clawOpen(false);
            }
        }
        waitForStart();

        if (isStopRequested()) {
            shutdownThread();
            return;
        }

        // Run the trajectories one after the other
        Actions.runBlocking(runTrajectories);

        // Stop the async updates thread after trajectories have completed
        shutdownThread();

        // Shutdown the scheduler
        scheduler.shutdown();
    }

    // Method to schedule delayed actions
    private void delayedRun(Runnable action, long delayInMillis) {
        scheduler.schedule(action, delayInMillis, TimeUnit.MILLISECONDS);
    }

    // Method to properly stop the asyncUpdatesThread
    private void shutdownThread() {
        asyncThread = false; // Stop the loop in the async runnable
        if (asyncUpdatesThread != null) {
            asyncUpdatesThread.interrupt(); // Interrupt any sleeping or waiting thread
            try {
                asyncUpdatesThread.join(); // Wait for the thread to terminate properly
            } catch (InterruptedException e) {
                Thread.currentThread().interrupt(); // Handle interruption and ensure the main thread isn't left interrupted
            }
        }
    }
}
