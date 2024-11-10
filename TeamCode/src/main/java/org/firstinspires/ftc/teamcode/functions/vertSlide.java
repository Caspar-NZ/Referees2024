package org.firstinspires.ftc.teamcode.functions;

import com.ThermalEquilibrium.homeostasis.Controllers.Feedback.BasicPID;
import com.ThermalEquilibrium.homeostasis.Parameters.PIDCoefficients;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.DigitalChannel;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.DigitalChannel;
import com.qualcomm.robotcore.hardware.TouchSensor;

public class vertSlide {

    // Constants for PID control and range limits

    public static double MIN_POSITION = 0;
    public static double MAX_POSITION = MIN_POSITION + 1170;
    private static final double DEADZONE = 2; // Small threshold around target position
    private static final double MAX_POWER = 1.0; // Max power to prevent hitting stops
    private static final double MIN_HOLD_POWER = 0.15; // Minimum power to counteract gravity

    // Define motors and PID controller
    private DcMotor vert0;
    private DcMotor vert1;
    private double targetPosition = 0;
    TouchSensor slideTouch;

    // PID Controller
    private BasicPID pidController;

    // Constructor for initializing motors and PID controller
    public vertSlide(HardwareMap hardwareMap) {
        vert0 = hardwareMap.get(DcMotor.class, "vert0");
        vert1 = hardwareMap.get(DcMotor.class, "vert1");
        slideTouch = hardwareMap.get(TouchSensor.class, "touch");

        // Reverse direction for left motor
        vert1.setDirection(DcMotorSimple.Direction.REVERSE);

        // Reset encoders and set run mode
        vert0.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        vert1.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        vert0.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        vert1.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);

        // Adjusted PID coefficients to account for gravity
        double kP = 0.02;  // Increased proportional gain
        double kI = 0.001; // Small integral gain to counteract gravity
        double kD = 0.001; // Derivative gain for smoothing
        PIDCoefficients pidCoefficients = new PIDCoefficients(kP, kI, kD);
        pidController = new BasicPID(pidCoefficients);
    }

    // Set target position for the slides
    public void setPosition(double position) {
        targetPosition = Math.max(MIN_POSITION, Math.min(position, MAX_POSITION));
    }

    // Update method to move motors towards the target position
    public void update() {
        double currentPosition = vert0.getCurrentPosition();

        if (slideTouch.isPressed()){
            MIN_POSITION = currentPosition;
        }
        MAX_POSITION = MIN_POSITION + 1170;

        // Calculate the power needed to reach target position
        double powerReq = pidController.calculate(currentPosition, targetPosition);


        // Clamp power to prevent overshooting
        powerReq = Math.max(-MAX_POWER, Math.min(powerReq, MAX_POWER));


        if (targetPosition < MIN_POSITION+25 && currentPosition < MIN_POSITION+25) {
            vert0.setPower(0);
            vert1.setPower(0);
            return;
        }



        // Set motor powers
        vert0.setPower(-powerReq);
        vert1.setPower(-powerReq);
    }

    // Get current position of the slides
    public double getCurrentPosition() {
        return vert0.getCurrentPosition();
    }
}
