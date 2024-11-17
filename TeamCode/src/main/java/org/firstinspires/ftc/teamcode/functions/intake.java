package org.firstinspires.ftc.teamcode.functions;

import com.qualcomm.robotcore.hardware.CRServo;  // Continuous rotation servos
import com.qualcomm.robotcore.hardware.DigitalChannel;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.Servo;


public class intake {
    // Servo declarations
    private CRServo intake0;
    private CRServo intake1;
    private Servo blocker;
    private Servo rightDown;
    private Servo leftDown;
    private DigitalChannel pin0;
    private DigitalChannel pin1;
    public double rightServoPos = 0;
    public double leftServoPos = 0;
    public double blockPos;
    public double intake0Power = 0;
    public double intake1Power = 0;
    private static double left_down = 0.03;
    private static double right_down = 0.97;
    private static double left_up = 0.97;
    private static double right_up = 0.03;
    private static double block_closed = 0.5;
    private static double block_open = 0.16;

    // Constructor
    public intake(HardwareMap hardwareMap) {
        // Initialize the hardware variables
        intake0 = hardwareMap.get(CRServo.class, "intake0");
        intake1 = hardwareMap.get(CRServo.class, "intake1");
        blocker = hardwareMap.get(Servo.class, "blocker");
        rightDown = hardwareMap.get(Servo.class, "rightDown");
        leftDown = hardwareMap.get(Servo.class, "leftDown");
        pin0 = hardwareMap.digitalChannel.get("digital0");
        pin1 = hardwareMap.digitalChannel.get("digital1");
    }

    // Method to process the intake logic
    public void update() {
        rightDown.setPosition(rightServoPos);
        leftDown.setPosition(leftServoPos);
        blocker.setPosition(blockPos);
        intake0.setPower(intake0Power);
        intake1.setPower(intake1Power);
    }

    public String getDetectedColor() {

        boolean pin0State = pin0.getState();
        boolean pin1State = pin1.getState();

        String color = "";
        if (pin0State && pin1State) {
            color = "Yellow"; // Pin0 True, Pin1 True
        } else if (pin0State && !pin1State) {
            color = "Blue"; // Pin0 True, Pin1 False
        } else if (!pin0State && pin1State) {
            color = "Red"; // Pin0 False, Pin1 True
        } else {
            color = "NA"; // Pin0 False, Pin1 False
        }
        return color;
    }

    public void intakeRotatedUp(boolean up) {
        if (up){
            rightServoPos = right_up;
            leftServoPos =  left_up;
        } else {
            rightServoPos = right_down;
            leftServoPos =  left_down;
        }
    }
    public void setSpeed(double left, double right) {
        intake0Power = right;
        intake1Power = left;
    }
    public void open(boolean passthrough) {
        if (passthrough){
            blockPos = block_open;
        } else {
            blockPos = block_closed;
        }
    }
}
