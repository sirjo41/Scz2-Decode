package org.firstinspires.ftc.teamcode.teleop;

import com.arcrobotics.ftclib.gamepad.GamepadEx;
import com.arcrobotics.ftclib.gamepad.GamepadKeys;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.Gamepad;
import com.qualcomm.robotcore.hardware.NormalizedColorSensor;
import com.qualcomm.robotcore.hardware.PIDFCoefficients;
import com.qualcomm.robotcore.hardware.Servo;


@TeleOp(name = "Drive", group = "TeleOp")
public class Drive extends LinearOpMode {

    // Hardware Names
    private static final String MOTOR_FL = "frontLeft";
    private static final String MOTOR_BL = "backLeft";
    private static final String MOTOR_FR = "frontRight";
    private static final String MOTOR_BR = "backRight";

    //Intake Hardware
    private static final String COLOR_SENSOR = "intakeColor";
    private static final String INTAKE_MOTOR = "intake";
    private static final String FEEDER_SERVO = "feeder";
    private static final String SHOOTER_MOTOR = "shooter";

    Gamepad.RumbleEffect shstart;
    Gamepad.RumbleEffect shstop;

    public static double TARGET_SHOOTER_RPM_CLOSE = 1400;
    public static double TARGET_SHOOTER_RPM_FAR = 1400;
    public static double TARGET_SHOOTER_RPM= 0;
    private static final double FEEDER_IDLE = 0;
    private static final double FEEDER_FEEDING = 0.55;

    public static final double SHOOTER_P = 55; // Tune: Start with 10% of F
    public static final double SHOOTER_I = 0.0;
    public static final double SHOOTER_D = 0.0;
    public static final double SHOOTER_F = 15;

    enum IntakeState {
        OFF,
        IN,
        OUT
    }

    IntakeState intakeState = IntakeState.OFF;
    @Override
    public void runOpMode() {

        // --- Hardware Initialization ---

        // Drive Motors
        DcMotorEx frontLeftMotor = hardwareMap.get(DcMotorEx.class, MOTOR_FL);
        DcMotorEx backLeftMotor = hardwareMap.get(DcMotorEx.class, MOTOR_BL);
        DcMotorEx frontRightMotor = hardwareMap.get(DcMotorEx.class, MOTOR_FR);
        DcMotorEx backRightMotor = hardwareMap.get(DcMotorEx.class, MOTOR_BR);

        frontLeftMotor.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        frontRightMotor.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        backLeftMotor.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        backRightMotor.setMode(DcMotor.RunMode.RUN_USING_ENCODER);

        frontLeftMotor.setDirection(DcMotorSimple.Direction.REVERSE);
        backLeftMotor.setDirection(DcMotorSimple.Direction.REVERSE);

        frontLeftMotor.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        frontRightMotor.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        backRightMotor.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        backLeftMotor.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);

        //System
        NormalizedColorSensor intakeSensor = hardwareMap.get(NormalizedColorSensor.class, COLOR_SENSOR);
        Servo feederServo = hardwareMap.get(Servo.class, FEEDER_SERVO);

        DcMotorEx shooterMotor = hardwareMap.get(DcMotorEx.class, SHOOTER_MOTOR);
        shooterMotor.setDirection(DcMotorSimple.Direction.REVERSE);

        shooterMotor.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        shooterMotor.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.FLOAT);

        PIDFCoefficients shooterPIDF = new PIDFCoefficients(SHOOTER_P, SHOOTER_I, SHOOTER_D, SHOOTER_F);
        shooterMotor.setPIDFCoefficients(DcMotor.RunMode.RUN_USING_ENCODER, shooterPIDF);
        // Subsystems

        GamepadEx gamepadEx = new GamepadEx(gamepad1);

        // Intake Motor
        DcMotor intake = hardwareMap.get(DcMotor.class, INTAKE_MOTOR);
        intakeSensor.setGain(2);

        shstart = new Gamepad.RumbleEffect.Builder()
                .addStep(0.5, 0.5, 500)
                .build();
        shstop = new Gamepad.RumbleEffect.Builder()
                .addStep(0.7, 0.7, 250)
                .addStep(0, 0, 50)
                .addStep(0.7, 0.7, 250)
                .build();


        telemetry.addLine("Initialized! Ready to Start.");
        telemetry.update();
        waitForStart();


        if (isStopRequested())
            return;

        while (opModeIsActive()) {

            gamepadEx.readButtons();
            // === 1. Drive Control (Gamepad 1) ===
            double y = -gamepad1.left_stick_y; // Remember, Y stick value is reversed
            double x = gamepad1.left_stick_x * 1.1; // Counteract imperfect strafing
            double rx = gamepad1.right_stick_x;

            double denominator = Math.max(Math.abs(y) + Math.abs(x) + Math.abs(rx), 1);
            double frontLeftPower = (y + x + rx) / denominator;
            double backLeftPower = (y - x + rx) / denominator;
            double frontRightPower = (y - x - rx) / denominator;
            double backRightPower = (y + x - rx) / denominator;

            frontLeftMotor.setPower(frontLeftPower);
            backLeftMotor.setPower(backLeftPower);
            frontRightMotor.setPower(frontRightPower);
            backRightMotor.setPower(backRightPower);

            // A: Intake (Run Intake)
            // Intake state changes (edge-triggered)
            if (gamepadEx.getTrigger(GamepadKeys.Trigger.LEFT_TRIGGER)> 0.2) {
                intakeState = IntakeState.IN;
                gamepad1.runRumbleEffect(shstart);
            }

            if (gamepadEx.wasJustPressed(GamepadKeys.Button.X)) {
                intakeState = IntakeState.OUT;
                gamepad1.runRumbleEffect(shstart);
            }

            if (gamepadEx.wasJustPressed(GamepadKeys.Button.LEFT_BUMPER)) {
                gamepad1.runRumbleEffect(shstop);
                intakeState = IntakeState.OFF;
            }

// Intake action (latched)
            switch (intakeState) {
                case IN:
                    intake.setPower(1);
                    break;
                case OUT:
                    intake.setPower(-1);
                    break;
                case OFF:
                    intake.setPower(0);
                    break;
            }

            if(gamepadEx.getButton(GamepadKeys.Button.A)){
                feederServo.setPosition(FEEDER_FEEDING);
            }
            else {
                feederServo.setPosition(FEEDER_IDLE);
            }
            if(gamepadEx.wasJustPressed(GamepadKeys.Button.DPAD_DOWN)){
                gamepad1.runRumbleEffect(shstart);
                TARGET_SHOOTER_RPM = TARGET_SHOOTER_RPM_CLOSE;
            }
            if(gamepadEx.wasJustPressed(GamepadKeys.Button.DPAD_UP)){
                gamepad1.runRumbleEffect(shstart);
                TARGET_SHOOTER_RPM = TARGET_SHOOTER_RPM_FAR;
            }
            if(gamepadEx.wasJustPressed((GamepadKeys.Button.RIGHT_BUMPER))){
                gamepad1.runRumbleEffect(shstop);
                TARGET_SHOOTER_RPM = 0;
            }
            shooterMotor.setVelocity(TARGET_SHOOTER_RPM);


            telemetry.addLine("=== Shooter State ===");
            telemetry.addData("Target RPM", TARGET_SHOOTER_RPM);
            telemetry.addData("Current RPM", String.format("%.0f", shooterMotor.getVelocity()));
            telemetry.addData("Shooter Ready", isShooterReady(shooterMotor.getVelocity()) ? "YES" : "NO");
            telemetry.addData("Feeder Position", String.format("%.2f", feederServo.getPosition()));
            telemetry.update();
        }
    }
    public boolean isShooterReady(double Shooter_RPM) {
        double SHOOTER_VELOCITY_TOLERANCE = 200;
    if(TARGET_SHOOTER_RPM == 0) {
            return false;
        }
        return (Math.abs(Shooter_RPM - TARGET_SHOOTER_RPM) < SHOOTER_VELOCITY_TOLERANCE);
    }

}
