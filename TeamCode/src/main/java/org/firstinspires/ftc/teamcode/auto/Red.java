package org.firstinspires.ftc.teamcode.auto;

import com.pedropathing.follower.Follower;
import com.pedropathing.geometry.BezierLine;
import com.pedropathing.geometry.Pose;
import com.pedropathing.paths.PathChain;
import com.pedropathing.util.Timer;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.NormalizedColorSensor;
import com.qualcomm.robotcore.hardware.PIDFCoefficients;
import com.qualcomm.robotcore.hardware.Servo;


import org.firstinspires.ftc.teamcode.auto.Shooter;
import org.firstinspires.ftc.teamcode.pedroPathing.Constants;

@Autonomous(name = "Red", group = "Red", preselectTeleOp = "Drive")
public class Red extends OpMode {

    private Follower follower;
    private Shooter shooter;
    private Timer pathTimer, opmodeTimer;
    private int pathState;

    // Subsystems
    private static final double SHOOT_DELAY = 0.35; // seconds


    private DcMotor intake;
    // Hardware Names
    private static final String INTAKE_MOTOR = "intake";
    private static final String FEEDER_SERVO = "feeder";
    private static final String SHOOTER_MOTOR = "shooter";

    // Poses
    private final Pose startPose = new Pose(122, 123, Math.toRadians(36));
    private final Pose shootPose = new Pose(100, 104, Math.toRadians(36));
    private final Pose intake2Pose = new Pose(97, 60, Math.toRadians(0));
    private final Pose feed2Pose = new Pose(116, 60, Math.toRadians(0));
    private final Pose gate = new Pose(127, 67, Math.toRadians(270));
    private final Pose intake1Pose = new Pose(47, 84, Math.toRadians(0));
    private final Pose feed1Pose = new Pose(23, 84, Math.toRadians(0));
    private final Pose intake3Pose = new Pose(47, 35, Math.toRadians(0));
    private final Pose feed3Pose = new Pose(23, 35, Math.toRadians(0));
    private final Pose parkPose = new Pose(36, 85, Math.toRadians(0));


    // Paths
    private PathChain toShoot1, intake1, feed1, toShoot2, intake2, feed2, ToGate,toShoot3,intake3,feed3,toShoot4,park;

    // Timeout for stuck paths (ms)
    private static final long PATH_TIMEOUT = 4000;

    public void buildPaths() {
        toShoot1 = follower.pathBuilder()
                .addPath(new BezierLine(startPose, shootPose))
                .setLinearHeadingInterpolation(Math.toRadians(36), Math.toRadians(36))
                .build();

        intake1 = follower.pathBuilder()
                .addPath(new BezierLine(shootPose, intake2Pose))
                .setLinearHeadingInterpolation(Math.toRadians(36), Math.toRadians(0))
                .build();

        feed1 = follower.pathBuilder()
                .addPath(new BezierLine(intake2Pose, feed2Pose))
                .setTangentHeadingInterpolation()
                .build();

        ToGate = follower.pathBuilder()
                .addPath(new BezierLine(feed2Pose, gate))
                .setLinearHeadingInterpolation(Math.toRadians(0), Math.toRadians(270))
                .build();

        toShoot2 = follower.pathBuilder()
                .addPath(new BezierLine(gate, shootPose))
                .setLinearHeadingInterpolation(Math.toRadians(270), Math.toRadians(36))
                .build();

        intake2 = follower.pathBuilder()
                .addPath(new BezierLine(shootPose, intake1Pose))
                .setLinearHeadingInterpolation(Math.toRadians(36), Math.toRadians(0))
                .build();

        feed2 = follower.pathBuilder()
                .addPath(new BezierLine(intake1Pose, feed1Pose))
                .setTangentHeadingInterpolation()
                .build();
        toShoot3 = follower.pathBuilder()
                .addPath(new BezierLine(feed1Pose, shootPose))
                .setLinearHeadingInterpolation(Math.toRadians(0), Math.toRadians(36))
                .build();
        intake3 = follower.pathBuilder()
                .addPath(new BezierLine(shootPose,intake3Pose))
                .setLinearHeadingInterpolation(Math.toRadians(36),Math.toRadians(0))
                .build();
        feed3 = follower.pathBuilder()
                .addPath(new BezierLine(intake3Pose, feed3Pose))
                .setTangentHeadingInterpolation()
                .build();
        toShoot4 = follower.pathBuilder()
                .addPath(new BezierLine(feed3Pose, shootPose))
                .setLinearHeadingInterpolation(Math.toRadians(0), Math.toRadians(36))
                .build();
        park = follower.pathBuilder()
                .addPath(new BezierLine(shootPose, parkPose))
                .setLinearHeadingInterpolation(Math.toRadians(36), Math.toRadians(0))
                .build();

    }

    public void autonomousPathUpdate() {
        switch (pathState) {
            /* ===================== MOVE TO FIRST SHOT ===================== */
            case 0:
                if (pathTimer.getElapsedTimeSeconds() < 0.05) {
                    follower.followPath(toShoot1);
                }

                if (!follower.isBusy()) {
                    pathTimer.resetTimer();
                    setPathState(1);
                }
                break;

            /* ===================== SHOOT #1 ===================== */
            case 1:
                if (shooter.isShooterReady()) {
                    shooter.feed();

                    if (pathTimer.getElapsedTimeSeconds() > SHOOT_DELAY) {
                        shooter.retractFeeder();
                        setPathState(2);
                    }
                }
                break;

            /* ===================== INTAKE → FEED 1 ===================== */
            case 2:
                if (!follower.isBusy()) {
                    follower.followPath(intake2, true);
                    setPathState(3);
                }
                break;

            case 3:
                if (!follower.isBusy()) {
                    follower.followPath(feed2, true);
                    setPathState(4);
                }
                break;

            /* ===================== MOVE TO SHOT #2 ===================== */
            case 4:
                if (!follower.isBusy()) {
                    shooter.spinUpShooter();
                    follower.setMaxPower(1);
                    follower.followPath(ToGate, true);
                    setPathState(5);
                }
                break;

            case 5:
                if (!follower.isBusy()) {
                    follower.followPath(toShoot2, true);
                    pathTimer.resetTimer();
                    setPathState(6);
                }
                break;

            /* ===================== SHOOT #2 ===================== */
            case 6:
                if (shooter.isShooterReady()) {
                    shooter.feed();

                    if (pathTimer.getElapsedTimeSeconds() > SHOOT_DELAY) {
                        shooter.retractFeeder();
                        setPathState(7);
                    }
                }
                break;

            /* ===================== INTAKE → FEED 2 ===================== */
            case 7:
                if (!follower.isBusy()) {
                    follower.followPath(intake1, true);
                    setPathState(8);
                }
                break;

            case 8:
                if (!follower.isBusy()) {
                    follower.followPath(feed1, true);
                    setPathState(9);
                }
                break;

            /* ===================== MOVE TO SHOT #3 ===================== */
            case 9:
                if (!follower.isBusy()) {
                    shooter.spinUpShooter();
                    follower.followPath(toShoot3, true);
                    pathTimer.resetTimer();
                    setPathState(10);
                }
                break;

            /* ===================== SHOOT #3 ===================== */
            case 10:
                if (shooter.isShooterReady()) {
                    shooter.feed();

                    if (pathTimer.getElapsedTimeSeconds() > SHOOT_DELAY) {
                        shooter.retractFeeder();
                        setPathState(11);
                    }
                }
                break;

            /* ===================== INTAKE → FEED 3 ===================== */
            case 11:
                if (!follower.isBusy()) {
                    follower.followPath(intake3, true);
                    setPathState(12);
                }
                break;

            case 12:
                if (!follower.isBusy()) {
                    follower.followPath(feed3, true);
                    setPathState(13);
                }
                break;

            /* ===================== MOVE TO SHOT #4 ===================== */
            case 13:
                if (!follower.isBusy()) {
                    shooter.spinUpShooter();
                    follower.followPath(toShoot4, true);
                    pathTimer.resetTimer();
                    setPathState(14);
                }
                break;

            /* ===================== SHOOT #4 ===================== */
            case 14:
                if (shooter.isShooterReady()) {
                    shooter.feed();

                    if (pathTimer.getElapsedTimeSeconds() > SHOOT_DELAY) {
                        shooter.retractFeeder();
                        setPathState(15);
                    }
                }
                break;

            /* ===================== PARK ===================== */
            case 15:
                if (!follower.isBusy()) {
                    follower.followPath(park, true);
                    setPathState(16);
                }
                break;

            /* ===================== DONE ===================== */
            case 16:
                // Auto complete
                break;
        }
    }

    public void setPathState(int pState) {
        pathState = pState;
        pathTimer.resetTimer();
    }

    @Override
    public void loop() {
        follower.update();

        autonomousPathUpdate();

        telemetry.addData("pathState",pathState);
        telemetry.update();
    }

    @Override
    public void init() {
        pathTimer = new Timer();
        opmodeTimer = new Timer();
        opmodeTimer.resetTimer();

        follower = Constants.createFollower(hardwareMap);
        follower.setStartingPose(startPose);
        buildPaths();

        Servo feederServo = hardwareMap.get(Servo.class, FEEDER_SERVO);
        DcMotorEx shooterMotor = hardwareMap.get(DcMotorEx.class, SHOOTER_MOTOR);
        shooterMotor.setDirection(DcMotorSimple.Direction.REVERSE);

        shooter = new Shooter(shooterMotor, feederServo);

        intake = hardwareMap.get(DcMotor.class, INTAKE_MOTOR);

    }

    @Override
    public void init_loop() {
    }

    @Override
    public void start() {
        opmodeTimer.resetTimer();
        shooter.spinUpShooter();
        intake.setPower(1);


        setPathState(0);
    }

    @Override
    public void stop() {
    }
}
