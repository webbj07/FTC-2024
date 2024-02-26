package org.firstinspires.ftc.team4100worlds.tests;

import com.acmerobotics.roadrunner.geometry.Pose2d;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import org.firstinspires.ftc.team4100worlds.pedropathing.follower.Follower;
import org.firstinspires.ftc.team4100worlds.pedropathing.pathgeneration.BezierCurve;
import org.firstinspires.ftc.team4100worlds.pedropathing.pathgeneration.BezierLine;
import org.firstinspires.ftc.team4100worlds.pedropathing.pathgeneration.BezierPoint;
import org.firstinspires.ftc.team4100worlds.pedropathing.pathgeneration.Path;
import org.firstinspires.ftc.team4100worlds.pedropathing.pathgeneration.Point;

public class BlueLeftInnerAuto extends OpMode {
    private Pose2d spikeMarkGoalPose, initialBackdropGoalPose;
    private final Pose2d startPose = new Pose2d(15.5, 61.75, Math.toRadians(270));
    private Follower follower;
    private Path scoreSpikeMark, initialScoreOnBackdrop;
    private Point backdropGoalPoint;
    private int pathState;

    public void setBackdropGoalPose() {
        spikeMarkGoalPose = new Pose2d(11.5, 35.5, Math.toRadians(270));
        initialBackdropGoalPose = new Pose2d(51, 33, Math.PI);
    }

    public void buildPaths() {
        scoreSpikeMark = new Path(new BezierCurve(new Point(startPose), new Point(spikeMarkGoalPose)));
        scoreSpikeMark.setConstantHeadingInterpolation(startPose.getHeading());
        scoreSpikeMark.setPathEndTimeout(3);

        initialScoreOnBackdrop = new Path(new BezierLine(scoreSpikeMark.getLastControlPoint(), new Point(initialBackdropGoalPose)));
        initialScoreOnBackdrop.setLinearHeadingInterpolation(scoreSpikeMark.getEndTangent().getTheta(), Math.PI, 0.5);
        initialScoreOnBackdrop.setPathEndTimeout(2.5);
    }

    public void autonomousPathUpdate() {
        switch (pathState) {
            case 1: // starts following the first path to score on the spike mark
                follower.followPath(scoreSpikeMark);
                setPathState(2);
                break;
            case 2:
                if (!follower.isBusy()) {
                    follower.holdPoint(new BezierPoint(scoreSpikeMark.getLastControlPoint()), Math.PI);
                    setPathState(3);
                }
                break;
            case 3:
                follower.followPath(initialScoreOnBackdrop);
                setPathState(4);
                break;
            case 4: // detects for end of the path and outtake out and drops pixel
                if (!follower.isBusy()) {
                    Follower.useHeading = true;
                    backdropGoalPoint = new Point(initialBackdropGoalPose);
                    follower.holdPoint(new BezierPoint(backdropGoalPoint), Math.PI);
                }
                break;
            default:
                requestOpModeStop();
                break;
        }
    }

    public void setPathState(int state) {
        pathState = state;
        autonomousPathUpdate();
    }

    @Override
    public void loop() {
        follower.update();

        autonomousPathUpdate();

        telemetry.addData("path state", pathState);
        telemetry.addData("x", follower.getPose().getX());
        telemetry.addData("y", follower.getPose().getY());
        double[] motorPowers = follower.motorPowers();
        for (int i = 0; i < motorPowers.length; i++) {
            telemetry.addData("motor " + i, motorPowers[i]);
        }
        telemetry.update();
    }

    @Override
    public void init() {
        follower = new Follower(hardwareMap);
        follower.setStartingPose(startPose);
    }

    @Override
    public void init_loop() {
        telemetry.addData("Ready!", ":)");
        telemetry.update();
    }

    @Override
    public void start() {
        setBackdropGoalPose();
        buildPaths();
        setPathState(1);
    }

    @Override
    public void stop() {
    }
}

/**
 * 8==D
 */