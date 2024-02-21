package org.firstinspires.ftc.team4100worlds.tests;


import static org.firstinspires.ftc.team4100worlds.RobotConstants.INTAKE_CLAW_CLOSE_TIME;
import static org.firstinspires.ftc.team4100worlds.RobotConstants.OUTTAKE_CLAW_DROP_TIME;
import static org.firstinspires.ftc.team4100worlds.RobotConstants.ROBOT_BACK_LENGTH;
import static org.firstinspires.ftc.team4100worlds.RobotConstants.ROBOT_FRONT_LENGTH;
import com.acmerobotics.roadrunner.geometry.Pose2d;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import org.firstinspires.ftc.team4100worlds.pedropathing.follower.Follower;
import org.firstinspires.ftc.team4100worlds.pedropathing.pathgeneration.BezierCurve;
import org.firstinspires.ftc.team4100worlds.pedropathing.pathgeneration.BezierLine;
import org.firstinspires.ftc.team4100worlds.pedropathing.pathgeneration.BezierPoint;
import org.firstinspires.ftc.team4100worlds.pedropathing.pathgeneration.MathFunctions;
import org.firstinspires.ftc.team4100worlds.pedropathing.pathgeneration.Path;
import org.firstinspires.ftc.team4100worlds.pedropathing.pathgeneration.PathChain;
import org.firstinspires.ftc.team4100worlds.pedropathing.pathgeneration.Point;
import org.firstinspires.ftc.team4100worlds.pedropathing.util.SingleRunAction;
import org.firstinspires.ftc.team4100worlds.util.Timer;

import java.util.ArrayList;

public class BlueLeftInnerAuto extends OpMode {
    private Timer pathTimer, opmodeTimer, scanTimer;
    private String navigation;

    private SingleRunAction foldUp;

    // IMPORTANT: y increasing is towards the backstage from the audience,
    // while x increasing is towards the red side from the blue side
    // this means that 0 heading is pointing from the blue side to the red side

    // all spike mark locations since I'm lazy
    private final Pose2d redLeftSideLeftSpikeMark = new Pose2d(36+72,-47.5+72);
    private final Pose2d redLeftSideMiddleSpikeMark = new Pose2d(24.5+72,-36+72);
    private final Pose2d redLeftSideRightSpikeMark = new Pose2d(36+72,-24.5+72);
    private final Pose2d redRightSideLeftSpikeMark = new Pose2d(36+72, 0.5+72);
    private final Pose2d redRightSideMiddleSpikeMark = new Pose2d(24.5+72, 12+72);
    private final Pose2d redRightSideRightSpikeMark = new Pose2d(36+72, 23.5+72);
    private final Pose2d blueLeftSideLeftSpikeMark = new Pose2d(-36+72, 23.5+72);
    private final Pose2d blueLeftSideMiddleSpikeMark = new Pose2d(-24.5+72, 12+72);
    private final Pose2d blueLeftSideRightSpikeMark = new Pose2d(-36+72, 0.5+72);
    private final Pose2d blueRightSideLeftSpikeMark = new Pose2d(-36+72, -24.5+72);
    private final Pose2d blueRightSideMiddleSpikeMark = new Pose2d(-24.5+72, -36+72);
    private final Pose2d blueRightSideRightSpikeMark = new Pose2d(-36+72, -47.5+72);

    // backdrop april tag locations
    private final Pose2d blueLeftBackdrop = new Pose2d(-42.875+72, 60.75+72);
    private final Pose2d blueMiddleBackdrop = new Pose2d(-36.75+72, 60.75+72);
    private final Pose2d blueRightBackdrop = new Pose2d(-30.75+72, 60.75+72);
    private final Pose2d redLeftBackdrop = new Pose2d(30.75+72, 60.75+72);
    private final Pose2d redMiddleBackdrop = new Pose2d(36.75+72, 60.75+72);
    private final Pose2d redRightBackdrop = new Pose2d(42.875+72, 60.75+72);

    // white pixel stack locations
    private final Pose2d redOuterStack = new Pose2d(36+72, -72+72);
    private final Pose2d redMiddleStack = new Pose2d(24+72, -72+72);
    private final Pose2d redInnerStack = new Pose2d(12+72,-72+72);
    private final Pose2d blueInnerStack = new Pose2d(-12+72,-72+72);
    private final Pose2d blueMiddleStack = new Pose2d(-24+72, -72+72);
    private final Pose2d blueOuterStack = new Pose2d(-36+72, -72+72);

    private Pose2d spikeMarkGoalPose, initialBackdropGoalPose, firstCycleStackPose, firstCycleBackdropGoalPose, secondCycleStackPose, secondCycleBackdropGoalPose;

    // TODO: adjust this for each auto
    private final Pose2d startPose = new Pose2d(144-(61.75+72), 15.5+72, 0);

    // TODO: dont forget to adjust this too
    private final Point abortPoint = new Point(144-83.5, 120, Point.CARTESIAN);
    private Point backdropGoalPoint;

    private Follower follower;

    private Path scoreSpikeMark, initialScoreOnBackdrop;
    private PathChain firstCycleToStack, firstCycleStackGrab, firstCycleScoreOnBackdrop, secondCycleToStack, secondCycleStackGrab, secondCycleScoreOnBackdrop;

    private int pathState, distanceSensorDisconnectCycleCount, detectDistanceSensorDisconnect;

    private ArrayList<Boolean> distanceSensorDisconnects;

    public void setBackdropGoalPose() {
        switch (navigation) {
            default:
            case "left":
                spikeMarkGoalPose = new Pose2d(blueLeftSideLeftSpikeMark.getX() + 0.5, blueLeftSideLeftSpikeMark.getY(), Math.PI/2);
                initialBackdropGoalPose = new Pose2d(blueLeftBackdrop.getX() - 2, blueLeftBackdrop.getY()-ROBOT_BACK_LENGTH-0.75, Math.PI * 1.5);
                firstCycleBackdropGoalPose = new Pose2d(blueRightBackdrop.getX() - 1, blueRightBackdrop.getY()-ROBOT_BACK_LENGTH+0.75, Math.PI * 1.5);
                secondCycleBackdropGoalPose = new Pose2d(blueRightBackdrop.getX() - 1, blueRightBackdrop.getY()-ROBOT_BACK_LENGTH+0.75, Math.PI * 1.5);
                break;
            case "middle":
                spikeMarkGoalPose = new Pose2d(blueLeftSideMiddleSpikeMark.getX() - 1.25, blueLeftSideMiddleSpikeMark.getY()+3, Math.PI/2);
                initialBackdropGoalPose = new Pose2d(blueMiddleBackdrop.getX() - 1.25, blueMiddleBackdrop.getY()-ROBOT_BACK_LENGTH-1,Math.PI * 1.5);
                firstCycleBackdropGoalPose = new Pose2d(blueRightBackdrop.getX()+0.5, blueRightBackdrop.getY()-ROBOT_BACK_LENGTH+0.5, Math.PI * 1.5);
                secondCycleBackdropGoalPose = new Pose2d(blueRightBackdrop.getX()+0.5, blueRightBackdrop.getY()-ROBOT_BACK_LENGTH+0.75, Math.PI * 1.5);
                break;
            case "right":
                spikeMarkGoalPose = new Pose2d(blueLeftSideRightSpikeMark.getX() + 2, blueLeftSideRightSpikeMark.getY()+0.5, Math.PI/2);
                initialBackdropGoalPose = new Pose2d(144-(26+72),52.75+72, Math.PI * 1.5);
                firstCycleBackdropGoalPose = new Pose2d(144-(26+72),52.75+72, Math.PI * 1.5);
                secondCycleBackdropGoalPose = new Pose2d(144-(26+72),52.75+72, Math.PI * 1.5);
                break;
        }
    }

    public void buildPaths() {
        Point scoreSpikeMarkMidPoint;
        double scoreSpikeMarkMidToSpikeDistance;
        switch (navigation) {
            default:
            case "left":
                scoreSpikeMarkMidPoint = new Point(144-131.5, 82, Point.CARTESIAN);
                break;
            case "middle":
                scoreSpikeMarkMidPoint = new Point(144-129.5, 106, Point.CARTESIAN);
                break;
            case "right":
                scoreSpikeMarkMidPoint = new Point(144-(37+72), 82, Point.CARTESIAN);
                break;
        }
        scoreSpikeMarkMidToSpikeDistance = MathFunctions.distance(spikeMarkGoalPose, scoreSpikeMarkMidPoint);
        scoreSpikeMark = new Path(new BezierCurve(new Point(startPose), scoreSpikeMarkMidPoint, new Point(spikeMarkGoalPose.getX() + MathFunctions.getSign(scoreSpikeMarkMidPoint.getX() - spikeMarkGoalPose.getX()) * Math.abs(scoreSpikeMarkMidPoint.getX() - spikeMarkGoalPose.getX()) * ROBOT_FRONT_LENGTH / scoreSpikeMarkMidToSpikeDistance, spikeMarkGoalPose.getY() + MathFunctions.getSign(scoreSpikeMarkMidPoint.getY() - spikeMarkGoalPose.getY()) * Math.abs(scoreSpikeMarkMidPoint.getY() - spikeMarkGoalPose.getY()) * ROBOT_FRONT_LENGTH / scoreSpikeMarkMidToSpikeDistance, Point.CARTESIAN)));
        scoreSpikeMark.setConstantHeadingInterpolation(startPose.getHeading());
        scoreSpikeMark.setPathEndTimeout(3);

        switch (navigation) {
            default:
            case "left":
                initialScoreOnBackdrop = new Path(new BezierCurve(scoreSpikeMark.getLastControlPoint(), new Point(144-135, 98, Point.CARTESIAN), new Point(initialBackdropGoalPose.getX(), 106, Point.CARTESIAN), new Point(initialBackdropGoalPose)));
                break;
            case "middle":
                initialScoreOnBackdrop = new Path(new BezierLine(scoreSpikeMark.getLastControlPoint(), new Point(initialBackdropGoalPose)));
                break;
            case "right":
//                initialScoreOnBackdrop = new Path(new BezierCurve(scoreSpikeMark.getLastControlPoint(), new Point(scoreSpikeMark.getLastControlPoint().getX(), 98, Point.CARTESIAN), new Point(initialBackdropGoalPose.getX(), 109.5, Point.CARTESIAN), new Point(initialBackdropGoalPose)));
                initialScoreOnBackdrop = new Path(new BezierLine(scoreSpikeMark.getLastControlPoint(), new Point(initialBackdropGoalPose)));
                break;
        }
        //initialScoreOnBackdrop.setConstantHeadingInterpolation(Math.PI * 1.5);
        initialScoreOnBackdrop.setLinearHeadingInterpolation(scoreSpikeMark.getEndTangent().getTheta(), Math.PI * 1.5, 0.5);
        initialScoreOnBackdrop.setPathEndTimeout(2.5);

        switch (navigation) {
            default:
            case "left":
                firstCycleStackPose = new Pose2d(blueInnerStack.getX()+2.75, blueInnerStack.getY() + ROBOT_FRONT_LENGTH+2.5, Math.PI * 1.5 + Math.toRadians(4));
                secondCycleStackPose = new Pose2d(blueInnerStack.getX()+2, blueInnerStack.getY() + ROBOT_FRONT_LENGTH+1, Math.PI * 1.5 + Math.toRadians(3));
                break;
            case "middle":
                firstCycleStackPose = new Pose2d(blueInnerStack.getX()+0.5, blueInnerStack.getY() + ROBOT_FRONT_LENGTH+0.5, Math.PI * 1.5 + Math.toRadians(3.5));
                secondCycleStackPose = new Pose2d(blueInnerStack.getX()+1, blueInnerStack.getY() + ROBOT_FRONT_LENGTH-0.25, Math.PI * 1.5 + Math.toRadians(2.5));
                break;
            case "right":
                firstCycleStackPose = new Pose2d(blueInnerStack.getX()+3.5, blueInnerStack.getY() + ROBOT_FRONT_LENGTH, Math.PI * 1.5 + Math.toRadians(3));
                secondCycleStackPose = new Pose2d(blueInnerStack.getX()+5, blueInnerStack.getY() + ROBOT_FRONT_LENGTH - 0.5, Math.PI * 1.5 + Math.toRadians(0));
                break;
        }

        firstCycleToStack = follower.pathBuilder()
                .addPath(new BezierCurve(new Point(initialBackdropGoalPose), new Point(144-76.5, 106, Point.CARTESIAN), new Point(firstCycleStackPose.getX()+1, 79, Point.CARTESIAN)))
                .setConstantHeadingInterpolation(firstCycleStackPose.getHeading())
                .addPath(new BezierLine(new Point(firstCycleStackPose.getX()+1, 79, Point.CARTESIAN), new Point(firstCycleStackPose.getX(), 23, Point.CARTESIAN)))
                .setConstantHeadingInterpolation(firstCycleStackPose.getHeading())
                .setPathEndTimeout(0)
                .build();

        firstCycleStackGrab = follower.pathBuilder()
                .addPath(new BezierLine(new Point(firstCycleStackPose.getX()+0.0001, 32, Point.CARTESIAN), new Point(firstCycleStackPose)))
                .setConstantHeadingInterpolation(firstCycleStackPose.getHeading())
                .build();

        firstCycleScoreOnBackdrop = follower.pathBuilder()
                .addPath(new BezierLine(new Point(firstCycleStackPose), new Point(firstCycleStackPose.getX()+1, 79, Point.CARTESIAN)))
                .setConstantHeadingInterpolation(firstCycleStackPose.getHeading())
                .addPath(new BezierCurve(new Point(firstCycleStackPose.getX()+1, 79, Point.CARTESIAN), new Point(144-76.5, 106, Point.CARTESIAN), new Point(firstCycleBackdropGoalPose)))
                .setConstantHeadingInterpolation(Math.PI * 1.5)
                .setPathEndTimeout(2.5)
                .build();

        secondCycleToStack = follower.pathBuilder()
                .addPath(new BezierCurve(new Point(firstCycleBackdropGoalPose), new Point(144-76.5, 106, Point.CARTESIAN), new Point(secondCycleStackPose.getX()+1, 79, Point.CARTESIAN)))
                .setConstantHeadingInterpolation(secondCycleStackPose.getHeading())
                .addPath(new BezierLine(new Point(secondCycleStackPose.getX()+1, 79, Point.CARTESIAN), new Point(secondCycleStackPose.getX(), 23, Point.CARTESIAN)))
                .setConstantHeadingInterpolation(secondCycleStackPose.getHeading())
                .setPathEndTimeout(0)
                .build();

        secondCycleStackGrab = follower.pathBuilder()
                .addPath(new BezierLine(new Point(secondCycleStackPose.getX()+0.0001, 32, Point.CARTESIAN), new Point(secondCycleStackPose)))
                .setConstantHeadingInterpolation(secondCycleStackPose.getHeading())
                .build();

        secondCycleScoreOnBackdrop = follower.pathBuilder()
                .addPath(new BezierLine(new Point(secondCycleStackPose), new Point(secondCycleStackPose.getX()+1, 79, Point.CARTESIAN)))
                .setConstantHeadingInterpolation(secondCycleStackPose.getHeading())
                .addPath(new BezierCurve(new Point(secondCycleStackPose.getX()+1, 79, Point.CARTESIAN), new Point(144-76.5, 106, Point.CARTESIAN), new Point(secondCycleBackdropGoalPose)))
                .setConstantHeadingInterpolation(Math.PI * 1.5)
                .setPathEndTimeout(2.5)
                .build();
    }

    public void autonomousPathUpdate() {
        switch (pathState) {
            case 10: // starts following the first path to score on the spike mark
                follower.followPath(scoreSpikeMark);
                setPathState(11);
                break;
            case 11: // detects the path to progress away from the wall and sets tangent interpolation
                if (follower.getCurrentTValue() > 0.1) {
                    //scoreSpikeMark.setReversed(false);
                    scoreSpikeMark.setLinearHeadingInterpolation(startPose.getHeading() - 0.1 * MathFunctions.getTurnDirection(startPose.getHeading(), scoreSpikeMark.getEndTangent().getTheta()) * MathFunctions.getSmallestAngleDifference(startPose.getHeading(), scoreSpikeMark.getEndTangent().getTheta()), scoreSpikeMark.getEndTangent().getTheta());
                    setPathState(12);
                }
                break;
            case 12:
                if (!follower.isBusy()) {
                    follower.holdPoint(new BezierPoint(scoreSpikeMark.getLastControlPoint()), scoreSpikeMark.getEndTangent().getTheta());
                    setPathState(13);
                }
                break;
            case 13: // detects for the end of the path and everything else to be in order and releases the pixel
                setPathState(14);
                break;
            case 14: // moves mechanisms into position to score and pick up from stack as well as starts moving to score
                setPathState(15);
                break;
            case 15:
                if (pathTimer.getElapsedTime() > 500) {
                    follower.followPath(initialScoreOnBackdrop);
                    setPathState(16);
                }
                break;
            case 16: // detects for end of the path and outtake out and drops pixel
                if (!follower.isBusy()) {
                    Follower.useHeading = true;
                    backdropGoalPoint = new Point(initialBackdropGoalPose);
                    follower.holdPoint(new BezierPoint(backdropGoalPoint), Math.PI * 1.5);
                    setPathState(17);
                }
                break;
            case 17:
                setPathState(18);
                break;
            case 18:
                setPathState(19);
                break;
            case 19: // detects for end of the path and outtake out and drops pixel
                if (pathTimer.getElapsedTime() > 700) {
                    setPathState(110);
                }
                break;
            case 110:
                if (pathTimer.getElapsedTime() > 500) {
                    setPathState(20);
                }
                break;


            case 20: // starts the robot off on to the first stack once the pixels have been dropped
                if (pathTimer.getElapsedTime() > OUTTAKE_CLAW_DROP_TIME) {
                    setPathState(21);
                }
                break;
            case 21:
                if (!follower.isBusy()) {
                    follower.holdPoint(new BezierPoint(new Point(firstCycleToStack.getPath(1).getLastControlPoint().getX(), firstCycleToStack.getPath(1).getLastControlPoint().getY() + 1, Point.CARTESIAN)), firstCycleStackPose.getHeading());
                    setPathState(25);
                }
                break;
            case 22:
                setPathState(50);
                break;
            case 23:
                setPathState(24);
                break;
            case 24:
                setPathState(25);
                break;
            case 25:
                if (pathTimer.getElapsedTime() > 300) {
                    follower.followPath(firstCycleStackGrab);
                    setPathState(26);
                }
                break;
            case 26:
                if (follower.getCurrentTValue() > 0.92) {//!follower.isBusy()) {
                    //Follower.useHeading = false;
                    //follower.holdPoint(new BezierPoint(new Point(firstCycleStackPose)), Math.PI * 1.5);
                    setPathState(27);
                }
                if (pathTimer.getElapsedTime() > 1000) {
                    setPathState(27);
                }
                break;
            case 27: // waits for the intake claw to close and then sets the intake to move back in while pulling the extension back in slightly
                if (pathTimer.getElapsedTime() > INTAKE_CLAW_CLOSE_TIME) {
                    follower.poseUpdater.resetOffset();
                    Follower.useHeading = true;
                    follower.followPath(firstCycleScoreOnBackdrop);
                    setPathState(28);
                }
                break;
            case 28:
                if (((follower.getCurrentPathNumber() == 1 && follower.getCurrentTValue() > 0.1) || !follower.isBusy())) {
                    setPathState(29);
                }
                break;
            case 29: // detects for end of the path and outtake out and drops pixel
                if (follower.atParametricEnd()) {
                    //Follower.useHeading = false;
                    backdropGoalPoint = new Point(firstCycleBackdropGoalPose);
                    follower.holdPoint(new BezierPoint(backdropGoalPoint), Math.PI * 1.5);
                    setPathState(210);
                }
                break;
            case 210:
                setPathState(211);
                break;
            case 211:
                setPathState(212);
                break;
            case 212:
                if (pathTimer.getElapsedTime() > 300) {
                    setPathState(213);
                }
                break;
            case 213:
                setPathState(214);
                /*
                if (pathTimer.getElapsedTime() > OUTTAKE_CLAW_DROP_TIME) {
                    twoPersonDrive.setOuttakeArmInterpolation(OUTTAKE_ARM_OUT_POSITION, 300);
                    setPathState(214);
                }
                 */
                break;
            case 214:
                setPathState(215);
                break;
            case 215:
                if (pathTimer.getElapsedTime() > 300) {
                    setPathState(216);
                }
                break;
            case 216:
                setPathState(217);
                break;
            case 217: // once the outer pixel has dropped, drop the inner one and fold up
                if (pathTimer.getElapsedTime() > 2 * OUTTAKE_CLAW_DROP_TIME) {
                    setPathState(30);
                }
                break;


            case 30: // once the inner pixel has dropped, start the robot off to the second pass on the first stack
                if (pathTimer.getElapsedTime() > OUTTAKE_CLAW_DROP_TIME) {
                    Follower.useHeading = true;
                    follower.poseUpdater.resetOffset();
                    setPathState(50);
                    break;
                }
                break;
            case 31:
                if (!follower.isBusy()) {
                    //startDistanceSensorDisconnectDetection(1);
                    follower.holdPoint(new BezierPoint(new Point(secondCycleToStack.getPath(1).getLastControlPoint().getX(), secondCycleToStack.getPath(1).getLastControlPoint().getY() + 1, Point.CARTESIAN)), secondCycleStackPose.getHeading());
                    setPathState(32);
                }
                break;
            case 32:
                setPathState(50);
                break;
            case 33:
                setPathState(34);
                break;
            case 34:
                setPathState(35);
                break;
            case 35:
                if (pathTimer.getElapsedTime() > 300) {
                    follower.followPath(secondCycleStackGrab);
                    setPathState(36);
                }
                break;
            case 36:
                if (follower.getCurrentTValue() > 0.92) {//!follower.isBusy()) {
                    //Follower.useHeading = false;
                    //follower.holdPoint(new BezierPoint(new Point(secondCycleStackPose)), Math.PI * 1.5);
                    setPathState(37);
                }
                if (pathTimer.getElapsedTime() > 1000) {
                    setPathState(37);
                }
                break;
            case 37: // waits for the intake claw to close and then sets the intake to move back in while pulling the extension back in slightly
                if (pathTimer.getElapsedTime() > INTAKE_CLAW_CLOSE_TIME) {
                    Follower.useHeading = true;
                    follower.poseUpdater.resetOffset();
                    follower.followPath(secondCycleScoreOnBackdrop);
                    setPathState(38);
                }
                break;
            case 38:
                if (((follower.getCurrentPathNumber() == 1 && follower.getCurrentTValue() > 0.1) || !follower.isBusy())) {
                    setPathState(39);
                }
                break;
            case 39: // detects for end of the path and outtake out and drops pixel
                if (follower.atParametricEnd()) {
                    //twoPersonDrive.moveToCustomIntakeOutPosition(INTAKE_ARM_AUTO_AVOID_POSITION);
                    //Follower.useHeading = false;
                    backdropGoalPoint = new Point(secondCycleBackdropGoalPose);
                    follower.holdPoint(new BezierPoint(backdropGoalPoint), Math.PI * 1.5);
                    setPathState(310);
                }
                break;
            case 310:
                setPathState(311);
                break;
            case 311:
                setPathState(312);
                break;
            case 312:
                if (pathTimer.getElapsedTime() > 300) {
                    setPathState(313);
                }
                break;
            case 313:
                setPathState(314);
                /*
                if (pathTimer.getElapsedTime() > OUTTAKE_CLAW_DROP_TIME) {
                    twoPersonDrive.setOuttakeArmInterpolation(OUTTAKE_ARM_OUT_POSITION, 300);
                    setPathState(314);
                }
                 */
                break;
            case 314:
                setPathState(315);
                break;
            case 315:
                if (pathTimer.getElapsedTime() > 300) {
                    setPathState(316);
                }
                break;
            case 316:
                setPathState(317);
                break;
            case 317: // once the outer pixel has dropped, drop the inner one and fold up
                if (pathTimer.getElapsedTime() > 2 * OUTTAKE_CLAW_DROP_TIME) {
                    //twoPersonDrive.setTransferState(TRANSFER_RESET);
                    Follower.useHeading = true;
                    setPathState(40);
                }
                break;


            case 40: // move the intake in
                setPathState(41);
                break;
            case 41: // once the robot is nice and folded up, request stop
                setPathState(-1);
                break;

            case 50:
                setPathState(51);
                break;
            case 51:
                follower.poseUpdater.resetOffset();
                PathChain abort = follower.pathBuilder()
                        .addPath(new BezierLine(new Point(follower.poseUpdater.getPose()), abortPoint))
                        .setConstantHeadingInterpolation(Math.PI * 1.5)
                        .build();
                follower.followPath(abort);
                setPathState(52);
                break;
            case 52:
                if (!follower.isBusy()) {
                    setPathState(-1);
                }
                break;

            default:
                requestOpModeStop();
                break;
        }

        if (opmodeTimer.getElapsedTimeSeconds() > 28) {
            foldUp.run();
        }
    }

    public void setPathState(int state) {
        pathState = state;
        pathTimer.resetTimer();
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
        //telemetry.update();
    }

    @Override
    public void init() {
        //PhotonCore.start(this.hardwareMap);

        foldUp = new SingleRunAction(()-> {
            if (Integer.parseInt(String.valueOf(pathState).substring(0,1)) < 4) setPathState(40);
        });

        distanceSensorDisconnects = new ArrayList<>();

        pathTimer = new Timer();
        opmodeTimer = new Timer();
        scanTimer = new Timer();
        follower = new Follower(hardwareMap);
        follower.setStartingPose(startPose);
        scanTimer.resetTimer();
    }

    @Override
    public void init_loop() {
        if (scanTimer.getElapsedTime() > 750) {
            navigation = "right";
            telemetry.addData("Navigation:", navigation);
            telemetry.update();
            scanTimer.resetTimer();
        }
    }

    @Override
    public void start() {
        setBackdropGoalPose();
        buildPaths();
        opmodeTimer.resetTimer();
        setPathState(10);
    }

    @Override
    public void stop() {
    }
}

/**
 * 8==D
 */