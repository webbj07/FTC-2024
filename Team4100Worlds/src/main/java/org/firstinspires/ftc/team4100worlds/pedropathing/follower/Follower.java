package org.firstinspires.ftc.team4100worlds.pedropathing.follower;

import static org.firstinspires.ftc.team4100worlds.FollowerConstants.drivePIDFSwitch;
import static org.firstinspires.ftc.team4100worlds.FollowerConstants.forwardZeroPowerAcceleration;
import static org.firstinspires.ftc.team4100worlds.FollowerConstants.headingPIDFSwitch;
import static org.firstinspires.ftc.team4100worlds.FollowerConstants.largeDrivePIDFFeedForward;
import static org.firstinspires.ftc.team4100worlds.FollowerConstants.largeHeadingPIDFFeedForward;
import static org.firstinspires.ftc.team4100worlds.FollowerConstants.largeTranslationalPIDFFeedForward;
import static org.firstinspires.ftc.team4100worlds.FollowerConstants.lateralZeroPowerAcceleration;
import static org.firstinspires.ftc.team4100worlds.FollowerConstants.smallDrivePIDFFeedForward;
import static org.firstinspires.ftc.team4100worlds.FollowerConstants.smallHeadingPIDFFeedForward;
import static org.firstinspires.ftc.team4100worlds.FollowerConstants.smallTranslationalPIDFFeedForward;
import static org.firstinspires.ftc.team4100worlds.FollowerConstants.translationalPIDFSwitch;
import static org.firstinspires.ftc.team4100worlds.FollowerConstants.zeroPowerAccelerationMultiplier;

import com.acmerobotics.dashboard.FtcDashboard;
import com.acmerobotics.dashboard.canvas.Canvas;
import com.acmerobotics.dashboard.config.Config;
import com.acmerobotics.dashboard.telemetry.TelemetryPacket;
import com.acmerobotics.roadrunner.geometry.Pose2d;
import com.acmerobotics.roadrunner.geometry.Vector2d;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.configuration.typecontainers.MotorConfigurationType;

import org.firstinspires.ftc.team4100worlds.FollowerConstants;
import org.firstinspires.ftc.team4100worlds.pedropathing.localization.PoseUpdater;
import org.firstinspires.ftc.team4100worlds.pedropathing.pathgeneration.BezierPoint;
import org.firstinspires.ftc.team4100worlds.pedropathing.pathgeneration.MathFunctions;
import org.firstinspires.ftc.team4100worlds.pedropathing.pathgeneration.Path;
import org.firstinspires.ftc.team4100worlds.pedropathing.pathgeneration.PathBuilder;
import org.firstinspires.ftc.team4100worlds.pedropathing.pathgeneration.PathCallback;
import org.firstinspires.ftc.team4100worlds.pedropathing.pathgeneration.PathChain;
import org.firstinspires.ftc.team4100worlds.pedropathing.pathgeneration.Point;
import org.firstinspires.ftc.team4100worlds.pedropathing.pathgeneration.Vector;
import org.firstinspires.ftc.team4100worlds.util.PIDFController;

import java.util.ArrayList;
import java.util.Arrays;
import java.util.List;

@Config
public class Follower {
    public static int POSE_HISTORY_LIMIT = 100;
    public static boolean useTranslational = true, useCentripetal = true, useHeading = true, useDrive = true;
    public static double holdPointTranslationalScaling = 0.8, holdPointHeadingScaling = 1;
    private final int BEZIER_CURVE_BINARY_STEP_LIMIT = 10, DELTA_POSE_RECORD_LIMIT = 8;
    private final HardwareMap hardwareMap;
    private final ArrayList<Pose2d> poseHistory = new ArrayList<>();
    private final ArrayList<Pose2d> deltaPoses = new ArrayList<>();
    private final ArrayList<Pose2d> deltaDeltaPoses = new ArrayList<>();
    private final PIDFController smallTranslationalPIDF = new PIDFController(FollowerConstants.smallTranslationalPIDFCoefficients);
    private final PIDFController smallTranslationalIntegral = new PIDFController(FollowerConstants.smallTranslationalIntegral);
    private final PIDFController largeTranslationalPIDF = new PIDFController(FollowerConstants.largeTranslationalPIDFCoefficients);
    private final PIDFController largeTranslationalIntegral = new PIDFController(FollowerConstants.largeTranslationalIntegral);
    private final PIDFController smallHeadingPIDF = new PIDFController(FollowerConstants.smallHeadingPIDFCoefficients);
    private final PIDFController largeHeadingPIDF = new PIDFController(FollowerConstants.largeHeadingPIDFCoefficients);
    private final PIDFController smallDrivePIDF = new PIDFController(FollowerConstants.smallDrivePIDFCoefficients);
    private final PIDFController largeDrivePIDF = new PIDFController(FollowerConstants.largeDrivePIDFCoefficients);
    public DcMotorEx leftFront, leftRear, rightFront, rightRear;
    public PoseUpdater poseUpdater;
    private List<DcMotorEx> motors;
    private DriveVectorScaler driveVectorScaler;
    private Pose2d closestPose;
    private Path currentPath;
    private PathChain currentPathChain;
    private int chainIndex;
    private long[] pathStartTimes;
    private boolean followingPathChain, holdingPosition, isBusy, auto = true, reachedParametricPathEnd;
    private double maxPower = 1, previousSmallTranslationalIntegral, previousLargeTranslationalIntegral;
    private long reachedParametricPathEndTime;
    private double[] drivePowers;
    private int collisionCounts = 0;
    private boolean isCollision = false;
    private boolean isFirstCollision = false;
    private Vector[] teleOpMovementVectors = new Vector[]{new Vector(0, 0), new Vector(0, 0), new Vector(0, 0)};
    private Pose2d averageDeltaPose, averagePreviousDeltaPose, averageDeltaDeltaPose;
    private Vector smallTranslationalIntegralVector, largeTranslationalIntegralVector;

    /**
     * This creates a new follower given a hardware map
     *
     * @param hardwareMap hardware map required
     */
    public Follower(HardwareMap hardwareMap) {
        this.hardwareMap = hardwareMap;
        initialize();
    }

    /**
     * This creates a new follower given a hardware map and sets whether the follower is being used
     * in autonomous or teleop
     *
     * @param hardwareMap hardware map required
     * @param setAuto     sets whether or not the follower is being used in autonomous or teleop
     */
    public Follower(HardwareMap hardwareMap, boolean setAuto) {
        this.hardwareMap = hardwareMap;
        setAuto(setAuto);
        initialize();
    }

    public static void drawPoseHistory(Canvas canvas, List<Pose2d> poseHistory) {
        double[] xPoints = new double[poseHistory.size()];
        double[] yPoints = new double[poseHistory.size()];
        for (int i = 0; i < poseHistory.size(); i++) {
            Pose2d pose = poseHistory.get(i);
            xPoints[i] = pose.getX();
            yPoints[i] = pose.getY();
        }
        canvas.strokePolyline(xPoints, yPoints);
    }

    public static void drawPath(Canvas canvas, Path path, double resolution) {
        int samples = (int) Math.ceil(path.length() / resolution);
        double[] xPoints = new double[samples];
        double[] yPoints = new double[samples];
        for (int i = 0; i < samples; i++) {
            double t = i / (double) (samples - 1);
            Point pose = path.getPoint(t);
            xPoints[i] = pose.getX();
            yPoints[i] = pose.getY();
        }
        canvas.strokePolyline(xPoints, yPoints);
    }

    public static void drawRobot(Canvas canvas, Pose2d pose, double robotRadius) {
        canvas.strokeCircle(pose.getX(), pose.getY(), robotRadius);
        Vector2d v = pose.headingVec().times(robotRadius);
        double x1 = pose.getX() + v.getX() / 2, y1 = pose.getY() + v.getY() / 2;
        double x2 = pose.getX() + v.getX(), y2 = pose.getY() + v.getY();
        canvas.strokeLine(x1, y1, x2, y2);
    }

    /**
     * This initializes the follower
     */
    public void initialize() {
        driveVectorScaler = new DriveVectorScaler(FollowerConstants.frontLeftVector);
        poseUpdater = new PoseUpdater(hardwareMap);

        leftFront = hardwareMap.get(DcMotorEx.class, "LF");
        leftRear = hardwareMap.get(DcMotorEx.class, "LB");
        rightRear = hardwareMap.get(DcMotorEx.class, "RB");
        rightFront = hardwareMap.get(DcMotorEx.class, "RF");

        rightRear.setDirection(DcMotorSimple.Direction.REVERSE);
        motors = Arrays.asList(leftFront, leftRear, rightFront, rightRear);

        for (DcMotorEx motor : motors) {
            MotorConfigurationType motorConfigurationType = motor.getMotorType().clone();
            motorConfigurationType.setAchieveableMaxRPMFraction(1.0);
            motor.setMotorType(motorConfigurationType);
        }

        for (DcMotorEx motor : motors) {
            motor.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        }

        for (int i = 0; i < DELTA_POSE_RECORD_LIMIT; i++) {
            deltaPoses.add(new Pose2d(0, 0, 0));
        }
        for (int i = 0; i < DELTA_POSE_RECORD_LIMIT / 2; i++) {
            deltaDeltaPoses.add(new Pose2d(0, 0, 0));
        }
        calculateAverageDeltaPoses();
    }

    /**
     * Sets the maximum power the motors are allowed to use
     */
    public void setMaxPower(double set) {
        maxPower = MathFunctions.clamp(set, 0, 1);
    }

    /**
     * Limits the powers of the drive power array to the max power
     */
    public void limitDrivePowers() {
        for (int i = 0; i < drivePowers.length; i++) {
            if (Math.abs(drivePowers[i]) > maxPower) {
                drivePowers[i] = maxPower * MathFunctions.getSign(drivePowers[i]);
            }
        }
    }

    /**
     * This returns the current pose
     *
     * @return returns the pose
     */
    public Pose2d getPose() {
        return poseUpdater.getPose();
    }

    public void setPose(Pose2d pose) {
        poseUpdater.setPose(pose);
    }

    /**
     * This returns the current velocity
     *
     * @return returns the current velocity
     */
    public Vector getVelocity() {
        return poseUpdater.getVelocity();
    }

    /**
     * This returns the magnitude of the current velocity
     *
     * @return returns the magnitude of the current velocity
     */
    public double getVelocityMagnitude() {
        return poseUpdater.getVelocity().getMagnitude();
    }

    /**
     * This sets the starting pose. Do not run this after moving at all.
     *
     * @param pose the pose to set the starting pose to
     */
    public void setStartingPose(Pose2d pose) {
        poseUpdater.setStartingPose(pose);
    }

    /**
     * This holds a point
     *
     * @param point   the point to stay at
     * @param heading the direction to face
     */
    public void holdPoint(BezierPoint point, double heading) {
        breakFollowing();
        holdingPosition = true;
        isBusy = true;
        followingPathChain = false;
        currentPath = new Path(point);
        currentPath.setConstantHeadingInterpolation(heading);
        closestPose = currentPath.getClosestPoint(poseUpdater.getPose(), 1);
    }

    /**
     * This follows a path
     *
     * @param path the path to follow
     */
    public void followPath(Path path) {
        breakFollowing();
        isBusy = true;
        followingPathChain = false;
        currentPath = path;
        closestPose = currentPath.getClosestPoint(poseUpdater.getPose(), BEZIER_CURVE_BINARY_STEP_LIMIT);
    }

    /**
     * This follows a path chain and only slows down at the end of the path chain
     *
     * @param pathChain the path chain to follow
     */
    public void followPath(PathChain pathChain) {
        breakFollowing();
        pathStartTimes = new long[pathChain.size()];
        pathStartTimes[0] = System.currentTimeMillis();
        isBusy = true;
        followingPathChain = true;
        chainIndex = 0;
        currentPathChain = pathChain;
        currentPath = pathChain.getPath(chainIndex);
        closestPose = currentPath.getClosestPoint(poseUpdater.getPose(), BEZIER_CURVE_BINARY_STEP_LIMIT);
    }

    /**
     * Updates the robot's position and drive powers
     */
    public void update() {
        poseUpdater.update();
        if (auto) {
            if (holdingPosition) {
                closestPose = currentPath.getClosestPoint(poseUpdater.getPose(), 1);

                drivePowers = driveVectorScaler.getDrivePowers(MathFunctions.scalarMultiplyVector(getTranslationalCorrection(), holdPointTranslationalScaling), MathFunctions.scalarMultiplyVector(getHeadingVector(), holdPointHeadingScaling), new Vector(), poseUpdater.getPose().getHeading());

                limitDrivePowers();

                for (int i = 0; i < motors.size(); i++) {
                    motors.get(i).setPower(drivePowers[i]);
                }
            } else {
                if (isBusy) {
                    if (isCollision) {
//                        Point closestPoint = currentPath.getPoint(currentPath.getClosestPointTValue() - 0.5);
//                        closestPose = new Pose2d(closestPoint.getX(), closestPoint.getY(), currentPath.getClosestPointHeadingGoal());

                        if (isFirstCollision) {
//                            closestPose = currentPath.getPoint(currentPath.getClosestPointTValue() - 0.25);
                            isFirstCollision = false;
                        }

                        if (MathFunctions.distance(closestPose, poseUpdater.getPose()) < 3) {
                            isCollision = false;
                        }
//                    } else if (getVelocityMagnitude() < 1.5 && !currentPath.isAtParametricEnd()) {
//                        collisionCounts++;
//
//                        if (collisionCounts > 2) {
//                            isCollision = true;
//                            isFirstCollision = true;
//                        }
                    } else {
                        collisionCounts = 0;
                        closestPose = currentPath.getClosestPoint(poseUpdater.getPose(), BEZIER_CURVE_BINARY_STEP_LIMIT);
                    }

                    if (followingPathChain) updateCallbacks();

                    drivePowers = driveVectorScaler.getDrivePowers(getCorrectiveVector(), getHeadingVector(), getDriveVector(), poseUpdater.getPose().getHeading());

                    limitDrivePowers();

                    for (int i = 0; i < motors.size(); i++) {
                        motors.get(i).setPower(drivePowers[i]);
                    }
                }
                if (currentPath != null && currentPath.isAtParametricEnd()) {
                    if (followingPathChain && chainIndex < currentPathChain.size() - 1) {
                        // Not at last path, keep going
                        breakFollowing();
                        pathStartTimes[chainIndex] = System.currentTimeMillis();
                        isBusy = true;
                        followingPathChain = true;
                        chainIndex++;
                        currentPath = currentPathChain.getPath(chainIndex);
                        closestPose = currentPath.getClosestPoint(poseUpdater.getPose(), BEZIER_CURVE_BINARY_STEP_LIMIT);
                    } else {
                        // At last path, run some end detection stuff
                        // set isBusy to false if at end
                        if (!reachedParametricPathEnd) {
                            reachedParametricPathEnd = true;
                            reachedParametricPathEndTime = System.currentTimeMillis();
                        }

                        if ((System.currentTimeMillis() - reachedParametricPathEndTime > currentPath.getPathEndTimeout()) || (poseUpdater.getVelocity().getMagnitude() < currentPath.getPathEndVelocity() && MathFunctions.distance(poseUpdater.getPose(), closestPose) < currentPath.getPathEndTranslational() && MathFunctions.getSmallestAngleDifference(poseUpdater.getPose().getHeading(), currentPath.getClosestPointHeadingGoal()) < currentPath.getPathEndHeading())) {
                            breakFollowing();
                        }
                    }
                }
            }
        } else {
            deltaPoses.add(poseUpdater.getDeltaPose());
            deltaPoses.remove(deltaPoses.get(deltaPoses.size() - 1));

            calculateAverageDeltaPoses();

            drivePowers = driveVectorScaler.getDrivePowers(teleOpMovementVectors[0], teleOpMovementVectors[1], teleOpMovementVectors[2], poseUpdater.getPose().getHeading());

            limitDrivePowers();

            for (int i = 0; i < motors.size(); i++) {
                motors.get(i).setPower(drivePowers[i]);
            }
        }

        Pose2d pose = poseUpdater.getPose();

        poseHistory.add(pose);

        if (POSE_HISTORY_LIMIT > -1 && poseHistory.size() > POSE_HISTORY_LIMIT) {
            poseHistory.remove(0);
        }

        FtcDashboard dashboard = FtcDashboard.getInstance();

        double robotLength = 19.5;

        TelemetryPacket packet = new TelemetryPacket();
        Canvas fieldOverlay = packet.fieldOverlay();
        fieldOverlay.setStrokeWidth(1);

        // Draw robot (current pose)
        fieldOverlay.setStroke("#3F51B5");
        drawRobot(fieldOverlay, poseUpdater.getPose(), robotLength / 2);

        // Draw pose history
        drawPoseHistory(fieldOverlay, poseHistory);

        if (currentPath != null) {
            // Draw robot (target pose)
            fieldOverlay.setStroke("#4CAF50");
            Point lastControlPoint = currentPath.getLastControlPoint();
            drawRobot(fieldOverlay, new Pose2d(lastControlPoint.getX(), lastControlPoint.getY(), currentPath.getPathEndHeading()), robotLength / 2);

            // Draw path
            drawPath(fieldOverlay, currentPath, 2.0);
        }

        dashboard.sendTelemetryPacket(packet);
    }

    /**
     * Calculates the average delta poses
     */
    public void calculateAverageDeltaPoses() {
        averageDeltaPose = new Pose2d(0, 0, 0);
        averagePreviousDeltaPose = new Pose2d(0, 0, 0);

        for (int i = 0; i < deltaPoses.size() / 2; i++) {
            averageDeltaPose.plus(deltaPoses.get(i));
        }
        averageDeltaPose.div(deltaPoses.size() / 2);

        for (int i = deltaPoses.size() / 2; i < deltaPoses.size(); i++) {
            averagePreviousDeltaPose.plus(deltaPoses.get(i));
        }
        averagePreviousDeltaPose.div(deltaPoses.size() / 2);

        deltaDeltaPoses.add(averageDeltaPose.minus(averagePreviousDeltaPose));
        deltaDeltaPoses.remove(deltaDeltaPoses.size() - 1);

        averageDeltaDeltaPose = new Pose2d(0, 0, 0);

        for (int i = 0; i < deltaDeltaPoses.size(); i++) {
            averageDeltaDeltaPose.plus(deltaDeltaPoses.get(i));
        }
        averageDeltaDeltaPose.div(deltaDeltaPoses.size());
    }

    /**
     * This checks if any callbacks should be run right now, and runs them if applicable.
     */
    public void updateCallbacks() {
        for (PathCallback callback : currentPathChain.getCallbacks()) {
            if (!callback.hasBeenRun()) {
                if (callback.getType() == PathCallback.PARAMETRIC) {
                    // parametric call back
                    if (chainIndex == callback.getIndex() && (getCurrentTValue() >= callback.getStartCondition() || MathFunctions.roughlyEquals(getCurrentTValue(), callback.getStartCondition()))) {
                        callback.run();
                    }
                } else {
                    // time based call back
                    if (chainIndex >= callback.getIndex() && System.currentTimeMillis() - pathStartTimes[callback.getIndex()] > callback.getStartCondition()) {
                        callback.run();
                    }

                }
            }
        }
    }

    /**
     * This resets the PIDFs and stops following
     */
    public void breakFollowing() {
        holdingPosition = false;
        isBusy = false;
        reachedParametricPathEnd = false;
        smallDrivePIDF.reset();
        largeDrivePIDF.reset();
        smallHeadingPIDF.reset();
        largeHeadingPIDF.reset();
        smallTranslationalPIDF.reset();
        smallTranslationalIntegral.reset();
        smallTranslationalIntegralVector = new Vector();
        previousSmallTranslationalIntegral = 0;
        largeTranslationalPIDF.reset();
        largeTranslationalIntegral.reset();
        largeTranslationalIntegralVector = new Vector();
        previousLargeTranslationalIntegral = 0;

        for (int i = 0; i < motors.size(); i++) {
            motors.get(i).setPower(0);
        }
    }

    // TODO: REMOVE
    public double[] motorPowers() {
        return driveVectorScaler.getDrivePowers(getCorrectiveVector(), getHeadingVector(), getDriveVector(), poseUpdater.getPose().getHeading());
    }

    /**
     * This returns if the follower is currently following a path or a path chain
     *
     * @return returns if the follower is busy
     */
    public boolean isBusy() {
        return isBusy;
    }

    /**
     * Sets the correctional, heading, and drive movement vectors for teleop enhancements
     */
    public void setMovementVectors(Vector correctional, Vector heading, Vector drive) {
        teleOpMovementVectors = new Vector[]{correctional, heading, drive};
    }

    public void setMovementPowers(double lfPower, double lrPower, double rfPower, double rrPower) {
        leftFront.setPower(lfPower);
        leftRear.setPower(lrPower);
        rightFront.setPower(rfPower);
        rightRear.setPower(rrPower);
    }

    /**
     * This returns a Vector in the direction the robot must go to move along the path
     * <p>
     * Note: This vector is clamped to be at most 1 in magnitude
     *
     * @return returns the drive vector
     */
    public Vector getDriveVector() {
        if (!useDrive) return new Vector();
        if (followingPathChain && chainIndex < currentPathChain.size() - 1) {
            return new Vector(1, currentPath.getClosestPointTangentVector().getTheta());
        }

        double driveError = getDriveVelocityError();

        if (Math.abs(driveError) < drivePIDFSwitch) {
            smallDrivePIDF.updateError(driveError);
            return new Vector(MathFunctions.clamp(smallDrivePIDF.runPIDF() + smallDrivePIDFFeedForward * MathFunctions.getSign(driveError), -1, 1), currentPath.getClosestPointTangentVector().getTheta());
        }

        largeDrivePIDF.updateError(driveError);
        return new Vector(MathFunctions.clamp(largeDrivePIDF.runPIDF() + largeDrivePIDFFeedForward * MathFunctions.getSign(driveError), -1, 1), currentPath.getClosestPointTangentVector().getTheta());
    }

    // TODO: remove
    public double zxcv() {
        return getDriveVelocityError();
    }

    /**
     * This returns the velocity the robot needs to be at to make it to the end of the trajectory
     *
     * @return returns the projected velocity
     */
    public double getDriveVelocityError() {
        double distanceToGoal;
        if (!currentPath.isAtParametricEnd()) {
            distanceToGoal = currentPath.length() * (1 - currentPath.getClosestPointTValue());
        } else {
            Vector offset = new Vector();
            offset.setOrthogonalComponents(getPose().getX() - currentPath.getLastControlPoint().getX(), getPose().getY() - currentPath.getLastControlPoint().getY());
            distanceToGoal = MathFunctions.dotProduct(currentPath.getEndTangent(), offset);
        }

        Vector distanceToGoalVector = MathFunctions.scalarMultiplyVector(MathFunctions.normalizeVector(currentPath.getClosestPointTangentVector()), distanceToGoal);
        Vector velocity = new Vector(MathFunctions.dotProduct(getVelocity(), MathFunctions.normalizeVector(currentPath.getClosestPointTangentVector())), currentPath.getClosestPointTangentVector().getTheta());
        Vector forwardHeadingVector = new Vector(1.0, poseUpdater.getPose().getHeading());
        double forwardVelocity = MathFunctions.dotProduct(forwardHeadingVector, velocity);
        double forwardDistanceToGoal = MathFunctions.dotProduct(forwardHeadingVector, distanceToGoalVector);
        Vector lateralHeadingVector = new Vector(1.0, poseUpdater.getPose().getHeading() - Math.PI / 2);
        double lateralVelocity = MathFunctions.dotProduct(lateralHeadingVector, velocity);
        double lateralDistanceToGoal = MathFunctions.dotProduct(lateralHeadingVector, distanceToGoalVector);

        Vector forwardVelocityError = new Vector(MathFunctions.getSign(forwardDistanceToGoal) * Math.sqrt(Math.abs(-2 * zeroPowerAccelerationMultiplier * forwardZeroPowerAcceleration * forwardDistanceToGoal)) - forwardVelocity, forwardHeadingVector.getTheta());
        Vector lateralVelocityError = new Vector(MathFunctions.getSign(lateralDistanceToGoal) * Math.sqrt(Math.abs(-2 * zeroPowerAccelerationMultiplier * lateralZeroPowerAcceleration * lateralDistanceToGoal)) - lateralVelocity, lateralHeadingVector.getTheta());
        Vector velocityErrorVector = MathFunctions.addVectors(forwardVelocityError, lateralVelocityError);

        return velocityErrorVector.getMagnitude() * MathFunctions.getSign(MathFunctions.dotProduct(velocityErrorVector, currentPath.getClosestPointTangentVector()));
    }

    /**
     * This returns a Vector in the direction of the robot that contains the heading correction
     * as its magnitude
     * <p>
     * Note: This vector is clamped to be at most 1 in magnitude
     *
     * @return returns the heading vector
     */
    public Vector getHeadingVector() {
        if (!useHeading) return new Vector();
        double headingError = MathFunctions.getTurnDirection(poseUpdater.getPose().getHeading(), currentPath.getClosestPointHeadingGoal()) * MathFunctions.getSmallestAngleDifference(poseUpdater.getPose().getHeading(), currentPath.getClosestPointHeadingGoal());
        if (Math.abs(headingError) < headingPIDFSwitch) {
            smallHeadingPIDF.updateError(headingError);
            return new Vector(MathFunctions.clamp(smallHeadingPIDF.runPIDF() + smallHeadingPIDFFeedForward * MathFunctions.getTurnDirection(poseUpdater.getPose().getHeading(), currentPath.getClosestPointHeadingGoal()), -1, 1), poseUpdater.getPose().getHeading());
        }
        largeHeadingPIDF.updateError(headingError);
        return new Vector(MathFunctions.clamp(largeHeadingPIDF.runPIDF() + largeHeadingPIDFFeedForward * MathFunctions.getTurnDirection(poseUpdater.getPose().getHeading(), currentPath.getClosestPointHeadingGoal()), -1, 1), poseUpdater.getPose().getHeading());
    }

    public Vector getHeadingVectorAprilTag(double angle) {
        double headingError = MathFunctions.getTurnDirection(poseUpdater.getPose().getHeading(), angle) * MathFunctions.getSmallestAngleDifference(poseUpdater.getPose().getHeading(), angle);
        smallHeadingPIDF.updateError(headingError);
        return new Vector(MathFunctions.clamp(smallHeadingPIDF.runPIDF() + smallHeadingPIDFFeedForward * MathFunctions.getTurnDirection(poseUpdater.getPose().getHeading(), headingError + poseUpdater.getPose().getHeading()), -1, 1), poseUpdater.getPose().getHeading());
    }

    /**
     * This returns a Vector in the direction the robot must go to account for both translational
     * error as well as centripetal force.
     * <p>
     * Note: This vector is clamped to be at most 1 in magnitude
     *
     * @return returns the corrective vector
     */
    public Vector getCorrectiveVector() {
        Vector centripetal = getCentripetalForceCorrection();
        Vector translational = getTranslationalCorrection();
        Vector corrective = MathFunctions.addVectors(centripetal, translational);

        if (corrective.getMagnitude() > 1) {
            return MathFunctions.addVectors(centripetal, MathFunctions.scalarMultiplyVector(translational, driveVectorScaler.findNormalizingScaling(centripetal, translational)));
        }

        return corrective;
    }

    /**
     * This returns a Vector in the direction the robot must go to account for only translational
     * error
     * <p>
     * Note: This vector is clamped to be at most 1 in magnitude
     *
     * @return returns the translational vector
     */
    public Vector getTranslationalCorrection() {
        if (!useTranslational) return new Vector();
        Vector translationalVector = new Vector();
        double x = closestPose.getX() - poseUpdater.getPose().getX();
        double y = closestPose.getY() - poseUpdater.getPose().getY();
        translationalVector.setOrthogonalComponents(x, y);

        if (!(currentPath.isAtParametricEnd() || currentPath.isAtParametricStart())) {
            translationalVector = MathFunctions.subtractVectors(translationalVector, new Vector(MathFunctions.dotProduct(translationalVector, MathFunctions.normalizeVector(currentPath.getClosestPointTangentVector())), currentPath.getClosestPointTangentVector().getTheta()));

            smallTranslationalIntegralVector = MathFunctions.subtractVectors(smallTranslationalIntegralVector, new Vector(MathFunctions.dotProduct(smallTranslationalIntegralVector, MathFunctions.normalizeVector(currentPath.getClosestPointTangentVector())), currentPath.getClosestPointTangentVector().getTheta()));
            largeTranslationalIntegralVector = MathFunctions.subtractVectors(largeTranslationalIntegralVector, new Vector(MathFunctions.dotProduct(largeTranslationalIntegralVector, MathFunctions.normalizeVector(currentPath.getClosestPointTangentVector())), currentPath.getClosestPointTangentVector().getTheta()));
        }

        if (MathFunctions.distance(poseUpdater.getPose(), closestPose) < translationalPIDFSwitch) {
            smallTranslationalIntegral.updateError(translationalVector.getMagnitude());
            smallTranslationalIntegralVector = MathFunctions.addVectors(smallTranslationalIntegralVector, new Vector(smallTranslationalIntegral.runPIDF() - previousSmallTranslationalIntegral, translationalVector.getTheta()));
            previousSmallTranslationalIntegral = smallTranslationalIntegral.runPIDF();

            smallTranslationalPIDF.updateError(translationalVector.getMagnitude());
            translationalVector.setMagnitude(smallTranslationalPIDF.runPIDF() + smallTranslationalPIDFFeedForward);
            translationalVector = MathFunctions.addVectors(translationalVector, smallTranslationalIntegralVector);
        } else {
            largeTranslationalIntegral.updateError(translationalVector.getMagnitude());
            largeTranslationalIntegralVector = MathFunctions.addVectors(largeTranslationalIntegralVector, new Vector(largeTranslationalIntegral.runPIDF() - previousLargeTranslationalIntegral, translationalVector.getTheta()));
            previousLargeTranslationalIntegral = largeTranslationalIntegral.runPIDF();

            largeTranslationalPIDF.updateError(translationalVector.getMagnitude());
            translationalVector.setMagnitude(largeTranslationalPIDF.runPIDF() + largeTranslationalPIDFFeedForward);
            translationalVector = MathFunctions.addVectors(translationalVector, largeTranslationalIntegralVector);
        }

        translationalVector.setMagnitude(MathFunctions.clamp(translationalVector.getMagnitude(), 0, 1));

        return translationalVector;
    }

    public Vector getTranslationalError() {
        Vector error = new Vector();
        double x = closestPose.getX() - poseUpdater.getPose().getX();
        double y = closestPose.getY() - poseUpdater.getPose().getY();
        error.setOrthogonalComponents(x, y);
        return error;
    }

    // TODO: remove later
    public double asdf() {
        return getTranslationalCorrection().getTheta();
    }

    public double qwerty() {
        return driveVectorScaler.getLeftSidePath().getTheta();
    }

    public double qwerty2() {
        return driveVectorScaler.getRightSidePath().getTheta();
    }

    /**
     * This returns a Vector in the direction the robot must go to account for only centripetal
     * force
     * <p>
     * Note: This vector is clamped to be between [0, 1] in magnitude
     *
     * @return returns the centripetal vector
     */
    public Vector getCentripetalForceCorrection() {
        if (!useCentripetal) return new Vector();
        double curvature;
        if (auto) {
            curvature = currentPath.getClosestPointCurvature();
        } else {
            double yPrime = averageDeltaPose.getY() / averageDeltaPose.getX();
            double yDoublePrime = averageDeltaDeltaPose.getY() / averageDeltaPose.getX();
            curvature = (Math.pow(Math.sqrt(1 + Math.pow(yPrime, 2)), 3)) / (yDoublePrime);
        }
        if (Double.isNaN(curvature)) return new Vector();
        return new Vector(MathFunctions.clamp(FollowerConstants.centrifugalScaling * FollowerConstants.mass * Math.pow(MathFunctions.dotProduct(poseUpdater.getVelocity(), MathFunctions.normalizeVector(currentPath.getClosestPointTangentVector())), 2) * curvature, -1, 1), currentPath.getClosestPointTangentVector().getTheta() + Math.PI / 2 * MathFunctions.getSign(currentPath.getClosestPointNormalVector().getTheta()));
    }

    /**
     * This returns the closest pose to the robot on the path the follower is currently following
     *
     * @return returns the closest pose
     */
    public Pose2d getClosestPose() {
        return closestPose;
    }

    /**
     * This sets whether or not the follower is being used in auto or teleop
     *
     * @param set sets auto or not
     */
    public void setAuto(boolean set) {
        auto = set;
    }

    /**
     * This returns whether the follower is at the parametric end of its current path
     * If running a path chain, this returns true only if at parametric end of last path in the chain
     *
     * @return returns whether the follower is at the parametric end of its path
     */
    public boolean atParametricEnd() {
        if (followingPathChain) {
            if (chainIndex == currentPathChain.size() - 1) return currentPath.isAtParametricEnd();
            return false;
        }
        return currentPath.isAtParametricEnd();
    }

    /**
     * This returns the t value of the closest point on the current path to the robot
     * In the absence of a current path, it returns 1.0
     *
     * @return returns the current t value
     */
    public double getCurrentTValue() {
        if (isBusy) return currentPath.getClosestPointTValue();
        return 1.0;
    }

    /**
     * This returns the current path number. For just paths, this will return 0. For path chains,
     * this will return the current path number.
     *
     * @return returns the current path number
     */
    public double getCurrentPathNumber() {
        if (!followingPathChain) return 0;
        return chainIndex;
    }

    public PathBuilder pathBuilder() {
        return new PathBuilder();
    }
}