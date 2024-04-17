package org.firstinspires.ftc.team4100worlds.pedropathing.localization;

import com.acmerobotics.roadrunner.geometry.Pose2d;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.IMU;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.team4100worlds.ScrappyConstants;
import org.firstinspires.ftc.team4100worlds.pedropathing.pathgeneration.MathFunctions;
import org.firstinspires.ftc.team4100worlds.pedropathing.pathgeneration.Vector;

import java.util.ArrayList;
import java.util.List;

public class PoseUpdater {
    private final HardwareMap hardwareMap;

    private final IMU imu;

    private final TwoWheelTrackingLocalizer localizer;
    private final ElapsedTime imuTimer = new ElapsedTime();
    private Pose2d startingPose = new Pose2d(0, 0, 0);
    private Pose2d previousPose = startingPose;
    private double xOffset = 0, yOffset = 0, headingOffset = 0;
    private long previousPoseTime, currentPoseTime;

    /**
     * Creates a new PoseUpdater from a hardware map
     *
     * @param hardwareMap the hardware map
     */
    public PoseUpdater(HardwareMap hardwareMap) {
        this.hardwareMap = hardwareMap;
        this.imu = hardwareMap.get(IMU.class, "imu");
        this.imu.initialize(new IMU.Parameters(ScrappyConstants.CONTROL_HUB_ORIENTATION));
        this.imu.resetYaw();

        List<Integer> lastTrackingEncPositions = new ArrayList<>();
        List<Integer> lastTrackingEncVels = new ArrayList<>();

        localizer = new TwoWheelTrackingLocalizer(hardwareMap, imu);
//        localizer = new StandardTrackingWheelLocalizer(hardwareMap, lastTrackingEncPositions, lastTrackingEncVels);
        imuTimer.reset();
    }

    /**
     * Updates the robot's pose
     */
    public void update() {
//        if (imuTimer.seconds() >= 1) {
//            resetHeadingToIMU();
//            imuTimer.reset();
//        }
        previousPose = applyOffset(localizer.getPoseEstimate());
        previousPoseTime = currentPoseTime;
        currentPoseTime = System.nanoTime();
        localizer.update();
    }

    /**
     * This sets the starting pose. Do not run this after moving at all.
     *
     * @param set the pose to set the starting pose to
     */
    public void setStartingPose(Pose2d set) {
        startingPose = set;
        previousPose = startingPose;
        previousPoseTime = System.nanoTime();
        currentPoseTime = System.nanoTime();
        localizer.setPoseEstimate(set);
    }

    /**
     * This resets the current pose to a specified pose, using offsets to avoid having to reset using roadrunners pose reset
     *
     * @param set this is the pose we want to set the current pose to, using offsets
     */
    public void setCurrentPoseUsingOffset(Pose2d set) {
        Pose2d currentPose = localizer.getPoseEstimate();
        setXOffset(set.getX() - currentPose.getX());
        setYOffset(set.getY() - currentPose.getY());
        setHeadingOffset(MathFunctions.getTurnDirection(currentPose.getHeading(), set.getHeading()) * MathFunctions.getSmallestAngleDifference(currentPose.getHeading(), set.getHeading()));
    }

    public Pose2d getOffset() {
        return new Pose2d(xOffset, yOffset, headingOffset);
    }

    public double getXOffset() {
        return xOffset;
    }

    public void setXOffset(double offset) {
        xOffset = offset;
    }

    public void setRelXOffset(double relOffset) {
        xOffset = xOffset + relOffset;
    }

    public double getYOffset() {
        return yOffset;
    }

    public void setYOffset(double offset) {
        yOffset = offset;
    }

    public double getHeadingOffset() {
        return headingOffset;
    }

    public void setHeadingOffset(double offset) {
        headingOffset = offset;
    }

    public Pose2d applyOffset(Pose2d pose) {
        return new Pose2d(pose.getX() + xOffset, pose.getY() + yOffset, pose.getHeading() + headingOffset);
    }

    public void resetOffset() {
        setXOffset(0);
        setYOffset(0);
        setHeadingOffset(0);
    }

    /**
     * This returns the current pose
     *
     * @return returns the current pose
     */
    public Pose2d getPose() {
        return applyOffset(localizer.getPoseEstimate());
    }

    /**
     * This sets the current pose
     *
     * @param set the pose to set the current pose to
     */
    public void setPose(Pose2d set) {
        resetOffset();
        localizer.setPoseEstimate(set);
    }

    /**
     * Returns the robot's pose from the previous update
     *
     * @return returns the robot's previous pose
     */
    public Pose2d getPreviousPose() {
        return previousPose;
    }

    /**
     * Returns the robot's change in pose from the previous update
     *
     * @return returns the robot's delta pose
     */
    public Pose2d getDeltaPose() {
        return getPose().minus(previousPose);
    }

    /**
     * This returns the velocity of the robot as a vector
     *
     * @return returns the velocity of the robot
     */
    public Vector getVelocity() {
        Vector velocity = new Vector();
        velocity.setOrthogonalComponents(getPose().getX() - previousPose.getX(), getPose().getY() - previousPose.getY());
        velocity.setMagnitude(MathFunctions.distance(getPose(), previousPose) / ((currentPoseTime - previousPoseTime) / Math.pow(10.0, 9)));
        return velocity;
    }

    /**
     * This returns the velocity of the robot as a vector
     *
     * @return returns the velocity of the robot
     */
    public double getAngularVelocity() {
        return MathFunctions.getTurnDirection(previousPose.getHeading(), getPose().getHeading()) * MathFunctions.getSmallestAngleDifference(getPose().getHeading(), previousPose.getHeading()) / ((currentPoseTime - previousPoseTime) / Math.pow(10.0, 9));
    }

    /**
     * This resets the heading of the robot to the IMU's heading
     */
//    public void resetHeadingToIMU() {
//        localizer.resetHeading(getNormalizedIMUHeading() + startingPose.getHeading());
//    }

    /**
     * This returns the IMU heading normalized to be between [0, 2 PI] radians
     *
     * @return returns the normalized IMU heading
     */
    public double getNormalizedIMUHeading() {
        return imu.getRobotYawPitchRollAngles().getYaw(AngleUnit.RADIANS);
    }
}