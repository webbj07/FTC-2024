package org.firstinspires.ftc.teamcode.autonomous.blue;

import com.acmerobotics.roadrunner.Action;
import com.acmerobotics.roadrunner.Pose2d;
import com.acmerobotics.roadrunner.Vector2d;
import com.arcrobotics.ftclib.command.InstantCommand;
import com.arcrobotics.ftclib.command.SequentialCommandGroup;

import org.firstinspires.ftc.teamcode.ScrappySettings;
import org.firstinspires.ftc.teamcode.autonomous.ScrappyAutoBase;
import org.firstinspires.ftc.teamcode.commands.DriveToAprilTag;
import org.firstinspires.ftc.teamcode.commands.RunAction;

public class BlueFar2PlusOne extends ScrappyAutoBase {
    public static Pose2d startingPose = new Pose2d(-39, 61.75, Math.toRadians(90.00));
    private Action stackTraj, backboardTraj;
    public BlueFar2PlusOne() {
        super(ScrappySettings.AllianceType.BLUE, ScrappySettings.AllianceSide.FAR, startingPose);
    }

    @Override
    public void initAuto() {
        stackTraj = robot.m_drive.actionBuilder(startingPose)
                .strafeTo(new Vector2d(-35.23, 33.51))
                .strafeToLinearHeading(new Vector2d(-54, 36.4), Math.toRadians(180.00))
                .build();

        backboardTraj = robot.m_drive.actionBuilder(new Pose2d(-55, 35.5, Math.toRadians(180.00)))
                .strafeToLinearHeading(new Vector2d(-48, 45), Math.toRadians(180))
                .splineToConstantHeading(new Vector2d(12.5, 59.2), 0)
                .splineToConstantHeading(new Vector2d(40, 33), 0)
                .build();
    }

    @Override
    public void startAuto() {
        schedule(
                new SequentialCommandGroup(
                        new InstantCommand(robot.m_intake::setIntakeExUp),
                        new RunAction(stackTraj),
                        // intake ex down
                        new DriveToAprilTag(this, 9, false, 5),
                        // grab
                        new RunAction(backboardTraj),
                        new DriveToAprilTag(this, 2, true, 4)
                        // drop
                )
        );
    }
}
