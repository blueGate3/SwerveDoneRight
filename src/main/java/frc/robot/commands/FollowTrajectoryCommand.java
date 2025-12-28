package frc.robot.commands;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.trajectory.Trajectory;
import edu.wpi.first.networktables.NetworkTableInstance;
import edu.wpi.first.networktables.StructSubscriber;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.Drivetrain;
import frc.robot.subsystems.TrajectoryFollower;

public class FollowTrajectoryCommand extends Command {
    private Drivetrain m_drivetrain = new Drivetrain();
    private Trajectory m_trajectory;
    TrajectoryFollower m_trajectoryFollower;
    private final Timer m_timer = new Timer();
    private final StructSubscriber<Pose2d> poseSub = NetworkTableInstance.getDefault()
      .getStructTopic("Robot/CurrentPose", Pose2d.struct).subscribe(new Pose2d());

    public FollowTrajectoryCommand(Drivetrain drivetrain) {
        m_drivetrain = drivetrain;
        addRequirements(m_drivetrain);
    }
    @Override
    public void initialize() {
        TrajectoryFollower m_trajectoryFollower = new TrajectoryFollower();
        Trajectory m_trajectory = m_trajectoryFollower.generateTrajectory(poseSub.get(), null, new Pose2d());
        m_timer.restart(); //starts the timer at 0 for timestamps below.
    }
    @SuppressWarnings("static-access")
    @Override
    public void execute() {
        m_drivetrain.driveWithChassisSpeeds(
            m_trajectoryFollower.trajectoryToChassisSpeeds(m_trajectory, m_timer.getTimestamp())
        );
    }
    
    @Override
    public void end(boolean interrupted) {
        m_timer.reset();
    }
}
