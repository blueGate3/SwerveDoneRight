package frc.robot.subsystems;

import java.util.List;

import edu.wpi.first.math.controller.HolonomicDriveController;
import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.controller.ProfiledPIDController;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.math.trajectory.Trajectory;
import edu.wpi.first.math.trajectory.TrajectoryConfig;
import edu.wpi.first.math.trajectory.TrajectoryGenerator;
import edu.wpi.first.math.trajectory.TrapezoidProfile.Constraints;
import edu.wpi.first.math.trajectory.constraint.TrajectoryConstraint;
import edu.wpi.first.networktables.NetworkTableInstance;
import edu.wpi.first.networktables.StructSubscriber;
import frc.robot.NetworkTables;
import frc.robot.Constants.TrajectoryConst;

public class TrajectoryFollower {
    TrajectoryConfig m_trajConfig;
    Trajectory m_trajectory = new Trajectory();
    HolonomicDriveController m_trajectoryController; 
    ChassisSpeeds m_chassisSpeeds;

    private final StructSubscriber<Pose2d> poseSub = NetworkTableInstance.getDefault()
      .getStructTopic("Robot/CurrentPose", Pose2d.struct).subscribe(new Pose2d());

    public TrajectoryFollower() {
        m_trajConfig = new TrajectoryConfig(TrajectoryConst.kMaxSpeed, TrajectoryConst.kMaxAcceleration);
        m_trajectoryController = new HolonomicDriveController(
            new PIDController(0, 0, 0), 
            new PIDController(0, 0, 0), 
            new ProfiledPIDController(0, 0, 0, 
                new Constraints(0, 0))
        );
    }

    public Trajectory generateTrajectory(Pose2d startPose, List<Translation2d> waypoints, Pose2d endPose) {
        m_trajectory = TrajectoryGenerator.generateTrajectory(
            startPose, 
            waypoints, 
            endPose, 
            m_trajConfig);
        return m_trajectory;
    }

    public ChassisSpeeds trajectoryToChassisSpeeds(Trajectory m_trajectory, double timestamp) {
        Pose2d currentPose = poseSub.get();
        Trajectory.State goal = m_trajectory.sample(timestamp);
        m_chassisSpeeds = m_trajectoryController.calculate(currentPose, goal, goal.poseMeters.getRotation());
        return m_chassisSpeeds;
    }

    
}