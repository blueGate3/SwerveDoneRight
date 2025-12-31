package frc.robot.subsystems;

import static edu.wpi.first.units.Units.Rotation;

import com.studica.frc.AHRS;
import com.studica.frc.AHRS.NavXComType;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.math.kinematics.Odometry;
import edu.wpi.first.math.kinematics.SwerveDriveKinematics;
import edu.wpi.first.math.kinematics.SwerveDriveOdometry;
import edu.wpi.first.math.kinematics.SwerveModulePosition;
import edu.wpi.first.math.kinematics.SwerveModuleState;
import edu.wpi.first.networktables.NetworkTableInstance;
import edu.wpi.first.networktables.StructPublisher;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants.DriveConst;

public class Drivetrain extends SubsystemBase{

    private final SwerveDriveOdometry m_odometry;
    private final StructPublisher<Pose2d> posePub = NetworkTableInstance.getDefault()
        .getStructTopic("Robot/CurrentPose", Pose2d.struct).publish();

    private final AHRS navx = new AHRS(NavXComType.kMXP_SPI); 
    
    private final Translation2d m_frontLeftLocation = new Translation2d(0.305,  0.305);
    private final Translation2d m_frontRightLocation = new Translation2d( 0.305, -0.305);
    private final Translation2d m_backLeftLocation = new Translation2d(-0.305,  0.305); 
    private final Translation2d m_backRightLocation = new Translation2d( -0.305, -0.305);

    // Constructor for each swerve module
    private final SwerveModule m_frontLeft = new SwerveModule(DriveConst.FLDrive, DriveConst.FLTURN); //
    private final SwerveModule m_frontRight = new SwerveModule(DriveConst.FRDrive, DriveConst.FRTurn); //
    private final SwerveModule m_backRight = new SwerveModule(DriveConst.BRDrive, DriveConst.BRTurn); //
    private final SwerveModule m_backLeft = new SwerveModule(DriveConst.BLDrive, DriveConst.BLTurn); //
    private final SwerveModule[] m_swerveModules = {m_frontLeft, m_frontRight, m_backLeft, m_backRight};

    private SwerveModuleState[] m_swerveModuleStates;
    private final SwerveDriveKinematics m_kinematics = new SwerveDriveKinematics(m_frontLeftLocation, m_frontRightLocation, m_backLeftLocation, m_backRightLocation);

    private SwerveModulePosition[] m_positions = {
        m_frontLeft.getPosition(),
        m_frontRight.getPosition(),
        m_backLeft.getPosition(),
        m_backRight.getPosition()
    };

    public Drivetrain() {
        m_odometry = new SwerveDriveOdometry(
            m_kinematics, 
            navx.getRotation2d(), 
            m_positions);
    }
    @Override
    public void periodic() {
        m_positions[0] = m_frontLeft.getPosition();
        m_positions[1] = m_frontRight.getPosition();
        m_positions[2] = m_backLeft.getPosition();
        m_positions[3] = m_backRight.getPosition();
        m_odometry.update(
            navx.getRotation2d(), 
            m_positions
        );
        posePub.set(m_odometry.getPoseMeters());

        SmartDashboard.putNumber("FrontLeftDriveSpeed", m_frontLeft.getState().speedMetersPerSecond);
        SmartDashboard.putNumber("FrontRightDriveSpeed", m_frontRight.getState().speedMetersPerSecond);
        SmartDashboard.putNumber("BackLeftDriveSpeed", m_backLeft.getState().speedMetersPerSecond);
        SmartDashboard.putNumber("BackRightDriveSpeed", m_backRight.getState().speedMetersPerSecond);

        SmartDashboard.putNumber("FrontLeftTurn", m_frontLeft.getPosition().angle.getDegrees());
        SmartDashboard.putNumber("FrontRightTurn", m_frontRight.getPosition().angle.getDegrees());
        SmartDashboard.putNumber("BackLeftTurn", m_backLeft.getPosition().angle.getDegrees());
        SmartDashboard.putNumber("BackRightTurn", m_backRight.getPosition().angle.getDegrees());
    SmartDashboard.putNumber("NavX Reading", navx.getRotation2d().getDegrees());
    }

    public void drive(double x, double y, double rot, boolean fieldRelative) {
        x *= DriveConst.kMaxSpeed;
        y *= DriveConst.kMaxSpeed;
        rot *= DriveConst.kModuleMaxAngularAcceleration;
        if (fieldRelative) {
            m_swerveModuleStates = m_kinematics.toSwerveModuleStates(
                ChassisSpeeds.fromFieldRelativeSpeeds(x, y, rot, navx.getRotation2d())
            );  
        } else {
            m_swerveModuleStates = m_kinematics.toSwerveModuleStates(
                ChassisSpeeds.fromRobotRelativeSpeeds(x, y, rot, navx.getRotation2d())
            );  
        }
        SwerveDriveKinematics.desaturateWheelSpeeds(m_swerveModuleStates, DriveConst.kMaxSpeed);
            m_frontLeft.setDesiredState(m_swerveModuleStates[0]);
            m_frontRight.setDesiredState(m_swerveModuleStates[1]);
            m_backLeft.setDesiredState(m_swerveModuleStates[2]);
            m_backRight.setDesiredState(m_swerveModuleStates[3]);
    }

    public void driveWithChassisSpeeds(ChassisSpeeds chassisSpeeds) {
        m_swerveModuleStates = m_kinematics.toSwerveModuleStates(chassisSpeeds);
        m_frontLeft.setDesiredState(m_swerveModuleStates[0]);
        m_frontRight.setDesiredState(m_swerveModuleStates[1]);
        m_backLeft.setDesiredState(m_swerveModuleStates[2]);
        m_backRight.setDesiredState(m_swerveModuleStates[3]);
    }
}
