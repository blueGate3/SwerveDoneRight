package frc.robot.subsystems;

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
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants.DriveConst;

public class Drivetrain extends SubsystemBase{

    private final SwerveDriveOdometry m_odometry;
    private final StructPublisher<Pose2d> posePub = NetworkTableInstance.getDefault()
        .getStructTopic("Robot/CurrentPose", Pose2d.struct).publish();

    private final AHRS navx = new AHRS(NavXComType.kMXP_SPI); 
    private final Translation2d m_frontRightLocation = new Translation2d( -0.305, 0.305);//side length total is at 29.5 inches including modules. Divided by 2 and set to meters is .37465 meters from one side to the tip of the module, minus a bit bc module is only like 1/2 distance. 
    private final Translation2d m_frontLeftLocation = new Translation2d(0.305,  0.305);//the frc kinematics section has the coordinates so x is front-back, where front is positive, and y is left-right, where left is positive. it's communist to the extreme but will affect the way we initialize our module locations.
    private final Translation2d m_backLeftLocation = new Translation2d(0.305,  -0.305);//continued: that's the reason for the strange abnormal abhorrent disgusting affronts-before-God translation signs. 
    private final Translation2d m_backRightLocation = new Translation2d( -0.305, -0.305);

    // Constructor for each swerve module
    private final SwerveModule m_frontLeft = new SwerveModule(DriveConst.FLDrive, DriveConst.FLTURN); //
    private final SwerveModule m_frontRight = new SwerveModule(DriveConst.FRDrive, DriveConst.FRTurn); //
    private final SwerveModule m_backRight = new SwerveModule(DriveConst.BRDrive, DriveConst.BRTurn); //
    private final SwerveModule m_backLeft = new SwerveModule(DriveConst.BLDrive, DriveConst.BLTurn); //

    // Swerve Drive Kinematics (note the ordering [frontRight, frontLeft, backLeft, backRight] [counterclockwise from the frontRight])
    private SwerveModuleState[] m_swerveModuleStates;
    private final SwerveDriveKinematics m_kinematics = new SwerveDriveKinematics(m_frontRightLocation, m_frontLeftLocation, m_backLeftLocation, m_backRightLocation);

    private SwerveModulePosition[] m_positions = {
        m_frontRight.getPosition(),
        m_frontLeft.getPosition(),
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
        posePub.set(m_odometry.getPoseMeters());
    }

    public void drive(double x, double y, double rot, boolean fieldRelative) {
        //TODO it looks like fromFieldRelative and fromRobotRelative might be the opposite of what we'd expect... if it breaks look here and try to flip.
        if (fieldRelative) {
            m_swerveModuleStates = m_kinematics.toSwerveModuleStates(
                ChassisSpeeds.fromFieldRelativeSpeeds(x, y, rot, new Rotation2d(Math.toRadians(navx.getAngle())))
            );  
        } else {
            m_swerveModuleStates = m_kinematics.toSwerveModuleStates(
                ChassisSpeeds.fromRobotRelativeSpeeds(x, y, rot, new Rotation2d(Math.toRadians(navx.getAngle())))
            );  
        }
        SwerveDriveKinematics.desaturateWheelSpeeds(m_swerveModuleStates, DriveConst.kMaxSpeed);
            m_frontRight.setDesiredState(m_swerveModuleStates[0]);
            m_frontLeft.setDesiredState(m_swerveModuleStates[1]);
            m_backLeft.setDesiredState(m_swerveModuleStates[2]);
            m_backRight.setDesiredState(m_swerveModuleStates[3]);
    }

    public void driveWithChassisSpeeds(ChassisSpeeds chassisSpeeds) {
        m_swerveModuleStates = m_kinematics.toSwerveModuleStates(chassisSpeeds);
        m_frontRight.setDesiredState(m_swerveModuleStates[0]);
        m_frontLeft.setDesiredState(m_swerveModuleStates[1]);
        m_backLeft.setDesiredState(m_swerveModuleStates[2]);
        m_backRight.setDesiredState(m_swerveModuleStates[3]);
    }
}
