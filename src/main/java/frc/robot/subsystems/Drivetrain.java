package frc.robot.subsystems;

import java.io.IOException;

import org.json.simple.parser.ParseException;

import com.pathplanner.lib.auto.AutoBuilder;
import com.pathplanner.lib.config.PIDConstants;
import com.pathplanner.lib.config.RobotConfig;
import com.pathplanner.lib.controllers.PPHolonomicDriveController;
import com.studica.frc.AHRS;
import com.studica.frc.AHRS.NavXComType;

import choreo.trajectory.SwerveSample;
import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.math.kinematics.SwerveDriveKinematics;
import edu.wpi.first.math.kinematics.SwerveDriveOdometry;
import edu.wpi.first.math.kinematics.SwerveModulePosition;
import edu.wpi.first.math.kinematics.SwerveModuleState;
import edu.wpi.first.networktables.NetworkTableInstance;
import edu.wpi.first.networktables.StructPublisher;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants.DriveConst;

public class Drivetrain extends SubsystemBase{
    private final PIDController xController = new PIDController(10.0, 0.0, 0.0);
    private final PIDController yController = new PIDController(10.0, 0.0, 0.0);
    private final PIDController rotController = new PIDController(7.5, 0.0, 0.0);

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
    private SwerveModuleState[] m_robotState = {m_frontLeft.getState(), m_frontRight.getState(), m_backLeft.getState(), m_backRight.getState()};
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
        rotController.enableContinuousInput(-Math.PI, Math.PI);
         try {
            AutoBuilder.configure(
                this::getOdometry, // Robot pose supplier
                this::resetOdometry, // Method to reset odometry (will be called if your auto has a starting pose)
                this::getChassisSpeeds, // ChassisSpeeds supplier. MUST BE ROBOT RELATIVE
                (speeds, feedforwards) -> driveWithChassisSpeeds(speeds), // Method that will drive the robot given ROBOT RELATIVE ChassisSpeeds. Also optionally outputs individual module feedforwards
                new PPHolonomicDriveController( // PPHolonomicController is the built in path following controller for holonomic drive trains
                        new PIDConstants(5.0, 0.0, 0.0), // Translation PID constants
                        new PIDConstants(5.0, 0.0, 0.0) // Rotation PID constants
                ),
                RobotConfig.fromGUISettings(), // The robot configuration
                () -> {
                  var alliance = DriverStation.getAlliance();
                  if (alliance.isPresent()) {
                    return alliance.get() == DriverStation.Alliance.Red;
                  }
                  return false;
                },
                this // Reference to this subsystem to set requirements
            );
        } catch (IOException e) {
            e.printStackTrace();
        } catch (ParseException e) {
            e.printStackTrace();
        }
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
        SwerveDriveKinematics.desaturateWheelSpeeds(m_swerveModuleStates, DriveConst.kMaxSpeed);
        m_frontLeft.setDesiredState(m_swerveModuleStates[0]);
        m_frontRight.setDesiredState(m_swerveModuleStates[1]);
        m_backLeft.setDesiredState(m_swerveModuleStates[2]);
        m_backRight.setDesiredState(m_swerveModuleStates[3]);
    }

    public ChassisSpeeds getChassisSpeeds() {
        m_robotState[0] = m_frontLeft.getState();
        m_robotState[1] = m_frontRight.getState();
        m_robotState[2] = m_backLeft.getState();
        m_robotState[3] = m_backRight.getState();
        return m_kinematics.toChassisSpeeds(m_robotState); //look here if thing break - could be that this is returning desired states and not actual ones.
    }

    public void followTrajectory(SwerveSample sample) {
        Pose2d pose = m_odometry.getPoseMeters();
        ChassisSpeeds speeds = new ChassisSpeeds(
            sample.vx + xController.calculate(pose.getX(), sample.x),
            sample.vy + yController.calculate(pose.getY(), sample.y),
            sample.omega + rotController.calculate(pose.getRotation().getRadians(), sample.heading)
        );
        driveWithChassisSpeeds(speeds);
    }
    public Pose2d getOdometry() {
        return m_odometry.getPoseMeters();
    }
    public void resetOdometry(Pose2d pose) {
        m_odometry.resetPose(pose);
    }
}
