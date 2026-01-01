package frc.robot;

import edu.wpi.first.networktables.DoublePublisher;
import edu.wpi.first.networktables.NetworkTable;
import edu.wpi.first.networktables.NetworkTableInstance;

public class NetworkTables {
    public DoublePublisher xPub;
    public DoublePublisher gyroReading;
    
    public NetworkTables() {
        NetworkTableInstance inst = NetworkTableInstance.getDefault();

        NetworkTable swerveTable = inst.getTable("swerveData");
        xPub = swerveTable.getDoubleTopic("x").publish();
        gyroReading = swerveTable.getDoubleTopic("GyroValue(Degrees)").publish();

        NetworkTable limelightTable = inst.getTable("limelightData");
    }
}
//https://docs.wpilib.org/en/stable/docs/software/networktables/robot-program.html

        //land of misfit toys (might reimplement in other ways later.)
        // SmartDashboard.putNumber("FL m/s", m_frontLeft.getState().speedMetersPerSecond);
        // SmartDashboard.putNumber("FR m/s", m_frontRight.getState().speedMetersPerSecond);
        // SmartDashboard.putNumber("BL m/s", m_backLeft.getState().speedMetersPerSecond);
        // SmartDashboard.putNumber("BR m/s", m_backRight.getState().speedMetersPerSecond);

        // SmartDashboard.putNumber("FrontLeftTurn", m_frontLeft.getPosition().angle.getDegrees());
        // SmartDashboard.putNumber("FrontRightTurn", m_frontRight.getPosition().angle.getDegrees());
        // SmartDashboard.putNumber("BackLeftTurn", m_backLeft.getPosition().angle.getDegrees());
        // SmartDashboard.putNumber("BackRightTurn", m_backRight.getPosition().angle.getDegrees());
        // SmartDashboard.putNumber("NavX Reading", navx.getRotation2d().getDegrees());