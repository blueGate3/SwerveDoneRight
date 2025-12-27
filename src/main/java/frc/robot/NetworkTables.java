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