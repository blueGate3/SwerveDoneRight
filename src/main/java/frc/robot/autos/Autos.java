package frc.robot.autos;

import java.util.Optional;

import choreo.Choreo;
import choreo.trajectory.SwerveSample;
import choreo.trajectory.Trajectory;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.subsystems.Drivetrain;

public class Autos extends SubsystemBase{
    private final Drivetrain drivetrain = new Drivetrain();

    private final Timer timer = new Timer();
    private final Optional<Trajectory<SwerveSample>> trajectory = Choreo.loadTrajectory("SquarePath");

    public Autos() {
        
    }
    public void periodic() {

    }

}
