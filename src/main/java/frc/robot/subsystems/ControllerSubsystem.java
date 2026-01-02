package frc.robot.subsystems;

import edu.wpi.first.wpilibj.GenericHID.RumbleType;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import edu.wpi.first.wpilibj2.command.button.CommandPS5Controller;

public class ControllerSubsystem extends SubsystemBase {
    CommandPS5Controller m_controller = new CommandPS5Controller(1);
    public ControllerSubsystem() {
    }

    public void setRumble(RumbleType rumbleType, double rumble) {
        m_controller.setRumble(rumbleType, rumble);
    }
}
