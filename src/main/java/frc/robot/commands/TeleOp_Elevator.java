// IGNORE THIS. It's just a placeholder.

package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.EndEvatorSubsystem;

public class TeleOp_Elevator extends Command {
    private EndEvatorSubsystem subsystem;
    public TeleOp_Elevator(EndEvatorSubsystem m_Subsystem) {
        subsystem = m_Subsystem;
        addRequirements(subsystem); 
    }
    public void execute() {
        // ElevatorSubsystem.elevatorMove();
    }
}
