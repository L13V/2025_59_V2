package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.BallIntakeSubsystem;
import frc.robot.subsystems.ClimbSubsystem;
import frc.robot.subsystems.BallIntakeSubsystem.BallIntakeState;
import frc.robot.subsystems.ClimbSubsystem.ClimbState;

public class TeleOp_Climb_Command extends Command{
    private BallIntakeSubsystem bi_subsystem;
    private ClimbSubsystem climb_subsystem;
    public enum climb_options {
        STARTING,
        DEPLOY,
        CLIMB
    };
    public climb_options selectedOption = climb_options.STARTING;
    public TeleOp_Climb_Command(BallIntakeSubsystem ballintake, ClimbSubsystem climb, climb_options out) {
        bi_subsystem = ballintake;
        climb_subsystem = climb;
        selectedOption = out;
        addRequirements(bi_subsystem,climb_subsystem);

    }
    @Override
    public void execute() {
        switch (selectedOption) {
            case STARTING -> {
                bi_subsystem.state = BallIntakeState.STOW;
                climb_subsystem.state = ClimbState.STOW;
            }
            case DEPLOY -> {
                climb_subsystem.state = ClimbState.DEPLOYED;
                bi_subsystem.state = BallIntakeState.CLIMB;
            }
            case CLIMB -> {
                climb_subsystem.state = ClimbState.CLIMB;
                bi_subsystem.state = BallIntakeState.CLIMB;
            }
        }
    }

}
