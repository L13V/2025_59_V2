package frc.robot.subsystems;

import com.ctre.phoenix6.hardware.TalonFX;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

public class ElevatorSubsystem extends SubsystemBase {
    private final TalonFX elevator_test = new TalonFX(6, "rio");

    // Required initialization crap
    public void initialize() {
        System.out.println("ElevatorSubsystem Initialized");

    }

    public ElevatorSubsystem() {
        System.out.println("ElevatorSubsystem Initialized");

    }

    // Initialize ElevatorState Enum, and start at stow
    public enum ElevatorState {
        L1,
        L2,
        L3,
        L4,
        STOW,
        FLOOR_INTAKE,
        BARGE
    }

    public ElevatorState state = ElevatorState.STOW;

    /**
     * Sets the elevator state machine to another state.
     * 
     * @param setto
     */
    public void setElevatorState(ElevatorState setto) {
        state = setto;
    }

    
    /** 
     * Command for moving the elevator. Usually accessed by the controller, and eventually sets the state of the state machine.
     * @param moveto
     * @return Command
     */
    public Command moveTo(ElevatorState moveto) {
        return runOnce(() -> setElevatorState(moveto));
    }

    // Checking State Machine States
    public ElevatorState getCurrenElevatorState() {
        return state;
    }

    // Booleans
    public Boolean readyToStow() {
        return getCurrenElevatorState() == (ElevatorState.L2);
    }

    // State Machine Garbage
    public void periodic() {
        switch (state) {
            case L1 -> {
                System.out.println("L1");
            }
            case L2 -> {
                System.out.println("L2");
                elevator_test.set(1);

            }
            case L3 -> {
                System.out.println("L3");
            }
            case L4 -> {
                System.out.println("L4");
            }
            case STOW -> {
                System.out.println("STOW");
            }
            case FLOOR_INTAKE -> {
                System.out.println("FLOOR");
            }
            case BARGE -> {
                System.out.println("BARGE");
            }

        }
        SmartDashboard.putString("State", state.toString());
        SmartDashboard.putBoolean("ReadyToStow", readyToStow());
    }
}
