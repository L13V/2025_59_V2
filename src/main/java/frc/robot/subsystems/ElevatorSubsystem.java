package frc.robot.subsystems;

import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

public class ElevatorSubsystem extends SubsystemBase{
    public enum ElevatorState {
        L1,
        L2,
        L3,
        L4,
        STOW,
        FLOOR_INTAKE,
        BARGE
    }
    private ElevatorState state = ElevatorState.STOW;
    public void initialize() {
        System.out.println("ElevatorSubsystem Initialized");

    }
    public ElevatorSubsystem() {
        System.out.println("ElevatorSubsystem Initialized");
    }
    public void setElevatorState(ElevatorState setto) {
        state = setto;
    }
    public Command moveTo(ElevatorState moveto) {
        return runOnce(() -> setElevatorState(moveto));
    }

    public void update() {
        switch (state) {
            case L1:
                System.out.println("L1");
            case L2:
                System.out.println("L2");

            case L3:
                System.out.println("L3");

            case L4:
                System.out.println("L4");

            case STOW:
                System.out.println("STOW");

            case FLOOR_INTAKE:
                System.out.println("FLOOR");
            case BARGE:
                System.out.println("BARGE");

        }
        SmartDashboard.putString("test",state.toString());
    }
}
