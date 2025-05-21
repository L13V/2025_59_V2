package frc.robot.subsystems;

import com.ctre.phoenix6.hardware.TalonFX;
import com.revrobotics.spark.SparkMax;
import com.revrobotics.spark.SparkMaxAlternateEncoder;

import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

import com.revrobotics.spark.SparkBase.PersistMode;
import com.revrobotics.spark.SparkBase.ResetMode;
import com.revrobotics.spark.SparkLowLevel.MotorType;
import com.revrobotics.spark.SparkBase;
import com.revrobotics.spark.SparkClosedLoopController;
import com.revrobotics.spark.config.ClosedLoopConfig;
import com.revrobotics.spark.config.SparkBaseConfig;
import com.revrobotics.spark.config.SparkBaseConfig.IdleMode;
import com.revrobotics.spark.config.SparkMaxConfig;
import com.revrobotics.spark.config.ClosedLoopConfig.FeedbackSensor;

public class EndEvatorSubsystem extends SubsystemBase {
    final SparkMax test_motor = new SparkMax(8, MotorType.kBrushless);
    final ClosedLoopConfig test_motor_closed_loop_config = new ClosedLoopConfig().pidf(0, 0, 0, 0.0010905125408942202835332606325).feedbackSensor(FeedbackSensor.kPrimaryEncoder).velocityFF(0.0010905125408942202835332606325);
    final SparkClosedLoopController test_motor_closed = test_motor.getClosedLoopController();
    final SparkBaseConfig test_motor_config = new SparkMaxConfig().idleMode(IdleMode.kCoast).inverted(false).apply(test_motor_closed_loop_config);



    // Required initialization crap
    public void initialize() {
        System.out.println("ElevatorSubsystem Initialized");

    }

    public EndEvatorSubsystem() {
        System.out.println("ElevatorSubsystem Initialized");
        test_motor.configure(test_motor_config, ResetMode.kResetSafeParameters,PersistMode.kPersistParameters);
    }

    // Initialize ElevatorState Enum, and start at stow
    public enum ElevatorState {
        L1,
        L2,
        L3,
        L4,
        L4_Score,
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
                test_motor.set(0.1);
            }
            case L2 -> {
                System.out.println("L2");
                test_motor.set(-0.2);
            }
            case L3 -> {
                System.out.println("L3");
                test_motor.set(0.3);
            }
            case L4 -> {
                System.out.println("L4");
                test_motor.set(-0.4);

            }
            case L4_Score -> {
                System.out.println("L4_Score");
                test_motor.set(0.5);
            }
            case STOW -> {
                System.out.println("STOW");
                test_motor_closed.setReference(100,SparkBase.ControlType.kVelocity);

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
        SmartDashboard.putNumber("Current", test_motor.getAppliedOutput());
    }
}
