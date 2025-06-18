package frc.robot.subsystems;

import com.ctre.phoenix6.configs.CANrangeConfiguration;
import com.ctre.phoenix6.configs.CommutationConfigs;
import com.ctre.phoenix6.configs.TalonFXConfiguration;
import com.ctre.phoenix6.controls.PositionDutyCycle;
import com.ctre.phoenix6.hardware.CANrange;
import com.ctre.phoenix6.hardware.TalonFX;
import com.ctre.phoenix6.signals.AdvancedHallSupportValue;
import com.ctre.phoenix6.signals.FeedbackSensorSourceValue;
import com.ctre.phoenix6.signals.InvertedValue;
import com.ctre.phoenix6.signals.MotorArrangementValue;
import com.ctre.phoenix6.signals.UpdateModeValue;
import com.revrobotics.AbsoluteEncoder;
import com.revrobotics.spark.SparkBase.ControlType;
import com.revrobotics.spark.SparkBase.PersistMode;
import com.revrobotics.spark.SparkBase.ResetMode;
import com.revrobotics.spark.SparkClosedLoopController;
import com.revrobotics.spark.SparkLowLevel.MotorType;
import com.revrobotics.spark.SparkMax;
import com.revrobotics.spark.config.ClosedLoopConfig.FeedbackSensor;
import com.revrobotics.spark.config.SparkMaxConfig;

import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants.BallIntakeConstants;
import frc.robot.Constants.EndevatorConstants;

public class BallIntakeSubsystem extends SubsystemBase {

    // Pivot Motor and PiD
    // ClimbSubsystem m_climb_subsystem = new ClimbSubsystem();
    // private final SparkMax PivotMotor = new
    // SparkMax(BallIntakeConstants.bi_pivot_motor_id, MotorType.kBrushless);
    // private final SparkClosedLoopController pid =
    // PivotMotor.getClosedLoopController();
    private final TalonFX pivot_motor = new TalonFX(BallIntakeConstants.bi_pivot_motor_id);
    private final TalonFXConfiguration pivot_motor_config = new TalonFXConfiguration();
    private final CommutationConfigs pivot_motor_commutation = new CommutationConfigs();
    private final PositionDutyCycle pivot_motor_PositionDutyCycle = new PositionDutyCycle(
            BallIntakeConstants.bi_stow_position);
    // Roller Motor
    private final TalonFX rollerMotor_right = new TalonFX(BallIntakeConstants.bi_roller_motor_id);

    // Through Bore Encoder
    // private final AbsoluteEncoder abs_encoder = PivotMotor.getAbsoluteEncoder();
    // SparkMaxConfig config = new SparkMaxConfig();

    //
    static CANrange algae_range = new CANrange(BallIntakeConstants.bi_range_id);
    static CANrangeConfiguration algae_range_config = new CANrangeConfiguration();

    public BallIntakeSubsystem() {
        /*
         * Back Ball Intake Pivot Config
         */
        pivot_motor.getConfigurator().refresh(pivot_motor_config);
        pivot_motor_config.MotorOutput.Inverted = InvertedValue.Clockwise_Positive;
        pivot_motor_config.Slot0.kP = BallIntakeConstants.bi_slot0_kP;
        pivot_motor_config.Slot0.kI = BallIntakeConstants.bi_slot0_kI;
        pivot_motor_config.Slot0.kD = BallIntakeConstants.bi_slot0_kD;
        pivot_motor_config.Slot0.kA = BallIntakeConstants.bi_slot0_kA;
        pivot_motor_config.Slot0.kV = BallIntakeConstants.bi_slot0_kV;
        pivot_motor_config.Slot0.kS = BallIntakeConstants.bi_slot0_kS;
        pivot_motor_config.Feedback.FeedbackSensorSource = FeedbackSensorSourceValue.FusedCANcoder;
        pivot_motor_config.Feedback.FeedbackRemoteSensorID = BallIntakeConstants.bi_pivot_encoder_id;
        pivot_motor_config.Feedback.RotorToSensorRatio = BallIntakeConstants.bi_rotor_to_sensor_ratio;
        pivot_motor_config.Feedback.SensorToMechanismRatio = BallIntakeConstants.bi_sensor_to_mechanism_ratio;
        pivot_motor_config.SoftwareLimitSwitch.ForwardSoftLimitEnable = true;
        pivot_motor_config.SoftwareLimitSwitch.ReverseSoftLimitEnable = true;
        pivot_motor_config.SoftwareLimitSwitch.ForwardSoftLimitThreshold = BallIntakeConstants.bi_fw_soft_limit;
        pivot_motor_config.SoftwareLimitSwitch.ReverseSoftLimitThreshold = BallIntakeConstants.bi_rev_soft_limit;
        pivot_motor.getConfigurator().apply(pivot_motor_config);


        /*
         * Back Ball Intake Algae CANRange Config
         */
        algae_range.getConfigurator().refresh(algae_range_config);
        algae_range_config.ProximityParams.MinSignalStrengthForValidMeasurement = 2000;
        algae_range_config.ProximityParams.ProximityThreshold = 0.3;
        algae_range_config.ToFParams.UpdateMode = UpdateModeValue.ShortRange100Hz;
        algae_range.getConfigurator().apply(algae_range_config);
    }

    public enum BallIntakeState {
        STOW,
        HOLD,
        INTAKE,
        SCORE,
        CLIMB
    }

    public BallIntakeState state = BallIntakeState.STOW;

    /**
     * FUNCTION for setting the elevator state machine to another state.
     * 
     * @param setstateto
     */
    public void setState(BallIntakeState setstateto) {
        state = setstateto;

    }

    /**
     * COMMAND for moving the elevator. Usually accessed by the controller, and
     * eventually sets the state of the state machine.
     * 
     * @param setto
     * @return Command
     */
    public Command setTo(BallIntakeState setto) {
        return runOnce(() -> setState(setto));
    }

    public BallIntakeState getCurrentState() {
        return state;
    }

    public boolean hasAlgae() {
        return algae_range.getIsDetected(true).getValue();
    }

    public void moveRollerByPower(double power) {
        if (Math.abs(power) > 1) {
            rollerMotor_right.set(1);
        } else {
            if (power > 0) {
                rollerMotor_right.set(power);

            } else {
                rollerMotor_right.set(power);
            }
        }
    }

    public void moveIntakeToPosition(double position) {
        pivot_motor.setControl(pivot_motor_PositionDutyCycle.withPosition(position));
    }

    public double getPosition() {
        return pivot_motor.getPosition().getValueAsDouble();
    }

    public void periodic() {
        switch (state) {
            case STOW -> {
                if (hasAlgae()) {
                    // m_climb_subsystem.ballstop();
                    moveIntakeToPosition(BallIntakeConstants.bi_algae_stow_position);
                    moveRollerByPower(BallIntakeConstants.bi_hold_power);
                } else {
                    moveIntakeToPosition(BallIntakeConstants.bi_stow_position);
                    moveRollerByPower(BallIntakeConstants.bi_idle_power);
                }

            }
            case HOLD -> {
                // moveIntakeToPosition(BallIntakeConstants.bi_algae_stow_position);
                // moveRollerByPower(BallIntakeConstants.bi_hold_power);

            }
            case INTAKE -> {
                moveIntakeToPosition(BallIntakeConstants.bi_algae_intake_position);
                moveRollerByPower(BallIntakeConstants.bi_intake_power);
                // m_climb_subsystem.ballstop();
                if (hasAlgae()) {
                    state = BallIntakeState.STOW;
                }
            }
            case SCORE -> {
                moveIntakeToPosition(BallIntakeConstants.bi_algae_score_position);
                moveRollerByPower(BallIntakeConstants.bi_outtake_power);
            }
            case CLIMB -> {
                moveIntakeToPosition(BallIntakeConstants.bi_climb_position);
                moveRollerByPower(0);
            }

        }
        SmartDashboard.putString("Ball Intake State", state.toString());
        SmartDashboard.putBoolean("Ball Intake HasAlgae", hasAlgae());

    }
}
