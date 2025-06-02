package frc.robot.subsystems;

import java.lang.Thread.State;

import com.ctre.phoenix6.configs.CANrangeConfiguration;
import com.ctre.phoenix6.hardware.CANrange;
import com.ctre.phoenix6.hardware.TalonFX;
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

import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants.BallIntakeConstants;

public class BallIntakeSubsystem extends SubsystemBase {

    // Pivot Motor and PiD
    private final SparkMax PivotMotor = new SparkMax(BallIntakeConstants.bi_pivot_motor_id, MotorType.kBrushless);
    private final SparkClosedLoopController pid = PivotMotor.getClosedLoopController();

    // Roller Motor
    private final TalonFX rollerMotor_right = new TalonFX(BallIntakeConstants.bi_roller_motor_id);

    // Booelan
    private boolean Balldetected = false;

    // Through Board Encoder
    private final AbsoluteEncoder abs_encoder = PivotMotor.getAbsoluteEncoder();
    SparkMaxConfig config = new SparkMaxConfig();

    //
    static CANrange algaeCanRange = new CANrange(BallIntakeConstants.bi_range_id);
    static final double PRINT_PERIOD = 0.5;

    public BallIntakeSubsystem() {
        config.closedLoop.feedbackSensor(FeedbackSensor.kAbsoluteEncoder);
        PivotMotor.configure(config, ResetMode.kNoResetSafeParameters, PersistMode.kPersistParameters);


        CANrangeConfiguration configAlgae = new CANrangeConfiguration();
        configAlgae.ProximityParams.MinSignalStrengthForValidMeasurement = 2000; // If CANrange has a signal strength of
                                                                                 // at least 2000, it is a valid
                                                                                 // measurement.
        configAlgae.ProximityParams.ProximityThreshold = 0.3; // If CANrange detects an object within 0.1 meters, it
                                                              // will trigger the "isDetected" signal.
        configAlgae.ToFParams.UpdateMode = UpdateModeValue.ShortRange100Hz; // Make the CANrange update as fast as
                                                                            // possible at 100 Hz. This requires
                                                                            // short-range mode.
        algaeCanRange.getConfigurator().apply(configAlgae);
    }
    public enum BallIntakeState {
        STOW,
        HOLD,
        INTAKE,
        SCORE
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


    

    public boolean isCanAlgaeDetected(){
                
        var distance = algaeCanRange.getDistance();
        var signalStrength = algaeCanRange.getSignalStrength();
        /**
         * Get the isDetected StatusSignalValue without refreshing
         */
        var isDetected = algaeCanRange.getIsDetected(false);
        /* This time wait for the signal to reduce latency */
        isDetected.waitForUpdate(PRINT_PERIOD); 


        return isDetected.getValue();
    }

    public void moveRollerByPower(double power){
        if (Math.abs(power) > 1){
            rollerMotor_right.set(1);
        }else {
            if(power > 0 ){
                rollerMotor_right.set(power);

            }else {
                rollerMotor_right.set(power);
            }    
        }
    }

    public void moveIntakeToPosition(double position){
            pid.setReference(position, ControlType.kPosition);
    }
    public double getPosition(){
        return abs_encoder.getPosition();
    }

    public void periodic(){
        switch (state) {
            case STOW -> {
                moveIntakeToPosition(BallIntakeConstants.bi_stow_position);
                moveRollerByPower(BallIntakeConstants.bi_idle_power);
            }
            case HOLD -> {
                moveIntakeToPosition(BallIntakeConstants.bi_algae_stow_position);
                moveRollerByPower(BallIntakeConstants.bi_hold_power);
            
            }
            case INTAKE ->{
                moveIntakeToPosition(BallIntakeConstants.bi_algae_intake_position);
                moveRollerByPower(BallIntakeConstants.bi_intake_power);
                if (isCanAlgaeDetected()){
                    state = BallIntakeState.HOLD;
                }
            }
            case SCORE ->{
                moveIntakeToPosition(BallIntakeConstants.bi_algae_score_position);
                moveRollerByPower(BallIntakeConstants.bi_outtake_power);
            }

    }


}
}

