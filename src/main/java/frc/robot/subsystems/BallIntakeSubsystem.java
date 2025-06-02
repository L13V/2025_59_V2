package frc.robot.subsystems;

import com.ctre.phoenix6.configs.CANrangeConfiguration;
import com.ctre.phoenix6.hardware.CANrange;
import com.ctre.phoenix6.hardware.TalonFX;
import com.ctre.phoenix6.signals.UpdateModeValue;
import com.revrobotics.AbsoluteEncoder;
import com.revrobotics.spark.SparkBase.PersistMode;
import com.revrobotics.spark.SparkBase.ResetMode;
import com.revrobotics.spark.SparkClosedLoopController;
import com.revrobotics.spark.SparkLowLevel.MotorType;
import com.revrobotics.spark.SparkMax;
import com.revrobotics.spark.config.ClosedLoopConfig.FeedbackSensor;
import com.revrobotics.spark.config.SparkMaxConfig;

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
        // rollerMotor_left.setControl(new
        // Follower(BallIntake_constant.rollers_right_motor_ID, false));
        // talonFXConfigurator.refresh(fx_cfg);
        // fx_cfg.Feedback.SensorToMechanismRatio = 1.0;
        // fx_cfg.Feedback.RotorToSensorRatio = 12.8;
        // talonFXConfigurator.apply(fx_cfg);

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

    public boolean isCanAlgaeDetected(){
                
        var distance = algaeCanRange.getDistance();
        var signalStrength = algaeCanRange.getSignalStrength();
        // System.out.println("Distance is " + distance.toString() + " with a signal strength of " + signalStrength + " and " + distance.getTimestamp().getLatency() + " seconds of latency");

        // boolean canRangeCoralInterval = canRangeCoral.getIsDetected().getValue();
        // GE_canRangeCoralInterval.setBoolean(canRangeCoralInterval);
        // if (canRangeCoralInterval) {
        //     GE_canRangeCoralInterval.setDouble(canRangeCoral.getDistance().getValueAsDouble());
        // } else {
        //     GE_canRangeCoralInterval.setDouble(0.0);
        // }
        /**
         * Get the isDetected StatusSignalValue without refreshing
         */
        var isDetected = algaeCanRange.getIsDetected(false);
        /* This time wait for the signal to reduce latency */
        isDetected.waitForUpdate(PRINT_PERIOD); // Wait up to our period
        /**
         * This uses the explicit getValue and getUnits functions to print, even though it's not
         * necessary for the ostream print
         */
        // System.out.println(
        //     "Is Detected is " +
        //     isDetected.getValue() + " " +
        //     isDetected.getUnits() + " with " +
        //     isDetected.getTimestamp().getLatency() + " seconds of latency"
        // );ss
        // System.out.println();

        return isDetected.getValue();
    }
}
