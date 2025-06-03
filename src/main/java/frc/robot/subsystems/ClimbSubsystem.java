package frc.robot.subsystems;

import com.ctre.phoenix6.controls.PositionDutyCycle;
import com.ctre.phoenix6.hardware.TalonFX;

import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants.ClimbConstants;

public class ClimbSubsystem extends SubsystemBase {


    private final TalonFX climbMotor = new TalonFX(ClimbConstants.climb_motor_id);
    private PositionDutyCycle positionDutyCycleController = new PositionDutyCycle(ClimbConstants.climb_stow_position);

    public ClimbSubsystem() {

    }
    public enum ClimbStates {
        STOW,
        DEPLOYED,
        CLIMB,
        BALLINTAKE
    }

    public void moveClimbByPower(double power){
        if (power < 0){
            if (getPosition() >= 0.2){
                climbMotor.set(power);
            }else {
                climbMotor.set(0);
            }
        }else {
            if (getPosition() <= 4.5){
                climbMotor.set(power);
            }else {
                climbMotor.set(0);
            }           
        } 
    }
    public void moveClimbByPosition(double position){

        if ( getPosition() <= ClimbConstants.climb_deployed_position){
           climbMotor.setControl( positionDutyCycleController.withPosition(position));
        }
    }

    public double getPosition(){
        return climbMotor.getPosition().refresh().getValueAsDouble();
    }

    public void intakingClim (){
        moveClimbByPosition(ClimbConstants.climb_deployed_position);
    }
    

}
