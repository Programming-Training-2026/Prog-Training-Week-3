package frc.robot.subsystems;

import com.ctre.phoenix6.configs.TalonFXConfiguration;
import com.ctre.phoenix6.controls.MotionMagicVoltage;
import com.ctre.phoenix6.controls.PositionDutyCycle;
import com.ctre.phoenix6.controls.PositionVoltage;
import com.ctre.phoenix6.hardware.TalonFX;

import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.filter.Debouncer;
import edu.wpi.first.math.filter.Debouncer.DebounceType;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

public class ExtensionSubsystem extends SubsystemBase{
    private final TalonFX extensionMotor; 

    private final double RETRACTED_POSITION = 0.0; 
    private final double EXTENDED_POSITION = 18.0; 

    private final Debouncer stallDebouncer = new Debouncer(0.25, DebounceType.kRising); 
    
    private final PIDController pidController = new PIDController(0, 0, 0); 

    private final PositionVoltage positionOutput = new PositionVoltage(0); 

    private final MotionMagicVoltage mmPositionOutput = new MotionMagicVoltage(0); 

    
    private double kp = 0.03; 
    public ExtensionSubsystem() {
        extensionMotor = new TalonFX(20); 
        extensionMotor.setPosition(0); 

        pidController.setTolerance(1); 

        SmartDashboard.putNumber("Extension kP", kp); 

        configureMotor();
    }

    public void configureMotor() {
        TalonFXConfiguration config = new TalonFXConfiguration(); 
        config.Slot0.kS = 0.1; 
        config.Slot0.kV = 0.12; 
        config.Slot0.kA = 0.001; 
        config.Slot0.kG = 0; 
        
        config.Slot0.kP = 0.05; 
        
        config.MotionMagic.MotionMagicCruiseVelocity = 9; 
        config.MotionMagic.MotionMagicAcceleration = 9; 
        config.MotionMagic.MotionMagicJerk = 0; 

        extensionMotor.getConfigurator().apply(config); 
    }
 

    public Command extend() {
        return runOnce(() -> {
            extensionMotor.setControl(
                mmPositionOutput.withPosition(EXTENDED_POSITION).withSlot(0)
            );
        })
        .until(() -> extensionMotor.getMotionMagicAtTarget().getValue()) // Returns true when done
        .finallyDo((interrupted) -> {
            // Do NOT call stopMotor() if you want the PID to keep holding the position
            // Especially important for stuff like elevators
        }); 
    }

    public Command retract() {
        return runOnce(() -> {
            extensionMotor.setControl(
                mmPositionOutput.withPosition(RETRACTED_POSITION).withSlot(0)
            );
        })
        .until(() -> extensionMotor.getMotionMagicAtTarget().getValue()) // Returns true when done
        .finallyDo((interrupted) -> {
            // Do NOT call stopMotor() if you want the PID to keep holding the position
            // Especially important for stuff like elevators
        }); 
    }

    public boolean checkEnd(boolean isExtending) {
        boolean isAtTarget = pidController.atSetpoint(); 
        // if (isExtending) {
        //     isAtTarget = Math.abs(getPosition() - EXTENDED_POSITION) <= 1; 
        // } else {
        //     isAtTarget = getPosition() <= 1; 
        // }

        double velocity = extensionMotor.getVelocity().getValueAsDouble(); 
        boolean isStalled = stallDebouncer.calculate(Math.abs(velocity) < 0.05); 

        return isStalled || isAtTarget; 
        
    }

    public double getPosition() {
        return extensionMotor.getPosition().getValueAsDouble(); 
    }

    @Override
    public void periodic() {
       kp = SmartDashboard.getNumber("Extension kP", 0.05); 
    }
}