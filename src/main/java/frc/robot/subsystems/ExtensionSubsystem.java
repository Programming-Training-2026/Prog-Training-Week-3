package frc.robot.subsystems;

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
    private double kp = 0.03; 

    public ExtensionSubsystem() {
        extensionMotor = new TalonFX(20); 
        extensionMotor.setPosition(0); 

        pidController.setP(kp); 

        SmartDashboard.putNumber("Extension kP", kp); 
    }

    public Command extend() {
        return run(
            () -> {
                double outputVoltage = pidController.calculate(getPosition(), EXTENDED_POSITION); 
                extensionMotor.setVoltage(outputVoltage); 
            }
        ).until(
            () -> checkEnd(true)
        ).finallyDo(
            () -> extensionMotor.stopMotor()
        ); 
    }

    public Command retract() {
        return run(
            () -> {
                double outputVoltage = pidController.calculate(getPosition(), RETRACTED_POSITION); 
                extensionMotor.setVoltage(outputVoltage); 
            }
        ).until(
            () -> checkEnd(false)
        ).finallyDo(
            () -> extensionMotor.stopMotor()
        ); 
    }

    public boolean checkEnd(boolean isExtending) {
        boolean isAtTarget; 
        if (isExtending) {
            isAtTarget = Math.abs(getPosition() - EXTENDED_POSITION) <= 1; 
        } else {
            isAtTarget = getPosition() <= 1; 
        }

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