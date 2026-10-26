package frc.robot.subsystems;

import com.ctre.phoenix6.configs.TalonFXConfiguration;
import com.ctre.phoenix6.controls.MotionMagicVelocityVoltage;
import com.ctre.phoenix6.controls.VelocityVoltage;
import com.ctre.phoenix6.hardware.TalonFX;
import com.revrobotics.spark.SparkFlex;
import com.revrobotics.spark.SparkLowLevel.MotorType;

import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

public class ShooterSubsystem extends SubsystemBase{
    private final TalonFX rightMotor; 
    
    private double RPS = 4.0; 
    private final VelocityVoltage velocityOutput = new VelocityVoltage(0);
    private final MotionMagicVelocityVoltage mmVelocityOutput = new MotionMagicVelocityVoltage(0); 

    public ShooterSubsystem() {
        rightMotor = new TalonFX(30); 

        SmartDashboard.putNumber("Shooter RPS", RPS); 
        configureMotor();
    }

    public void configureMotor() {
        TalonFXConfiguration config = new TalonFXConfiguration(); 
        config.Slot0.kS = 0.1; 
        config.Slot0.kV = 0.12; 
        config.Slot0.kA = 0.001; 
        config.Slot0.kG = 0; 
        
        config.Slot1.kA = 5; 

        config.MotionMagic.MotionMagicAcceleration = 30;
        
        rightMotor.getConfigurator().apply(config); 
    }

    public Command runShooting() {
        return run(
            () -> {
                rightMotor.setControl(
                    mmVelocityOutput.withVelocity(RPS)
                ); 
            }
        ).finallyDo(
            () -> rightMotor.stopMotor()
        ); 
    }

    @Override
    public void periodic() {
        RPS = SmartDashboard.getNumber("Shooter RPS", 0); 
    }
}
