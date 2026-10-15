package frc.robot.subsystems;

import com.ctre.phoenix6.configs.TalonFXConfiguration;
import com.ctre.phoenix6.hardware.TalonFX;
import com.ctre.phoenix6.signals.InvertedValue;
import com.ctre.phoenix6.signals.NeutralModeValue;

import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

public class KickerSubsystem extends SubsystemBase{
    private final TalonFX kickerMotor; 

    public KickerSubsystem() {
        kickerMotor = new TalonFX(23); 
    }

    public void configureMotor() {
        TalonFXConfiguration config = new TalonFXConfiguration(); 

        config.CurrentLimits.StatorCurrentLimitEnable = true; 
        config.CurrentLimits.StatorCurrentLimit = 60;  // Amps allowed at the motor

        config.CurrentLimits.SupplyCurrentLimitEnable = true; 
        config.CurrentLimits.SupplyCurrentLimit = 40; // Amps allowed from the battery

        config.MotorOutput.NeutralMode = NeutralModeValue.Coast; // Slows down and stops if the speed of motor is 0
        // config.MotorOutput.NeutralMode = NeutralModeValue.Brake; // Instantly stops if the speed of motor is 0

        config.MotorOutput.Inverted = InvertedValue.Clockwise_Positive;  // Positive num spins this counter clockwise
    }

    public Command runKicker() {
        // Spin the motor at six volts
        return run(                     
            () -> kickerMotor.setVoltage(5)
        ).finallyDo(
            () -> kickerMotor.stopMotor()
        ); 


    }

    public Command runKicker2() {
        // Spin the motor at six volts
        return Commands.run(
            () -> kickerMotor.setVoltage(5), this
        ); 

        
    }
}
