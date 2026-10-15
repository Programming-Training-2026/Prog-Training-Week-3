package frc.robot.subsystems;

import com.ctre.phoenix6.controls.Follower;
import com.ctre.phoenix6.hardware.TalonFX;
import com.ctre.phoenix6.signals.MotorAlignmentValue;

import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

public class IntakeSubsystem extends SubsystemBase{
    private final TalonFX leftMotor; 
    private final TalonFX rightMotor; 

    public IntakeSubsystem() {
        leftMotor = new TalonFX(21); 
        rightMotor = new TalonFX(40); 

        rightMotor.setControl(new Follower(leftMotor.getDeviceID(), MotorAlignmentValue.Opposed)); 
    }
    // Click a(), the intake runs forever
    // Click b(), the intake stops
    public Command runIntaking() {
        return run(
            () -> {
                leftMotor.setVoltage(-5); 
                //rightMotor.setVoltage(5); 
            }
        ); 
    }

    public Command stopIntaking() {
        return run(
            () -> {
                leftMotor.setVoltage(0); 
                //rightMotor.setVoltage(0); 
            }
        ); 
    }
}
