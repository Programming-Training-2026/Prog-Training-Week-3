package frc.robot.subsystems;

import com.ctre.phoenix6.hardware.TalonFX;

import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

public class SpindexerSubsystem extends SubsystemBase{
    private final TalonFX spindexerMotor; 
    
    public SpindexerSubsystem() {
        spindexerMotor = new TalonFX(22); 
    }

    public Command spin() {
        return run(
            () -> spindexerMotor.setVoltage(4)
        ).finallyDo(
            () -> spindexerMotor.stopMotor()
        ); 
    }

    public Command spin2() {
        return run(// same as execute()
            () -> spindexerMotor.setVoltage(4)
        )                                               
        // .until( //same as isFinished
        //     () -> timer.hasElapsed(5)
        // )
        .finallyDo( //same as end()
            () -> spindexerMotor.stopMotor()
        ); 
    }
}
