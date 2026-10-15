package frc.robot.subsystems;

import com.revrobotics.spark.SparkFlex;
import com.revrobotics.spark.SparkLowLevel.MotorType;

import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

public class ShooterSubsystem extends SubsystemBase{
    private final SparkFlex rightMotor; 
    
    private double volts = 4.0; 
    public ShooterSubsystem() {
        rightMotor = new SparkFlex(30, MotorType.kBrushless); 

        SmartDashboard.putNumber("Shooter Volts", volts); 
    }

    public Command runShooting() {
        return run(
            () -> {
                rightMotor.setVoltage(volts);
                //leftMotor.setVoltage(3); 
            }
        ).finallyDo(
            () -> rightMotor.stopMotor()
        ); 
    }

    @Override
    public void periodic() {
        volts = SmartDashboard.getNumber("Shooter Volts", 0); 
    }
}
