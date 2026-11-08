package frc.robot.subsystems;

import static edu.wpi.first.units.Units.Meters;
import static edu.wpi.first.units.Units.MetersPerSecond;
import static edu.wpi.first.units.Units.Rotations;
import static edu.wpi.first.units.Units.RotationsPerSecond;
import static edu.wpi.first.units.Units.Second;
import static edu.wpi.first.units.Units.Seconds;
import static edu.wpi.first.units.Units.Volts;

import com.ctre.phoenix6.configs.TalonFXConfiguration;
import com.ctre.phoenix6.controls.MotionMagicVelocityVoltage;
import com.ctre.phoenix6.controls.VelocityVoltage;
import com.ctre.phoenix6.hardware.TalonFX;
import com.revrobotics.spark.SparkFlex;
import com.revrobotics.spark.SparkLowLevel.MotorType;

import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import edu.wpi.first.wpilibj2.command.sysid.SysIdRoutine;

public class ShooterSubsystem extends SubsystemBase{
    private final TalonFX rightMotor; 
    
    private double RPS = 4.0; 
    private final VelocityVoltage velocityOutput = new VelocityVoltage(0);
    private final MotionMagicVelocityVoltage mmVelocityOutput = new MotionMagicVelocityVoltage(0); 
    private double rotationsToMeters = 0.5; 

    private final SysIdRoutine sysId; 
    public ShooterSubsystem() {
        rightMotor = new TalonFX(30); 

        SmartDashboard.putNumber("Shooter RPS", RPS); 
        configureMotor();
        // Simple Motor
        // sysId = new SysIdRoutine(
        //     new SysIdRoutine.Config(
        //         Volts.per(Seconds).of(1),
        //         Volts.of(5), 
        //         null,
        //         null
        //     ),
        //     new SysIdRoutine.Mechanism(
        //         (voltage) -> rightMotor.setVoltage(voltage.in(Volts)), 
        //         (log) -> log.motor("Shooter")
        //                 .voltage(Volts.of(rightMotor.getMotorVoltage().getValueAsDouble()))
        //                 .angularPosition(
        //                     Meters.of(
        //                         rightMotor.getPosition().getValueAsDouble()
        //                     )
                            
        //                 ).angularVelocity(
        //                     RotationsPerSecond.of(
        //                         rightMotor.getVelocity().getValueAsDouble()
        //                     )
        //                 ), 
        //         this
                
        //     )

        // ); 

        // Elevator
        sysId = new SysIdRoutine(
            new SysIdRoutine.Config(
                Volts.per(Seconds).of(1),
                Volts.of(5), 
                null,
                null
            ),
            new SysIdRoutine.Mechanism(
                (voltage) -> rightMotor.setVoltage(voltage.in(Volts)), 
                (log) -> log.motor("Shooter")
                        .voltage(Volts.of(rightMotor.getMotorVoltage().getValueAsDouble()))
                        .linearPosition(
                            Meters.of(
                                getHeight()
                            )
                            
                        ).linearVelocity(
                            MetersPerSecond.of(
                                rightMotor.getVelocity().getValueAsDouble()
                            )
                        ), 
                this
                
            )
        );
    }

    public Command sysIdQuasistatic(SysIdRoutine.Direction direction) {
        return sysId.quasistatic(direction); 
    }

    public Command sysIdDynamic(SysIdRoutine.Direction direction) {
        return sysId.dynamic(direction); 
    }

    public double getHeight() {
        return rightMotor.getPosition().getValueAsDouble() * rotationsToMeters; 
    }

    public double getLinearVelocity() {
        return rightMotor.getVelocity().getValueAsDouble() * rotationsToMeters; 
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
