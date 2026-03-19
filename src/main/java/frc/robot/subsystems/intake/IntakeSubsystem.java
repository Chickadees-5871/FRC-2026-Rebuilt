package frc.robot.subsystems.intake;

import com.revrobotics.spark.SparkMax;
import com.revrobotics.spark.SparkLowLevel.MotorType;

import edu.wpi.first.wpilibj2.command.SubsystemBase;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.*;

public class IntakeSubsystem extends SubsystemBase{
    private SparkMax intakeMotorOne, intakeMotorTwo;

    private int reverse = 1;
    
    public IntakeSubsystem(){
        intakeMotorOne = new SparkMax(9, MotorType.kBrushless);
        intakeMotorTwo = new SparkMax(10, MotorType.kBrushless);

    }

    public Command startIntakeCommand(){
        return new InstantCommand(() -> startIntake());
    }

    public Command stopIntakeCommand(){
        return new InstantCommand(() -> stopIntake());
    }

    public Command toggleReverseCommand(){
        return new InstantCommand(() -> toggleReverse());
    }

    public void startIntake(){
        this.intakeMotorOne.set(0.8 * reverse);
        this.intakeMotorTwo.set(0.8 * reverse);
    }

    public void stopIntake(){
        this.intakeMotorOne.set(0);
        this.intakeMotorTwo.set(0);
    }

    public void toggleReverse(){
        reverse *= -1;
    }


}
