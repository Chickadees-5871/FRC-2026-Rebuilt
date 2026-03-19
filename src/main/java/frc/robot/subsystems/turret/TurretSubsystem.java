package frc.robot.subsystems.turret;

import com.revrobotics.spark.SparkMax;
import com.revrobotics.spark.SparkLowLevel.MotorType;

import edu.wpi.first.wpilibj.motorcontrol.Spark;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.LimelightHelpers;

public class TurretSubsystem extends SubsystemBase{
    private SparkMax frontMotor, backMotor, hoodMotor, rotationMotor;
    
    public TurretSubsystem(){
        frontMotor = new SparkMax(11, MotorType.kBrushless);
        backMotor = new SparkMax(12, MotorType.kBrushless);
        hoodMotor = new SparkMax(13, MotorType.kBrushless);
        rotationMotor = new SparkMax(14, MotorType.kBrushless);
    }

    public void shootABall(){
        double range = LimelightHelpers.getTX("turretLimelight");
        // Calculate the shooting power based on the limelight range
        double power = whatIsTheShootPower(range);
        
        // Calculate hood angle
        adjustHood(range);
        
        frontMotor.set(power);
        backMotor.set(power);
    }



    public double whatIsTheShootPower(double range){
        return 1.0;
    }

    public void adjustHood(double range){

    }
}
