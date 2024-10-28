package frc.robot.subsystems;

//import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
//import edu.wpi.first.wpilibj2.command.WaitCommand;
//import frc.robot.commands.SpeakerShooterCmd;

import com.revrobotics.CANSparkMax;
import com.revrobotics.CANSparkLowLevel.MotorType;
import com.revrobotics.CANSparkBase.IdleMode;
import edu.wpi.first.wpilibj.PneumaticsModuleType;
import edu.wpi.first.wpilibj.Solenoid;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj.Compressor;
import edu.wpi.first.wpilibj.DigitalInput;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;

//import java.util.function.Supplier;

import com.ctre.phoenix.motorcontrol.ControlMode;
import com.ctre.phoenix.motorcontrol.NeutralMode;
import com.ctre.phoenix.motorcontrol.can.VictorSPX;

//import edu.wpi.first.wpilibj2.command.WaitUntilCommand; //e's code

import com.revrobotics.RelativeEncoder;

/* 
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import com.revrobotics.CANSparkMax;
import edu.wpi.first.networktables.NetworkTableInstance;
import frc.robot.Constants;
import frc.robot.RobotContainer;
*/
public class RobotMechSubsystem extends SubsystemBase {
    //create solenoids and compressor
    public final Compressor compressor = new Compressor(PneumaticsModuleType.CTREPCM);
    public final Solenoid AmpSolenoid = new Solenoid(13, PneumaticsModuleType.CTREPCM, 0);



    //public final Solenoid OrangeLight = new Solenoid(13, PneumaticsModuleType.CTREPCM, 7);
   // public final Solenoid ClimbSolenoid = new Solenoid(13, PneumaticsModuleType.CTREPCM, 1);

    Timer timer = new Timer();

    //public final DoubleSolenoid ClimbingLockSolenoid = new DoubleSolenoid(13, PneumaticsModuleType.CTREPCM, 1, 2);

   

    //creates intake and shooting motors
    public CANSparkMax Intake550 = new CANSparkMax(14, MotorType.kBrushless);
    public CANSparkMax bottomShooterMotor = new CANSparkMax(15, MotorType.kBrushless);
    public CANSparkMax topShooterMotor = new CANSparkMax(30, MotorType.kBrushless);
    public VictorSPX ampRoller = new VictorSPX(31);

    //public CANSparkMax ClimbingMotorRight = new CANSparkMax(16, MotorType.kBrushless);
    //public CANSparkMax ClimbingMotorLeft = new CANSparkMax(17, MotorType.kBrushless);

    public VictorSPX rightIntakeDC = new VictorSPX(19);
    public VictorSPX leftIntakeDC = new VictorSPX(18);


    //Climbing relative encoders
    public RelativeEncoder topShooterEncoder = topShooterMotor.getEncoder();
    public RelativeEncoder bottomShooterEncoder = bottomShooterMotor.getEncoder();

    //public RelativeEncoder ClimbLeftEncoder = ClimbingMotorLeft.getEncoder();
    public final DigitalInput digitalInputIntake = new DigitalInput(3);// 
    //public boolean intakeLimitSwitch1 = digitalInputIntake.get();
    public final Solenoid PurpleLightTop = new Solenoid(13, PneumaticsModuleType.CTREPCM, 6);
    public final Solenoid PurpleLightBottom = new Solenoid(13, PneumaticsModuleType.CTREPCM, 3);
    
    
    public double time;
    public double elapsedTime;
    public boolean intakeSwitch;
    public int i;

    public RobotMechSubsystem(){
      Intake550.setInverted(true);
      bottomShooterMotor.setInverted(true);
      topShooterMotor.setInverted(true);
      rightIntakeDC.setInverted(true);
      leftIntakeDC.setInverted(false);
      ampRoller.setInverted(true);
      ampRoller.setNeutralMode(NeutralMode.Coast);
      bottomShooterMotor.setIdleMode(IdleMode.kCoast);
      topShooterMotor.setIdleMode(IdleMode.kCoast);
      Intake550.setIdleMode(IdleMode.kBrake);
    }
    
    public void setIntake(double speed550, double speedDC){
      Intake550.set(speed550);
      rightIntakeDC.set(ControlMode.PercentOutput, speedDC);
      leftIntakeDC.set(ControlMode.PercentOutput, speedDC);
    }

    public void setShooter(double bottomMotorSpeed, double topMotorSpeed){
      bottomShooterMotor.set(bottomMotorSpeed);
      topShooterMotor.set(topMotorSpeed);
    }

    public void setShooterVoltage(double topVoltage, double bottomVoltage){
      bottomShooterMotor.setVoltage(bottomVoltage);
      topShooterMotor.setVoltage(topVoltage);
    }

    public double getTopShooterVoltage(double motorPercent) {
      return topShooterMotor.getBusVoltage() * motorPercent;
    }

    public double getBottomShooterVoltage(double motorPercent) {
      return bottomShooterMotor.getBusVoltage() * motorPercent;
    }

    public double getTopShooterVelocity(){
      return topShooterEncoder.getVelocity();
    }    

    public double getBottomShooterVelocity(){
      return bottomShooterEncoder.getVelocity();
    }    

    //fuction to open solenoid
    public void setAmpSolenoidOn(){
      AmpSolenoid.set(true);
    }
    //function to close solenoid
    public void setAmpSolenoidOff(){
      AmpSolenoid.set(false);
    }

    public void ampRoller(double speedRoller){
      ampRoller.set(ControlMode.PercentOutput, speedRoller);
    }

    public boolean getIntakeSensor(){
      if(digitalInputIntake.get()){
        setOrangeLightsOff();
      }else if(!digitalInputIntake.get()){
        setOrangeLightsOn();
      }
      return digitalInputIntake.get();
    }




    public void countSourceIntake(){
      i++;
    }

    public void setOrangeLightsOn(){
      PurpleLightTop.set(true); 
      PurpleLightBottom.set(true);
    }
    
    public void setOrangeLightsOff(){
      PurpleLightTop.set(false);
      PurpleLightBottom.set(false);
    }
    


    
    @Override
    public void periodic() {
      // This method will be called once per scheduler run
      SmartDashboard.putNumber("Shooter Velocity", topShooterEncoder.getVelocity());
      SmartDashboard.putBoolean("Intake Sensor", digitalInputIntake.get());
      
    }
  }
    

