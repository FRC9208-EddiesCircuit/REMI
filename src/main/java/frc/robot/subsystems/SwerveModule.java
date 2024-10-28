package frc.robot.subsystems;

import com.ctre.phoenix.sensors.WPI_CANCoder;
import com.revrobotics.CANSparkMax;
import com.revrobotics.RelativeEncoder;
import com.revrobotics.CANSparkBase.IdleMode;
import com.revrobotics.CANSparkLowLevel.MotorType;
import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.kinematics.SwerveModulePosition;
import edu.wpi.first.math.kinematics.SwerveModuleState;
import frc.robot.Constants.DriveConstants;
import frc.robot.Constants.ModuleConstants;
//import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;

/* 
Currently unused imports
import edu.wpi.first.wpilibj.AnalogInput;
import edu.wpi.first.wpilibj.RobotController;
*/
public class SwerveModule {

    private final CANSparkMax driveMotor;
    private final CANSparkMax turningMotor;

    private final RelativeEncoder driveEncoder;
    private final RelativeEncoder turningEncoder;

    private final PIDController turningPidController;

    //private final AnalogInput absoluteEncoder;
    private final WPI_CANCoder absoluteEncoder;
    private final boolean absoluteEncoderReversed;
    private final double absoluteEncoderOffsetRad;


    public SwerveModule(int driveMotorId, int turningMotorId, boolean driveMotorReversed, boolean turningMotorReversed,
            int absoluteEncoderId, double absoluteEncoderOffset, boolean absoluteEncoderReversed) {

        this.absoluteEncoderOffsetRad = absoluteEncoderOffset;      // Create the absolute encoder
        this.absoluteEncoderReversed = absoluteEncoderReversed;
        absoluteEncoder = new WPI_CANCoder(absoluteEncoderId);

        driveMotor = new CANSparkMax(driveMotorId, MotorType.kBrushless);   // Create the motors
        turningMotor = new CANSparkMax(turningMotorId, MotorType.kBrushless);

        driveMotor.setInverted(driveMotorReversed);
        turningMotor.setInverted(turningMotorReversed);

        driveEncoder = driveMotor.getEncoder();             //  Create the motor encoders
        turningEncoder = turningMotor.getEncoder();

        driveEncoder.setPositionConversionFactor(ModuleConstants.kDriveEncoderRot2Meter);        // Set the encoder conversion constants to switch from rotations to meters
        driveEncoder.setVelocityConversionFactor(ModuleConstants.kDriveEncoderRPM2MeterPerSec);  // Set the encoder conversion constants to switch from rotations to velocity
        turningEncoder.setPositionConversionFactor(ModuleConstants.kTurningEncoderRot2Rad);      // Set the encoder conversion constants to switch from rotations to Radians
        turningEncoder.setVelocityConversionFactor(ModuleConstants.kTurningEncoderRPM2RadPerSec);// Set the encoder conversion constants to switch from rotations to Radians per second

        turningPidController = new PIDController(ModuleConstants.kPTurning, 0, 0);  // initialize the turning PID controller - Proportional
        turningPidController.enableContinuousInput(-Math.PI, Math.PI);  // lets PID know the encoder is continous and circular
        setIdleModeBrake();
        resetEncoders();
    }

    public double getDrivePosition() {
        return driveEncoder.getPosition();
    }


    public SwerveModulePosition getPosition() {
        return new SwerveModulePosition(driveEncoder.getPosition(), new Rotation2d(turningEncoder.getPosition()));
      }
    

    public double getTurningPosition() {
        double angle = turningEncoder.getPosition() % (2*Math.PI);
        if(angle> Math.PI){
            angle = -((2*Math.PI) - angle);
          }
        return angle;
    }

    public double getDriveVelocity() {
        return driveEncoder.getVelocity();
    }

    public double getTurningVelocity() {
        return turningEncoder.getVelocity();
    }

    public double getAbsoluteEncoderRad() {

        double angle;
        double AbsEncToRad = absoluteEncoder.getAbsolutePosition() * (Math.PI /180);
        //return AbsEncToRad;
        if(absoluteEncoderOffsetRad <= AbsEncToRad){
            angle = AbsEncToRad - absoluteEncoderOffsetRad;
        }else{
            angle = AbsEncToRad + (2 *(Math.PI) - absoluteEncoderOffsetRad);
        }        

        if(angle> Math.PI){
            angle = -((2*Math.PI) - angle);
          }
        return angle * (absoluteEncoderReversed ? -1.0 : 1.0);
    }
    
    public void resetEncoders() {
        driveEncoder.setPosition(0);
        turningEncoder.setPosition(getAbsoluteEncoderRad());
    }

    public SwerveModuleState getState() {
        return new SwerveModuleState(getDriveVelocity(), new Rotation2d(getTurningPosition()));
    }

    public void setDesiredState(SwerveModuleState state) {
        if (Math.abs(state.speedMetersPerSecond) < 0.001) {
            stop();
            return;
        }
        state = SwerveModuleState.optimize(state, getState().angle);
        driveMotor.set(state.speedMetersPerSecond / DriveConstants.kPhysicalMaxSpeedMetersPerSecond);
        turningMotor.set(turningPidController.calculate(getTurningPosition(), state.angle.getRadians()));
   
        //SmartDashboard.putNumber("Swerve[" + absoluteEncoder.getDeviceID() + "] position", absoluteEncoder.getAbsolutePosition());
        //SmartDashboard.putNumber("Swerve[" + driveEncoder + "] state", driveEncoder.getPosition());
        //SmartDashboard.putNumber("Swerve[" + turningEncoder + "] state", turningEncoder.getPosition());
        //SmartDashboard.putNumber("Swerve Absolute[" + absoluteEncoder.getDeviceID() + "] state", getAbsoluteEncoderRad());
        //SmartDashboard.putNumber("Swerve turn[" + turningEncoder + "] state", getTurningPosition());

    }


    public void stop() {
        driveMotor.set(0);
        turningMotor.set(0);
    }

    public void setIdleModeBrake(){
        driveMotor.setIdleMode(IdleMode.kBrake);
    }
    public void setIdleModeCoast(){
        driveMotor.setIdleMode(IdleMode.kCoast);
    }
}
