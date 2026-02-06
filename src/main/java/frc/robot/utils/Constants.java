package frc.robot.utils;

public class Constants {

    public class DriverStation {
        public static final int xboxControllerID = 0;
        public static final int leftFlightStickID = 1;
        public static final int rightFlightStickID = 2;
    }

    public class CANBus {
       
        public static final int FRAngle = 4;
        public static final int FRDrive = 3;
        public static final int FLAngle = 2;
        public static final int FLDrive = 1;
        public static final int BRAngle = 8;
        public static final int BRDrive = 7;
        public static final int BLAngle = 6;
        public static final int BLDrive = 5;
        // Intake 11-20
        public static final int LIntake = 11;
        public static final int RIntake = 12;
        // Agitators 21-30
        public static final int LAgitator = 21;
        public static final int RAgitator = 22;
        // Shooters 31-40
        public static final int LShooter = 31;
        public static final int RShooter = 32;
        // Climb 41-50
        public static final int L1Climb = 41;
        public static final int L2Climb = 42;
        public static final int R1Climb = 43;
        public static final int R2Climb = 44;
        public static final int colorSensor = 45;
        public static final int Shooter = 0;
    } 
    public class Manipulator{
         public static final double autoAgitatorSpeed = 0.5;
         public static final double autoIntakeSpeed = 0.5;
         public static final double autoShooterSpeed = 0.5;
         public static final double shooterProportion = 0;
         public static final double shooterIntegral = 0;
        public static final double shooterDerivative = 0;
        
       


        public static final double teleopAgitatorSpeed = 0.7;
        public static final double teleopIntakeSpeed = 0.7;
        public static final double teleopShooterSpeed = 0.7;
    }
}
