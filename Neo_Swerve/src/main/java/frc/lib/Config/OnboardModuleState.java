package frc.lib.Config;

import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.kinematics.SwerveModuleState;

public class OnboardModuleState {
   /**
   * Minimize the change in heading the desired swerve module state would require by potentially
   * reversing the direction the wheel spins. Customized from WPILib's version to include placing in
   * appropriate scope for CTRE and REV onboard control as both controllers as of writing don't have
   * support for continuous input.
   *
   * @param desiredState The desired state.
   * @param currentAngle The current module angle.
   */
  public static SwerveModuleState optimize(SwerveModuleState desiredState, Rotation2d currentAngle) {
    double targetAngle = placeInAppropriate0To360Scope(currentAngle.getDegrees(), desiredState.angle.getDegrees());
    double targetSpeed = desiredState.speedMetersPerSecond;
    double delta = targetAngle - currentAngle.getDegrees(); // this finds how far module needs to travel

    //if needs to travel more than +/- 90 degrees, we can optimize
    if (Math.abs(delta) > 90) {
        targetSpeed = -targetSpeed; //makes it reverse
        targetAngle = delta > 90 ? (targetAngle -= 180) : (targetAngle +=180); // x -= y is x = x - y an x += y is x = x + y
      //set target angle if the delta is +90 by subracting 180, if the delta is -90 add 180 to target angle
      //reversing the wheel meens the turn modules travel less
    }
    return new SwerveModuleState(targetSpeed, Rotation2d.fromDegrees(targetAngle));
  }

   /**
   * @param scopeReference Current Angle
   * @param newAngle Target Angle
   * @return Closest angle within scope
   */
  private static double placeInAppropriate0To360Scope(double scopeReference, double newAngle) {
    double lowerBound;
    double upperBound;
    double lowerOffset = scopeReference % 360; //what is the degrees in 0 to 360
    //modulo (%) finds the remainder. for example, 450%360 is 90, 450˚ is the same as 90˚

    //modulos are funky and output differently between coding languages. 
    //the tldr of this https://en.wikipedia.org/wiki/Modulo wikipedia page is that i think loweroffset
    //will be negative when scopeReference is negative. 
    //this varies by coding languages though and is dumb and annoying
    if (lowerOffset >= 0) {
        //lowerBound is now where the module is minus the offset. This creates the nearest "bound" of 360 degree ranges
        lowerBound = scopeReference - lowerOffset;
        upperBound = scopeReference + (360 - lowerOffset);
         //upperBound is 360 higher than lowerBound
    } else {
        //if the angle is currently negative, we have to reverse the signs! yay!
        //this is very painful to wrap my head around
        upperBound = scopeReference - lowerOffset;
        lowerBound = scopeReference - (360 + lowerOffset);
    }

    //makes sure newAngle calculated is between -360 and 360
    while (newAngle < lowerBound) {
        //keep adding 360 until in bounds
        newAngle += 360;
    }
    while (newAngle > upperBound) {
        //keep subtracting 360 until in bounds
        newAngle -= 360;
    }

    //if its greater than 180, we can subtract or add 180 to make it a lower angle to turn to
    if (newAngle - scopeReference > 180) {
        newAngle -= 360;
     } else if (newAngle - scopeReference < -180) {
        newAngle += 360;
     }
     return newAngle;
  }
}
