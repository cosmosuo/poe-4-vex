# ---------------------------------------------------------------------------- #
#                                                                              #
# 	Module:       main.py                                                      #
# 	Author:       Ivy Liu; Maggie Chen                                         #
# 	Created:      5/6/2026, 1:20:07 PM                                         #
# 	Description:  V5 project                                                   #
#                                                                              #
# ---------------------------------------------------------------------------- #

# Library imports
from vex import *

# Brain should be defined by default
brain = Brain()

#------------------------------------- Robot Configuration --------------------------------------
rightMotor = Motor(Ports.PORT1, GearSetting.RATIO_18_1, False)  # Right drivetrain motor
leftMotor = Motor(Ports.PORT2, GearSetting.RATIO_18_1, True)    # Left drivetrain motor
liftMotor = Motor(Ports.PORT3, GearSetting.RATIO_18_1, False)   # Lift drivetrain motor
inertial_1 = Inertial(Ports.PORT5)                              # Inertial sensor
liftArmRotation = Rotation(Ports.PORT6, False)                  # Liftarm rotation sensor
bumpSwitch = Bumper(brain.three_wire_port.a)                    # Bumper switch
#------------------------------------------------------------------------------------------------

#-------------------------------------- Helper Functions ----------------------------------------
def bump():
    """
    Hold the program's execution until the button is pressed.
    """

    while(bumpSwitch.pressing() == False):
        wait(10, MSEC)                # Debounce the button 

        brain.screen.set_cursor(1, 1) # Place cursor in row 1, col. 1
        brain.screen.print("Press the button to start the program")
        pass
    brain.screen.clear_line(1)
    brain.screen.set_cursor(1,1)
    brain.screen.print("Program executed") 
    wait(1, SECONDS)

def inertialCalibration():
    """
    1. Calibrate the inertial sensor.
    2. Include a 2 second wait time for calibration.
    3. Call this function at the start of the program's execution.
    """

    brain.screen.clear_screen()
    brain.screen.set_cursor(1, 1)
    brain.screen.print("Calibrating the inertial sensor")
    brain.screen.set_cursor(2, 1)
    brain.screen.print("Don't move the robot!")
    inertial_1.calibrate() # Calibrate the inertial sensor 

    wait(2, SECONDS)       # Time required to calibrate the inertial sensor

    brain.screen.set_cursor(1, 1)
    brain.screen.clear_line(1)
    brain.screen.print("Inertial calibration complete")

def testInertial():
    """
    1. Test the inertial sensor by having it display heading and rotation data
    2. Press the button to end the test
    """
    
    brain.screen.clear_screen()
    while(bumpSwitch.pressing() == False):
        wait(10, MSEC) # Debounce the button
        brain.screen.set_cursor(5, 1)
        brain.screen.print("Heading: " + str(inertial_1.heading()))
        brain.screen.set_cursor(6, 1)
        brain.screen.print("Rotation: " + str(inertial_1.rotation()))
        brain.screen.set_cursor(8, 1)
        brain.screen.print("Press the button to end the test.")

def driveStraightData(e):
    """
    1. Report position, rotation, and error
    2. Parameter: e = error (setpoint - rotation)
    """

    brain.screen.set_cursor(1, 1)
    brain.screen.print("Position: " + str(leftMotor.position()))  # Return the current encoder count
    
    brain.screen.set_cursor(2, 1)
    brain.screen.print("Rotation: " + str(inertial_1.rotation())) # Return the current rotation
    
    brain.screen.set_cursor(3, 1)
    brain.screen.print("Error: " + str(e))                        # Return the current error

def stopMotors():
    """
    Stop both motors at the same time
    """

    rightMotor.stop()
    leftMotor.stop()
    wait(0.5, SECONDS) # Wait 0.5 seconds for the system to stabilize

def driveStraight(distance, setpoint, motorVelocity):
    """
    1. distance = distance in inches
    2. setpoint = 0-degrees for driving straight
    3. motorVelcotiy = nominal motor velocity (+) => Forward, (-) => Reverse
    """
    
    inertial_1.reset_rotation() # Reset the rotation value before taking action

    # Set stopping mode for the motors
    leftMotor.set_stopping(COAST)
    rightMotor.set_stopping(COAST)

    kP = 0.705 # Proportional constant for driving straight
               # Used to calculate the correction to maintain course
               # If too small, correction will occur too slowly
               # If too large, over-correction will occur
               # Determine the best value by iteratively testing

    wheelDiameter = 4                               # 4" Wheel Diameter
    wheelCircumference = wheelDiameter * math.pi    # Wheel circumference

    # Convert the distance in inches to distance in "ticks"
    # distance (ticks) = (distance in inches / wheel circumference) * 360
    distance = (distance / wheelCircumference) * 360

    # Reset the motor encoders
    leftMotor.set_position(0, DEGREES)
    rightMotor.set_position(0, DEGREES)

    # Drive forward if motor velocity > 0
    if(motorVelocity > 0):
        # While loop to track distance traveled.
        while(leftMotor.position() < distance):
            error = (setpoint - inertial_1.rotation()) # Error
            correction = kP * error                    # Motor velocity correction

            # Correct motor velocities
            # If error > 0 (setpoint > rotation) => drifting left
            # If error < 0 (setpoint < rotation) => drifting right

            leftMotor.set_velocity((motorVelocity + correction), PERCENT)
            rightMotor.set_velocity((motorVelocity - correction), PERCENT)

            # Spin the motors
            leftMotor.spin(FORWARD)
            rightMotor.spin(FORWARD)

            driveStraightData(error) # Display position, rotation, and error
        
        stopMotors()                 # Stop both motors when the desired distance is reached

    else:
        # While loop to track distance traveled.
        distance *= -1 # distance = distance * -1
        while(leftMotor.position() > distance):
            error = (setpoint - inertial_1.rotation()) # Error
            correction = kP * error                    # Motor velocity correction

            # Correct motor velocities
            # If error > 0 (setpoint > rotation) => drifting left
            # If error < 0 (setpoint < rotation) => drifting right

            leftMotor.set_velocity((motorVelocity + correction), PERCENT)
            rightMotor.set_velocity((motorVelocity - correction), PERCENT)

            # Spin the motors
            leftMotor.spin(FORWARD)
            rightMotor.spin(FORWARD)

            driveStraightData(error) # Display position, rotation, and error
        
        stopMotors()                 # Stop both motors when the desired distance is reached

def turnData(turnError, derivative):
    brain.screen.set_cursor(1, 1)
    brain.screen.print("Heading: " + str(intertial_1.heading()))  # Return the heading
    
    brain.screen.set_cursor(2, 1)
    brain.screen.print("Error: " + str(abs(turnError)))           # Return the error
    
    brain.screen.set_cursor(3, 1)
    brain.screen.print("Derivative: " + str(abs(derivative)))     # Return the derivative

def pointTurn(setPoint):
    """
    1. Perform a point turn using the inertial sensor heading and proportional & derivative control
    2. Argument: Desired heading (setpoint) in degrees
    """

    brain.screen.clear_screen() # Clear the screen
    
    # Set stopping mode for the left and right motors 
    leftMotor.set_stopping(BRAKE)
    rightMotor.set_stopping(BRAKE)

    # Calculate the difference between the setPoint and the current heading
    # to determine the turning direction
    difference = setPoint - inertial_1.heading() 

    # Want to turn the smallest amount to reach the desired heading (not the reflex angle)
    if(setPoint> inertial_1.heading()):
        if(abs(difference) <= 180):
            clockwise = True  # Turn CW
        else:
            clockwise = False # Turn CCW
    else:
        if(abs(difference) <= 180):
            clockwise = False # Turn CCW
        else:
            clockwise = True  # Turn CW

    # Define kP and kD for CW and CCW turns
    if(clockwise): # Values for a CW turn
        kP = 0.04
        kD = 0.00
    else:          # Values for a CCW turn
        kP = 0.04
        kD = 0.00

    # Define maximum turning velocity and previous error term
    maxVelocity = 50    # Maximum turning velocity
    previousError = 0.0 # Error from the previous loop iteration

    while(True):
        turnError = setPoint - inertial_1.heading() # Calculate error
        derivative = turnError - previousError      # Current error - previous error

        # Break out of the loop and stop turning when the setPoint is reached 
        # without oscillation
        if((abs(turnError) < 1 ) and (abs(derivative) < 0.2)):
            stopMotors() # Stop motors
            break        # Exit the while loop

        # Calculate the correction for the motor velocities
        turnCorrection = (kP * turnError) + (kD * derivative)

        # Limit the turnCorrection to be between -1 and 1.
        # This will keep the motor velocity <= maximum turn velocity
        if(abs(turnCorrection) > 1):
            turnCorrection = 1
        
        turnVelocity = maxVelocity * turnCorrection

        # Set the motor velocities based on the direction (CW or CCW)
        if(clockwise): # Turn clockwise
            leftMotor.set_velocity(turnVelocity)
            rightMotor.set_velocity(-1 * turnVelocity)
        else:          # Turn counterclockwise
            leftMotor.set_velocity(-1 * turnVelocity)
            rightMotor.set_velocity(turnVelocity)

        # Spin the motors
        leftMotor.spin(FORWARD)
        rightMotor.spin(FORWARD)

        turnData(turnError, derivative) # Print heading, error & derivatative data
        
        previousError = turnError       # Update previous error term
        wait(20, MSEC) 
        

#------------------------------------------------------------------------------------------------

#------------------------------------- Define main() function -----------------------------------
def main():
    """
    The main() function is the program that is execute by the Brain
    """

    bump() # Call the bump() function to begin program execution

    # Set stopping mode for left nad right motors
    leftMotor.set_stopping(BRAKE) # This mode will help reduce 'lurch'.
    rightMotor.set_stopping(BRAKE)
    inertialCalibration()         # Calibrate the inertial sensor
    
    #driveStraight(87, 0, 50)      # Call driveStraight with the necessary parameters
    #wait(4, SECONDS) 
    #driveStraight(87, 0, -50)

    pointTurn(224)
    wait(2, SECONDS)
    #pointTurn(37)
    #wait(2, SECONDS)
    #pointTurn(135)
    #wait(2, SECONDS)
    
#------------------------------------------------------------------------------------------------

#------------------------------------------------------------------------------------------------
main()

#------------------------------------------------------------------------------------------------



        
