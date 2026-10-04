from controller import Robot

robot = Robot()
timestep = int(robot.getBasicTimeStep())

MAX_SPEED = 6.28

leftMotor = robot.getDevice('left wheel motor')
rightMotor = robot.getDevice('right wheel motor')

leftMotor.setPosition(float('inf'))
rightMotor.setPosition(float('inf'))

leftMotor.setVelocity(0.0)
rightMotor.setVelocity(0.0)



while robot.step(timestep) != -1:
    

    phildot = 0.0
    phirdot = 0.0

    phildot = 0.5 * MAX_SPEED
    phirdot = 0.5 * MAX_SPEED

    leftMotor.setVelocity(phildot)
    rightMotor.setVelocity(phirdot)
