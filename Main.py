from gpiozero import Motor 

Config = {
    "MotorPinFoward": 0,
    "MotorPinBackward": 0
}

motor = Motor(forward=17, backward=18)