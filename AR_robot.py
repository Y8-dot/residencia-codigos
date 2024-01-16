import time
from machine import Pin, PWM
import random

#Definimos la parte del motor, las acciones que puede realizar
Dir_forward = 1
Dir_backward = -1

LF_forward = 1
LF_backward = -1
LB_forward = 1
LB_backward = -1
RF_forward = 1
RF_backward = -1
RB_forward = 1
RB_backward = -1

#Todas las posibles acciones para el control de los motores en las siguientes funciones
acciones = ["forward", "backward", "turn_left", "turn_right"] #"left", "right", "left_forward", "right_forward", "left_backward", "right_backward",

# Se define la tabla de Q para los posibles valores
Q  = {
    (1, 1, 1): {'forward': 0.7, 'turn_left': 0.1, 'turn_right': 0.2}, # 'left': 0.2, 'right': 0.1, 'backward': 0.1, 'left_forward': 0.2, 'right_forward': 0.1,'left_backward': 0.7, 'right_backward': 0.1,
    (1, 1, 0): {'forward': 0.2, 'turn_left': 0.2, 'turn_right': 0.1}, # 'left': 0.1, 'right': 0.7, 'backward': 0.1, 'left_forward': 0.1, 'right_forward': 0.7,'left_backward': 0.1, 'right_backward': 0.7,
    (1, 0, 0): {'forward': 0.1,  'turn_left': 0, 'turn_right': 0},  
    (1, 0, 1): {'forward': 0.7, 'left': 0, 'right': 0.2, 'backward': 0.1, 'left_forward': 0.3, 'right_forward': 0,
                'left_backward': 0, 'right_backward': 0, 'turn_left': 0, 'turn_right': 0},
    (0, 0, 1): {'forward': 0.2, 'left': 0, 'right': 0.2, 'backward': 0.3, 'left_forward': 0.1, 'right_forward': 0.3,
                'left_backward': 0, 'right_backward': 0.1, 'turn_left': 0.1, 'turn_right': 0.7},
    (0, 1, 1): {'forward': 0.2, 'left': 0, 'right': 0.2, 'backward': 0.1, 'left_forward': 0.1, 'right_forward': 0.2,
                'left_backward': 0, 'right_backward': 0, 'turn_left': 0.1, 'turn_right': 0.7},
    (0, 1, 0): {'forward': 0.2, 'left': 0, 'right': 0.2, 'backward': 0.1, 'left_forward': 0.1, 'right_forward': 0.2,
                'left_backward': 0.1, 'right_backward': 0.1, 'turn_left': 0.1, 'turn_right': 0.2},
    (0, 0, 0): {'forward': 0.5, 'left': 0.1, 'right': 0.2, 'backward': 0.4, 'left_forward': 0.1, 'right_forward': 0.2,
                'left_backward': 0.1, 'right_backward': 0.1, 'turn_left': 0.1, 'turn_right': 0.2},
}

# Parametros SARSA
alpha = 0.5  # tasa de aprendizaje
gamma = 0.6  # factor de descuento
epsilon = 0.1  # exploración vs explotación

# pines correspondientes al modulo de sensores infrarrojos
ir_left = Pin(4, Pin.IN)
ir_middle = Pin(5, Pin.IN)
ir_right = Pin(6, Pin.IN)

# left front wheel M1
Motor_LF_PWM = PWM(Pin(12))
Motor_LF_Dir = Pin(13, Pin.OUT)
# right front wheel M2
Motor_RF_PWM = PWM(Pin(15))
Motor_RF_Dir = Pin(14, Pin.OUT)
# right back wheel M3
Motor_RB_PWM = PWM(Pin(16))
Motor_RB_Dir = Pin(17, Pin.OUT)
# left back wheel M4
Motor_LB_PWM = PWM(Pin(19))
Motor_LB_Dir = Pin(18, Pin.OUT)


def map(x,in_max, in_min, out_max, out_min):
    return (x - in_min)/(in_max - in_min)*(out_max - out_min) + out_min
class Motor():
    def _init_(self):
        pass

    # Control the M1 motor (left front).
    def motor_left_front(self, status, direction, speed):
        if status == 0:  # stop.
            Motor_LF_PWM = PWM(Pin(12))
            Motor_LF_Dir = Pin(13, Pin.OUT)
            Motor_LF_Dir.low()
            Motor_LF_PWM.duty_u16(0)
        else:
            value = int(map(speed, 100, 0, 65535, 0))

            if direction == Dir_forward:

                Motor_LF_PWM = PWM(Pin(12))
                Motor_LF_Dir = Pin(13, Pin.OUT)
                Motor_LF_PWM.freq(500)
                Motor_LF_Dir.low()
                Motor_LF_PWM.duty_u16(value)
            elif direction == Dir_backward:
                Motor_LF_PWM = PWM(Pin(13))
                Motor_LF_Dir = Pin(12, Pin.OUT)
                Motor_LF_Dir.low()
                Motor_LF_PWM.duty_u16(value)

    # Control the M2 motor (right front).
    def motor_right_front(self, status, direction, speed):
        if status == 0:  # stop.
            Motor_RF_PWM = PWM(Pin(15))
            Motor_RF_Dir = Pin(14, Pin.OUT)
            Motor_RF_Dir.low()
            Motor_RF_PWM.duty_u16(0)
        else:
            value = int(map(speed, 100, 0, 65535, 0))

            if direction == Dir_forward:

                Motor_RF_PWM = PWM(Pin(15))
                Motor_RF_Dir = Pin(14, Pin.OUT)
                Motor_RF_PWM.freq(500)
                Motor_RF_Dir.low()
                Motor_RF_PWM.duty_u16(value)
            elif direction == Dir_backward:
                Motor_RF_PWM = PWM(Pin(14))
                Motor_RF_Dir = Pin(15, Pin.OUT)
                Motor_RF_Dir.low()
                Motor_RF_PWM.duty_u16(value)

    # Control the M3 motor (right back).
    def motor_right_back(self, status, direction, speed):

        if status == 0:  # stop.
            Motor_RB_PWM = PWM(Pin(16))
            Motor_RB_Dir = Pin(17, Pin.OUT)
            Motor_RB_Dir.low()
            Motor_RB_PWM.duty_u16(0)
        else:
            value = int(map(speed, 100, 0, 65535, 0))

            if direction == Dir_forward:

                Motor_RB_PWM = PWM(Pin(17))
                Motor_RB_Dir = Pin(16, Pin.OUT)
                Motor_RB_PWM.freq(500)
                Motor_RB_Dir.low()
                Motor_RB_PWM.duty_u16(value)
            elif direction == Dir_backward:
                Motor_RB_PWM = PWM(Pin(16))
                Motor_RB_Dir = Pin(17, Pin.OUT)
                Motor_RB_Dir.low()
                Motor_RB_PWM.duty_u16(value)

    # Control the M4 motor (left back).
    def motor_left_back(self, status, direction, speed):
        if status == 0:  # stop.
            Motor_LB_PWM = PWM(Pin(19))
            Motor_LB_Dir = Pin(18, Pin.OUT)
            Motor_LB_Dir.low()
            Motor_LB_PWM.duty_u16(0)
        else:
            value = int(map(speed, 100, 0, 65535, 0))

            if direction == Dir_forward:

                Motor_LB_PWM = PWM(Pin(18))
                Motor_LB_Dir = Pin(19, Pin.OUT)
                Motor_LB_PWM.freq(500)
                Motor_LB_Dir.low()
                Motor_LB_PWM.duty_u16(value)
            elif direction == Dir_backward:
                Motor_LB_PWM = PWM(Pin(19))
                Motor_LB_Dir = Pin(18, Pin.OUT)
                Motor_LB_Dir.low()
                Motor_LB_PWM.duty_u16(value)

    def motor_stop(self):
        Motor_LF_PWM = PWM(Pin(12))
        Motor_LF_Dir = Pin(13, Pin.OUT)
        Motor_RF_PWM = PWM(Pin(16))
        Motor_RF_Dir = Pin(17, Pin.OUT)
        Motor_RB_PWM = PWM(Pin(19))
        Motor_RB_Dir = Pin(18, Pin.OUT)
        Motor_LB_PWM = PWM(Pin(15))
        Motor_LB_Dir = Pin(14, Pin.OUT)

        Motor_LF_Dir.low()
        Motor_LF_PWM.duty_u16(0)
        Motor_LB_Dir.low()
        Motor_LB_PWM.duty_u16(0)
        Motor_RF_Dir.low()
        Motor_RF_PWM.duty_u16(0)
        Motor_RB_Dir.low()
        Motor_RB_PWM.duty_u16(0)

    def move(self, status, direction, speed):
        if status == 0:
            self.motor_stop()
        else:
            if direction == "forward":
                print('forward')
                self.motor_left_front(1, LF_forward, speed)  # M1
                self.motor_right_front(1, RF_forward, speed)  # M2
                self.motor_right_back(1, RB_forward, speed)  # M3
                self.motor_left_back(1, LB_forward, speed)  # M4
            elif direction == "backward":
                print('backward')
                self.motor_left_front(1, LF_backward, speed)  # M1
                self.motor_right_front(1, RF_backward, speed)  # M2
                self.motor_right_back(1, RB_backward, speed)  # M3
                self.motor_left_back(1, LB_backward, speed)  # M4
            # elif direction == "left":
            #     print('left')
            #     self.motor_left_front(1, LF_backward, speed)  # M1
            #     self.motor_right_front(1, RF_forward, speed)  # M2
            #     self.motor_right_back(1, RB_backward, speed)  # M3
            #     self.motor_left_back(1, LB_forward, speed)  # M4            elif direction == "left":
            #     print('left')
            #     self.motor_left_front(1, LF_forward, speed)  # M1
            #     self.motor_right_front(1, RF_backward, speed)  # M2
            #     self.motor_right_back(1, RB_forward, speed)  # M3
            #     self.motor_left_back(1, LB_backward, speed)  # M4

            # elif direction == "left_forward":
            #     print('left_forward')
            #     self.motor_left_front(0, LF_forward, speed * 0.5)  # M1
            #     self.motor_right_front(1, RF_forward, speed)  # M2
            #     self.motor_right_back(0, RB_forward, speed * 0.5)  # M3
            #     self.motor_left_back(1, LB_forward, speed)  # M4

            # elif direction == "right_forward":
            #     print('right_forward')
            #     self.motor_left_front(1, LF_forward, speed)  # M1
            #     self.motor_right_front(0, RF_forward, speed * 0.5)  # M2
            #     self.motor_right_back(1, RB_forward, speed)  # M3
            #     self.motor_left_back(0, LB_forward, speed * 0.5)  # M4
            # elif direction == "left_backward":
            #     print('left_backward')
            #     self.motor_left_front(1, LF_backward, speed)  # M1
            #     self.motor_right_front(0, RF_backward, speed * 0.5)  # M2
            #     self.motor_right_back(1, RB_backward, speed)  # M3
            #     self.motor_left_back(0, LB_backward, speed * 0.5)  # M4

            # elif direction == "right_backward":
            #     print('right_backward')
            #     self.motor_left_front(0, LF_backward, speed * 0.5)  # M1
            #     self.motor_right_front(1, RF_backward, speed)  # M2
            #     self.motor_right_back(0, RB_backward, speed * 0.5)  # M3
            #     self.motor_left_back(1, LB_backward, speed)  # M4

            elif direction == "turn_left":
                print('turn_left')
                self.motor_left_front(1, LF_backward, speed)  # M1
                self.motor_right_front(1, RF_forward, speed)  # M2
                self.motor_right_back(1, RB_forward, speed)  # M3
                self.motor_left_back(1, LB_backward, speed)  # M4

            elif direction == "turn_right":
                print('turn_right')
                self.motor_left_front(1, LF_forward, speed)  # M1
                self.motor_right_front(1, RF_backward, speed)  # M2
                self.motor_right_back(1, RB_backward, speed)  # M3
                self.motor_left_back(1, LB_forward, speed)  # M4
            else:
                print("Direction error!")

    def motor_left_front11(self, status, direction, speed):

        if status == 0:  # stop.
            Motor_LF_PWM = PWM(Pin(12))
            Motor_LF_Dir = Pin(13, Pin.OUT)
            Motor_LF_Dir.low()
            Motor_LF_PWM.duty_u16(0)
        else:
            value = int(map(speed, 100, 0, 65535, 0))

            if direction == Dir_forward:

                Motor_LF_PWM = PWM(Pin(12))
                Motor_LF_Dir = Pin(13, Pin.OUT)
                Motor_LF_PWM.freq(500)
                Motor_LF_Dir.low()
                Motor_LF_PWM.duty_u16(value)
            elif direction == Dir_backward:
                Motor_LF_PWM = PWM(Pin(13))
                Motor_LF_Dir = Pin(12, Pin.OUT)
                Motor_LF_Dir.low()
                Motor_LF_PWM.duty_u16(value)

    def test(self):  # test M1 motor.
        print("test")

        self.motor_left_front(1, 1, 100)
        time.sleep(2)
        self.motor_left_front(1, -1, 100)
        time.sleep(2)

# obtiene el estado actual del los sensores
def obtener_estado():
    return tuple(get_ir_value())

# funcion para seleccionar una accion segun la politica epsilon-greedy
def select_accion(estado):
    if random.uniform(0, 1) < epsilon:
        return random.choice(acciones)
    else:
        if estado not in Q:
            Q[estado] = {a: 0 for a in acciones}
        return max(Q[estado], key=Q[estado].get)

# funcion SARSA para actualizar la tabla Q
def sarsa(estado, accion, recompensa, nuevo_estado, nueva_accion):
    if estado not in Q:
        Q[estado] = {a: 0 for a in acciones}
    if nuevo_estado not in Q:
        Q[nuevo_estado] = {a: 0 for a in acciones}

    Q[estado][accion] += alpha * (recompensa + gamma * Q[nuevo_estado][nueva_accion] - Q[estado][accion])
def get_ir_value():
    return [ir_left.value(), ir_middle.value(), ir_right.value()]

# funcion donde inicializa la clase del motor
motor = Motor()

# main bucle
try:
    for episodio in range(1000):  # ajustable a las necesidades del usuario
        estado = obtener_estado()
        accion = select_accion(estado)

        # realiza la opcion y obtiene la recompensa
        time.sleep(1)
        motor.move(1, accion, 30)
        time.sleep(0.3)
        nuevo_estado = obtener_estado()
        recompensa = Q.get(estado, {}).get(accion, 0)

        # select the next action
        nueva_accion = select_accion(nuevo_estado)

        # update Q table using SARSA method
        sarsa(estado, accion, recompensa, nuevo_estado, nueva_accion)

except KeyboardInterrupt:
    motor.motor_stop()  # detener el robot en caso de interrupción

print("Tabla Q:")
print(Q)
