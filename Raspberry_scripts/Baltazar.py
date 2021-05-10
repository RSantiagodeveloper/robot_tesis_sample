""" Este script tiene como finalidad realizar la misma tarea que se ejecuta con 
Arduino en el archivo Baltazar.ino, pero esta vez controlando las acciones desde
Raspberry """

import serial #biblioteca que controla las comunicaciones a travez del puerto serial
import time #biblioteca para controlar los tiempos de espera en la transmision de datos

arduino = serial.Serial('/dev/ttyACM0', baudrate=9600,timeout=1.0) #configuracion primaria, indicando el puerto y el baudio

#definicion de los comandos que se enviaran al Arduino
DIST_FRONT = "sonar_f"
DIST_BACK = "sonar_b"
DIST_FRONT_R = "IR_R"
DIST_FRONT_l = "IR_L"
GOAL_LOCATION = "findGoal"
ADVANCE = "advmts 0.01"
BACKWARD = "bckmts 0.01"
EV_RIGTH = "turn_R 45"
EV_LEFT = "turn_L 45"
EV_FRONT = "turn_R 90"

TIME_SLEEP = 0.125
STATE_ROBOT = 0
UNKNOW = 100.0

print("starting")

def getLecturas(cadena):
    dato = str(cadena.rstrip('\n').rstrip('\r'))
    if dato != '':    
        return float(dato)
    else:
        return UNKNOW


#arduino.open()
while True:
    #Se Recolecta informacion de los diferentes sensores
    arduino.write(DIST_FRONT)
    time.sleep(TIME_SLEEP)
    debuggerData = getLecturas(arduino.readline())
    arduino.write(DIST_FRONT)
    time.sleep(TIME_SLEEP)
    disfrontal = getLecturas(arduino.readline())
    #distancia trasera
    arduino.write(DIST_BACK)
    time.sleep(TIME_SLEEP)
    distrasera = getLecturas(arduino.readline())
    #distancia a la derecha
    arduino.write(DIST_FRONT_R)
    time.sleep(TIME_SLEEP)
    disderecha = getLecturas(arduino.readline())
    #distancia a la izquierda
    arduino.write(DIST_FRONT_l)
    time.sleep(TIME_SLEEP)
    disIzquierda = getLecturas(arduino.readline())

    #definimos la ubicacion de la meta
    arduino.write(GOAL_LOCATION)
    
    #MAquina de estados para el comportamiento del robot
    if STATE_ROBOT == 0:
        #medimos las distancias medidas por los sensores IR y Ultrasonicos
        print 'BUGCatch: ', debuggerData,' Frontal: ', disfrontal,' Trasera: ', distrasera, ' Derecha: ', disderecha, ' Izquierda: ', disIzquierda
        print('Avanza al frente')
        if disderecha <= 12.5:
            print('STATE 1: Obstacculo a la Derecha')
            arduino.write(GOAL_LOCATION)
            STATE_ROBOT = 1
        elif disIzquierda <= 12.5:
            print('STATE 2: Obstacculo a la Izquierda')
            arduino.write(GOAL_LOCATION)
            STATE_ROBOT = 2
        elif disfrontal <= 12.5:
            print('STATE 3: Obstacculo al frente')
            arduino.write(GOAL_LOCATION)
            STATE_ROBOT = 3
        elif distrasera <= 12.5:
            print('STATE 4: Obstacculo atras')
            arduino.write(GOAL_LOCATION)
            STATE_ROBOT = 4
    elif STATE_ROBOT == 1:
        print('Esquiva por la IZQUIERDA')
        arduino.write(GOAL_LOCATION)
        STATE_ROBOT = 0
    elif STATE_ROBOT == 2:
        print('Esquiva por la DERECHA')
        arduino.write(GOAL_LOCATION)
        STATE_ROBOT = 0
    elif STATE_ROBOT == 3:
        print('Maniobra evasiva al FRENTE')
        arduino.write(GOAL_LOCATION)
        STATE_ROBOT = 0
    elif STATE_ROBOT == 4:
        print('Maniobra evasiva en REVERSA')
        arduino.write(GOAL_LOCATION)
        STATE_ROBOT = 0
    else:
        print('Ninguna Accion')

    #arduino.flushInput(); 
    #repetimos el proceso

arduino.close() #cierra la comunicacion serial