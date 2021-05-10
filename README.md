# AVISO
 * Este repositorio contiene una copia (muy simple) de mi proyecto de tesis que aún esta en desarrollo, por lo que aún no decido abrir el código de mi proyecto.
 * El Proyecto de tesis es acerca del desarrollo de un control PID para la navegacion del robot
 * Este trabajo esta siendo desarrollado bajo la dirección del Dr. Jesus Savage Carmona, principal responsable del laboratorio de Biorobótica de la Facultad de ingeniería de la UNAM [Biorobotics](https://biorobotics.fi-p.unam.mx/es/)
 * En este repositorio solamente se encuentran algunos códigos Arduino/c++ de prueba con los que se estaban manipulando sensores y actuadores del robot, al igual que la implementacion (aun sin perfeccionar) del control PID para la velocidad y posicion del robot
 * También se encuntra un script Python que permite la ejecución de acciones en el robot enviadas a traves del protocolo SSH a la computadora del robot, para los ensayos.

# project_robot_thesis
En este repositorio se muestran los principales avances del proyecto de robotica para la tesis
- La carpeta 'Baltazar' muestra el codigo que lleva a cabo la navegacion del robot evadiendo obstaculos y siguiendo la luz
- La carpeta 'Tester Baltazar' muestra el codigo con todas las funciones para probar las medidas de
  los sensores, las funciones para el movimiento, tanto de avance, retroceso rotacion, Hasta este momento en que
  cree el repositorio, ya implemente el Control PID en las funciones de Avance y retroceso
- La carpeta tester Velocidad solo han sido pruebas realizadas para poder implementar adecuadamente el control PID
- La Carpeta Raspberry_scripts contiene los Scrips para la ejecucion de tareas del robot controladas desde la raspberry pi

# Acotaciones de las conexiones de los Sensores y actuadores del robot

Sensores:
    Encoders:
        - M1 => Green
        - M2 => Blue
        - CH-A => Grey
        - CH-B => White
        - VCC => Purple
        - GND => Black
    HC-SRO4 Ultrasonicos
        - Trigger => Yellow
        - Echo => Green
        - VCC => Orange
        - GND => Blue
    SHARP 0A41SK F-86
        - OA = Yellow
        - VCC => Red
        - GND => Black
    LDR:
        - Front => Red
        - Left => Brown
        - Right => Green
        - Back => Blue

conexion LD293
    VSS (Arduino) => Red
    GND => Blue
    VS (Bateria) => Green
    GND => Grey
    
    E0 => Purple
    Dir1 => Grey
    E1 => Orange
    Dir2 => Yellow

Pines GPIO
 - 8 -> Tx => Negro
 - 10 -> Rx => Blanco
# Mediciones

TICKs ENCODERS:
 -1 vuelta => 305 ticks

Vel T_ms  TicksR  TicksL
255 100     77      80
255 10      1       1
255 1       0       0
128 100     40      40
128 10      1       1
128 1       0       0
64  100     18      18
64  10       1      1
64  1        0      0


Velocidades en m/s (suponiendo que cada 10ms ocurre un pulso):
PWM     ~V_L  ~V_R
255    0.044  0.044
128    0.044  0.044
64     0.044  0.044

Velocidades en m/s (segun las observaciones previamente registradas):
PWM     ~V_L  ~V_R
255     0.352 0.352
128     0.176 0.176
64      0.0792 0.0792
