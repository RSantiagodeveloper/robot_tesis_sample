#script de control para la comunicacion serial de raspPI y Arduino
import serial
import time

arduino = serial.Serial('/dev/ttyACM0', baudrate=9600,timeout=1.0) #configuracion primaria, indicando el puerto y el baudio
print("starting")
#arduino.open()
msg = '' #contiene la respuesta de arduino

while True:
	comando = raw_input("command$ ")
	arduino.write(comando) #envia el comando al monitor serial de arduino

	time.sleep(0.25)
	msg = arduino.readline()
	if(msg != ''):
		print float(msg.rstrip('\n'))
		msg = ''

arduino.close() #cierra la comunicacion serial
