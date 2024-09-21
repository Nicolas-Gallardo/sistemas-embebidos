import serial
from struct import pack, unpack
import time

# Se configura el puerto y el BAUD_Rate
PORT = 'COM3'  # Esto depende del sistema operativo
BAUD_RATE = 115200  # Debe coincidir con la configuracion de la ESP32

ser = None

window_size = 10
# Funciones Auxiliares

def send_message(message):
    """ Funcion para enviar un mensaje a la ESP32 """
    print(f'<send_message> Message Sent: [{message}]')
    ser.write(message)

def receive_response():
    """ Funcion para recibir un mensaje de la ESP32 """
    response = ser.readline()
    print(f"<receive_response> >ESP32<: [{response}]")
    return response

def receive_data():
    """ Funcion que recibe dos floats (ff) de la ESP32 """
    data = receive_response()
    data = unpack("ff", data)
    print(f'<receive_data> received data: {data}')
    return data

def wait_message(message, timeout = 5):
    start_time = time.time()
    while(True):
        timer = time.time() - start_time
        if timer > timeout:
            print(f"<wait_confirmation> {message} not received")
            return False
        if ser.in_waiting > 0:
            try:
                response = receive_response()
                if message.encode() in response: 
                    print(f"<wait_confirmation> {message} received")
                    return True
            except: continue

def start_conn():
    """ Funcion para abrir la conexion a la ESP32 y recibir el 
    tamaño de la ventana"""
    global ser
    ser = serial.Serial(PORT, BAUD_RATE, timeout = 1)
    # Se envia el mensaje de inicio de comunicacion
    message = pack('6s','BEGIN\0'.encode())
    send_message(message)
    wait_message("OK")
    while True:
        if ser.in_waiting > 0:
            try:
                response = receive_response()
                window_size  = unpack("i", response)
                return window_size
            except: continue

#Funciones Principales
def recieve_window_data():

    # Se envia la orden de recibir datos
    message = pack('8s','GETDATA\0'.encode())
    send_message(message)
    if not wait_message("OK"): return
    # Se espera mensaje RAW
    print("<recieve_window_data> wait raw data")
    wait_message("RAW")
    
    press, temp = [], []
    # Se reciben los datos en pares de floats
    print("<recieve_window_data>")
    while True:
        if ser.in_waiting > 0:
            try:
                response = receive_response()
                if b'END' in response:
                    print("<recieve_window_data> END received")
                    break
                data = unpack("ff", response)
                temp.append(data[0])
                press.append(data[1]) 

            except:
                print('<recieve_window_data> Error en leer mensaje3')
                continue

    print(f"temperatura: {temp}\n presion: {press}")

    print("<recieve_window_data>")
    wait_message("RMS")

    rms_temp = 0
    rms_press = 0
    # Se reciben los datos de RMS en un par de floats
    """
    while True:
        if ser.in_waiting > 0:
            try:
                response = receive_response()
                data = unpack("ff", response)
                rms_temp = data[0]
                rms_press = data[1]
                print(f"<recieve_window_data> rms_temp: {rms_temp}, rms_press: {rms_press}")
                break
            except:
                print('<recieve_window_data> error')
                continue
    """
    return temp, press, rms_temp, rms_press



def set_window_size(size):
    # Se abre la conexion serial
    print(f"<set_window_size> seteando winSize = [{size}]")
    #start_conn()
    window_size = size
    # print("Setwind")Z
    # Se envia la orden de cambiar window_size
    message = pack('8s','SETWIND\0'.encode())
    send_message(message)
    s = str(size) + '\0'
    message = pack(f'{len(s)}s', s.encode())
    print(f"Sending message: [{message}]")
    send_message(message)
    wait_message("SUNLIGHT")

    # print("<set_window_size> Esperando OK")
    # # Se espera el Ok
    
    # Se envia el mensaje de termino de comunicacion
    #close_conn()

    return True

def restart_ESP():
    # Se envia la orden de reinicio
    message = pack('8s','RESTART\0'.encode())
    send_message(message)
    wait_message("OK")

    return True