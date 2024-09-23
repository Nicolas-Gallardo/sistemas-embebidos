import serial
from struct import pack, unpack
import time

# Se configura el puerto y el BAUD_Rate
PORT = 'COM4'  # Esto depende del sistema operativo
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
    print(f"<start_conn> starting connection in {ser}")
    # Se envia el mensaje de inicio de comunicacion
    send_message(pack('8s','TESTING\0'.encode()))
    if (wait_message("OK")): return True
    else: False

#Funciones Principales
def recieve_window_data(window_size):
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
                response = ser.read_until(b'END',8)
                if b'END' in response:
                    print("<recieve_window_data> END received")
                    break
                data = unpack("ff", response)
                temp.append(data[0])
                press.append(data[1])

            except: continue

    print(f"temperatura: {temp}\n presion: {press}")

    print("<recieve_window_data>")
    wait_message("RMS")

    rms_temp = 0
    rms_press = 0
    # Se reciben los datos de RMS en un par de floats
    while True:
        if ser.in_waiting > 0:
            try:
                response = ser.read_until(b'END',8)
                if b'END' in response:
                    print("<recieve_window_data> END received")
                    break
                data = unpack("ff", response)
                rms_temp = data[0]
                rms_press = data[1]
                print(f"<recieve_window_data> rms_temp: {rms_temp}, rms_press: {rms_press}")
                break
            except:
                print('<recieve_window_data> error')
                continue

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

    return True

def restart_ESP():
    # Se envia la orden de reinicio
    message = pack('8s','RESTART\0'.encode())
    send_message(message)
    wait_message("OK")

    return True

def get_window_size():
    # Se envia la orden de recibir datos
    message = pack('8s','GETWIND\0'.encode())
    send_message(message)
    if not wait_message("OK"): return

    print("<get_window_size> waiting window size")
    while True:
        if ser.in_waiting > 0:
            try:
                response = ser.read_until(4)
                window_size = unpack("i", response)
                print(f"<get_window_size> window size = {window_size}")
                return window_size
            except:
                print('<get_window_size> Error en leer mensaje')
                continue