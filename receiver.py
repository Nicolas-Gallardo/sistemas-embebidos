import time
from struct import pack, unpack

import serial

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

def wait_message(message, timeout = 5):
    start_time = time.time()
    while(True):
        timer = time.time() - start_time
        if timer > timeout:
            print(f"<wait_message> {message} not received")
            return False
        if ser.in_waiting > 0:
            try:
                response = receive_response()
                if message.encode() in response:
                    print(f"<wait_message> {message} received")
                    return True
            except: continue

def start_conn():
    """Funcion para testear la conexion a la ESP32"""
    global ser
    ser = serial.Serial(PORT, BAUD_RATE, timeout = 1)
    print(f"<start_conn> starting connection in {ser}")
    # Se envia el mensaje de inicio de comunicacion
    send_message(pack('8s','TESTING\0'.encode()))
    return wait_message("OK")


def receive_raw():
    """Funcion para recibir los datos sin procesar de la ESP32"""

    # Se espera mensaje RAW
    print("<receive_raw> wait RAW message")
    wait_message("RAW")

    temp, press, hum, gas = [], [], [], []
    # Se reciben los datos en pares de floats
    print("<receive_raw> waiting data")

    while True:
        if ser.in_waiting > 0:
            try:
                response = ser.read(16)
                print(f"<receive_raw> response = [{response}]")
                if b'END' in response:
                    print("<receive_raw> END received")
                    break
                data = unpack("ffff", response)
                print(f"<receive_raw> data recieved temp = [{data[0]}], press = [{data[1]}], hum = [{data[2]}], gas = [{data[3]}]")
                temp.append(data[0])
                press.append(data[1])
                hum.append(data[2])
                gas.append(data[3])
            except:
                continue

    print(f"temperatura: {temp}\n presion: {press}\n humedad: {hum}\n gas: {gas}")
    return temp, press, hum, gas

def receive_max():
    """Funcion para recibir los valores maximos de la ventana"""
    wait_message("MAX")
    ser
    max_temp, max_press, max_hum, max_gas = [], [], [], []
    # Se reciben los datos en pares de floats
    print("<recieve_window_data> waiting data")

    while True:
        if ser.in_waiting > 0:
            try:
                response = ser.read(16)
                print(f"<recieve_window_data> response = [{response}]")
                if b'END' in response:
                    print("<recieve_window_data> END received")
                    break
                data = unpack("ffff", response)
                print(f"<recieve_window_data> data recieved temp = [{data[0]}], press = [{data[1]}], hum = [{data[2]}], gas = [{data[3]}]")
                max_temp.append(data[0])
                max_press.append(data[1])
                max_hum.append(data[2])
                max_gas.append(data[3])
            except:
                continue
    return max_temp, max_press, max_hum, max_gas

def receive_rms():
    wait_message("RMS")
    rms_temp = 0
    rms_press = 0
    rms_hum = 0
    rms_gas = 0
    # Se reciben los datos de RMS en un par de floats
    while True:
        if ser.in_waiting > 0:
            try:
                response = ser.read(16)
                print(f"<recieve_window_data> response = [{response}]")
                if b'END' in response:
                    print("<recieve_window_data> END received")
                    break

                data = unpack("ffff", response)
                rms_temp = data[0]
                rms_press = data[1]
                rms_hum = data[2]
                rms_gas = data[3]
                print(f"<recieve_window_data> rms_temp: {rms_temp}, rms_press: {rms_press}, rms_hum: {rms_hum}, rms_gas: {rms_gas}")
                break
            except:
                print('<recieve_window_data> error')
                continue
    return rms_temp, rms_press, rms_hum, rms_gas

def receive_fft():
    wait_message("FFT")

    fft_temp, fft_press, fft_hum, fft_gas = [], [], [], []
    # Se reciben los datos en pares de floats
    print("<recieve_window_data> waiting data")
    ser.flush()
    while True:
        if ser.in_waiting > 0:
            try:
                response = ser.read(16)
                print(f"<recieve_window_data> response = [{response}]")
                if b'END' in response:
                    print("<recieve_window_data> END received")
                    break
                data = unpack("ffff", response)
                print(f"<recieve_window_data> data recieved temp = [{data[0]}], press = [{data[1]}], hum = [{data[2]}], gas = [{data[3]}]")
                fft_temp.append(data[0])
                fft_press.append(data[1])
                fft_hum.append(data[2])
                fft_gas.append(data[3])
            except:
                continue

    return fft_temp, fft_press, fft_hum, fft_gas

#Funciones Principales
def request_window_data():
    # Se envia la orden de recibir datos
    message = pack('8s','GETDATA\0'.encode())
    send_message(message)
    if not wait_message("OK"):
        return
    return

def set_window_size(size):
    # Se abre la conexion serial
    print(f"<set_window_size> seteando winSize = [{size}]")
    #start_conn()
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
    if not wait_message("OK"): return -1

    print("<get_window_size> waiting window size")
    ser.flush()
    while True:
        if ser.in_waiting > 0:
            try:
                response = ser.read(4)
                print(f"<get_window_size> response = [{response}]")
                window_size  = int.from_bytes(response, "little")
                print(f"<get_window_size> decoded window_size = [{window_size}]")
                return window_size
            except: continue
