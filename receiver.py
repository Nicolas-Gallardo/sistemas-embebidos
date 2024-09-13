import serial
from struct import pack, unpack
import time

# Se configura el puerto y el BAUD_Rate
PORT = 'COM3'  # Esto depende del sistema operativo
BAUD_RATE = 115200  # Debe coincidir con la configuracion de la ESP32

ser = None

# Funciones Auxiliares
def start_conn():
    """ Funcion para abrir la conexion a la ESP32 """
    global ser
    ser = serial.Serial(PORT, BAUD_RATE, timeout = 1)
    # Se envia el mensaje de inicio de comunicacion
    message = pack('6s','BEGIN\0'.encode())
    send_message(message)


def send_message(message):
    """ Funcion para enviar un mensaje a la ESP32 """
    ser.write(message)

def receive_response():
    """ Funcion para recibir un mensaje de la ESP32 """
    response = ser.read_until(b'\x00')
    print(f"<receive_response> response: [{response}]")
    return response

def receive_data():
    """ Funcion que recibe tres floats (ff) de la ESP32 
    y los imprime en consola """
    data = receive_response()
    data = unpack("ff", data)
    print(f'Received: {data}')
    return data

def receive_string():
    """ Funcion que recibe char, se espera un OK"""
    data = receive_response()

    print(f'<receive_string> raw: [{data}]')
    # print(f'Type: {type(data)}')
    # print(f'Len: {len(data)}')

    return data

def send_end_message():
    """ Funcion para enviar un mensaje de finalizacion a la ESP32 """
    end_message = pack('4s', 'END\0'.encode())
    ser.write(end_message)

def close_conn():
    """ Funcion para cerrar la conexion a la ESP32 """
    send_end_message()
    ser.close()

#Funciones Principales
def recieve_window_data():
    
    # Se abre la conexion serial
    #start_conn()

    print(ser)

    # Se envia la orden de recibir datos
    message = pack('8s','GETDATA\0'.encode())
    send_message(message)

    inicio = time.time()
    # Se espera el OK
    print("<receive_window_size> Espera OK")
    while True:
        timer = time.time() - inicio
        if timer > 4:
            print("Timer")
            return

        if ser.in_waiting > 0:
            try:
                message = receive_string()
                if b'OK' in message:
                    print("<receive_window_size> OK received")
                    break
            except:
                print('<receive_window_size> Error en leer mensaje1')
                continue
    

    # Se espera mensaje RAW
    print("<waiting_raw>")
    while True:
        if ser.in_waiting > 0:
            try:
                message = receive_string()
                if b'RAW' in message:
                    print("<wait_raw> Raw received")
                    break
            except:
                print('<wait_raw> Error en leer mensaje2')
                continue
    
    presion, temperatura = [], []
    # Se reciben los datos en pares de floats
    print("<waiting_data>")
    while True:
        if ser.in_waiting > 0:
            try:
                message = receive_data()
                
                print("<wait_data> data received")
                message[0].append(temperatura)
                message[1].append(presion) 
            except:
                try:
                    message = receive_string()
                    if b'END' in message:
                        print("<wait_end> END received")
                        break
                except:
                    print('<wait_end> Error en leer mensaje3')
                    continue


    print(f"temperatura: {temperatura}\n presion: {presion}")

    print("<waiting_RMS>")
    # Se espera mensaje RMS
    while True:
        if ser.in_waiting > 0:
            try:
                message = receive_data()
                if b'RMS' in message:
                    print("<wait_rms> RMS received")
                    break
            except:
                print('Error en leer mensaje')
                continue

    # Se reciben los datos de RMS en un par de floats
    while True:
        if ser.in_waiting > 0:
            try:
                message = receive_data()

                if b'FINISH' in message:
                    print("<finish_wait> FINISH received")
                    break

                print(f"<rms> [{message}]")
                rms_temperatura = message[0]
                rms_presion = message[1]
            except:
                print('Error en leer mensaje')
                continue

    return temperatura, presion, rms_temperatura, rms_presion

def restart_ESP():
    # Se abre la conexion serial
    #start_conn()

    # Se envia la orden de reinicio
    message = pack('8s','RESTART\0'.encode())
    send_message(message)

    inicio = time.time()
    # Se espera el OK
    while True:
        timer = time.time() - inicio

        if timer > 4:
            print("Timer")
            break
        if ser.in_waiting > 0:
            try:
                message = receive_string()
            except:
                print('Error en leer mensaje')
                continue
            else:
                if b'OK' in message:
                    break
    
    # Se envia el mensaje de termino de comunicacion
    #close_conn()

    return True

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

    while True:
        if ser.in_waiting <= 0:
            continue
        try:
            message = receive_string()
            if b'SUNLIGHT' in message:
                print("BREAKING SUNLIGHT")
                break
        except:
            print('Error en leer mensaje')
            continue
    
    # print("<set_window_size> Esperando OK")
    # # Se espera el Ok
    
    # Se envia el mensaje de termino de comunicacion
    #close_conn()

    return True

