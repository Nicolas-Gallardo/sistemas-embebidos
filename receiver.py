import serial
from struct import pack, unpack

# Se configura el puerto y el BAUD_Rate
PORT = 'COM3'  # Esto depende del sistema operativo
BAUD_RATE = 115200  # Debe coincidir con la configuracion de la ESP32

ser = None

# Funciones Auxiliares
def start_conn():
    """ Funcion para abrir la conexion a la ESP32 """
    ser = serial.Serial(PORT, BAUD_RATE, timeout = 1)

def send_message(message):
    """ Funcion para enviar un mensaje a la ESP32 """
    ser.write(message)

def receive_response():
    """ Funcion para recibir un mensaje de la ESP32 """
    response = ser.readline()
    return response

def receive_data():
    """ Funcion que recibe tres floats (fff) de la ESP32 
    y los imprime en consola """
    data = receive_response()
    data = unpack("fff", data)
    print(f'Received: {data}')
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
def recieve_window_data(window):
    data_list = []
    # Se abre la conexion serial
    start_conn()

    # Se envia el mensaje de inicio de comunicacion
    message = pack('6s','BEGIN\0'.encode())
    send_message(message)

    # Se lee data por la conexion serial
    counter = 0
    while True:
        if ser.in_waiting > 0:
            try:
                message = receive_data()
            except:
                print('Error en leer mensaje')
                continue
            else:
                data_list.append(message)
                counter += 1
                print(counter)
            finally:
                if counter == window+1:
                    print('Lecturas listas!')
                    break
    
    # Se envia el mensaje de termino de comunicacion
    close_conn()

    return data_list

def restart_ESP():
    # Se abre la conexion serial
    start_conn()

    # Se envia el mensaje de reinicio de comunicacion
    message = pack('8s','RESTART\0'.encode())
    send_message(message)

    # Se envia el mensaje de termino de comunicacion
    close_conn()