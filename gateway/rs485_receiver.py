# TODO: Implement this

from handle_data import handle_data
import time

def start_rs485_receiver():
    while True:
        try:
            message = "Hello, World!"
            handle_data(message)
            time.sleep(1)

        except Exception as e:
            print(e)
