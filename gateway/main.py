# check repo: https://github.com/chandrawi/LoRaRF-Python

import threading
from lora_receiver import start_lora_receiver
from rs485_receiver import start_rs485_receiver

if __name__ == "__main__":
    print("Starting services...")
    
    try:
        # Create and start threads for each service
        lora_thread = threading.Thread(target=start_lora_receiver, daemon=True)
        rs485_thread = threading.Thread(target=start_rs485_receiver, daemon=True)

        lora_thread.start()
        rs485_thread.start()

        # Keep the main thread alive while services run
        while True:
            pass

    except KeyboardInterrupt:
        print("Services stopped.")