import os
from LoRaRF import SX127x, LoRaSpi, LoRaGpio
from handle_data import handle_data

# Configuration parameters
LORA_FREQUENCY = 434.0e6       # 434 MHz in Hz
LORA_BANDWIDTH = 125000        # 125 kHz
LORA_SPREADING_FACTOR = 9      # SF9
LORA_CODING_RATE = 7           # 4/7 coding rate
LORA_SYNC_WORD = 0x12          # sync word 18 decimal => 0x12 hex
LORA_PREAMBLE_LENGTH = 8
LORA_GAIN = 0                  # We'll use automatic gain if possible

def start_lora_receiver():
    # Initialize SPI and GPIO for LoRa module
    spi = LoRaSpi(busId=0, csId=0)      # SPI0 with CS0
    cs = LoRaGpio(0, 8)                 # GPIO 8 for chip select
    reset = LoRaGpio(0, 24)             # GPIO 24 for reset
    LoRa = SX127x(spi, cs, reset)

    print("Initializing LoRa radio...")
    if not LoRa.begin():
        raise Exception("Cannot initialize LoRa module.")

    # Set frequency
    LoRa.setFrequency(int(LORA_FREQUENCY))  # frequency in Hz

    # Set RX gain - using power saving and AGC on
    LoRa.setRxGain(LoRa.RX_GAIN_POWER_SAVING, LoRa.RX_GAIN_AUTO)

    # Set modulation parameters
    LoRa.setSpreadingFactor(LORA_SPREADING_FACTOR)   # e.g. SF9
    LoRa.setBandwidth(LORA_BANDWIDTH)                # 125 kHz
    LoRa.setCodeRate(LORA_CODING_RATE)               # 4/7

    # Set packet parameters
    LoRa.setHeaderType(LoRa.HEADER_EXPLICIT)
    LoRa.setPreambleLength(LORA_PREAMBLE_LENGTH)
    LoRa.setCrcEnable(True)

    # Set sync word
    LoRa.setSyncWord(LORA_SYNC_WORD)

    print("LoRa Receiver configured. Waiting for incoming packets...\n")

    # Continuously listen for packets
    while True:
        try:
            # Request a packet
            LoRa.request()
            LoRa.wait()

            # Read received packet
            message = ""
            while LoRa.available() > 0:
                message += chr(LoRa.read())

            handle_data(message)

        except Exception as e:
            print(e)

