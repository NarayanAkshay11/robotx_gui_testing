import socket
import uuid
import time
from datetime import datetime

# Constants
TEAM_ID = "NTCH"  # 5-character Team ID
MESSAGE_TYPE = "GATE"  # Additional label before Message ID
GROUND_STATION_IP = "192.168.1.100"  # Replace with ground station IP
PORT = 12345  # Define a port for communication
RECONNECT_DELAY = 5  # Time (in seconds) to wait before reconnecting after a failure

def generate_message():
    # Generate unique Message ID with UUID
    message_id = f"$RXGAT-{uuid.uuid4().hex[:8].upper()}"  # Unique for each message, truncated UUID

    # Get current date and time in EST format (assuming system time is in EST)
    est_date = datetime.now().strftime("%d%m%y")
    est_time = datetime.now().strftime("%H%M%S")

    # Define Gate Values (these can be dynamically updated as needed)
    active_entrance_gate = 1  # Entrance Gate (1, 2, or 3)
    active_exit_gate = 2      # Exit Gate (1, 2, or 3)

    # Construct the message without the checksum
    message = f"{MESSAGE_TYPE},{message_id},{est_date},{est_time},{TEAM_ID},{active_entrance_gate},{active_exit_gate}"

    # Calculate checksum (bitwise XOR of ASCII values of all characters)
    checksum = 0
    for char in message:
        checksum ^= ord(char)
    message_with_checksum = f"{message}*{checksum:02X}\r\n"  # Append checksum in hexadecimal format

    return message_with_checksum

def send_message():
    while True:
        try:
            # Attempt to create a TCP socket and connect to the ground station
            with socket.socket(socket.AF_INET, socket.SOCK_STREAM) as s:
                print("Attempting to connect to the ground station...")
                s.connect((GROUND_STATION_IP, PORT))
                print("Connected to ground station.")

                while True:
                    # Generate the heartbeat message
                    message = generate_message()
                    print(f"Sending message: {message.strip()}")

                    # Send the message to the ground station
                    s.sendall(message.encode('utf-8'))

                    # Wait for 1 second before sending the next message (1 Hz frequency)
                    time.sleep(1)

        except (socket.error, socket.timeout) as e:
            print(f"Connection failed: {e}. Retrying in {RECONNECT_DELAY} seconds...")
            time.sleep(RECONNECT_DELAY)  # Wait before attempting to reconnect

if __name__ == "__main__":
    send_message()
