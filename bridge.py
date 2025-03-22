#!/usr/bin/env python3
"""
    Bridge Arduino serial data to PlotJuggler over UDP using Protobuf (raw forwarding)
"""
import serial
import socket
import logging
import time
import argparse
import re

class SerialToUDPBridge:
    def __init__(self, serial_port: str, baud_rate: int = 9600, udp_host: str = "127.0.0.1", udp_port: int = 5005, filter_pattern: str = None, verbose: bool = False):
        self.serial_port = serial_port
        self.baud_rate = baud_rate
        self.udp_host = udp_host
        self.udp_port = udp_port
        # self.filter_pattern = re.compile(filter_pattern.encode('utf-8')) if filter_pattern else None
        # self.filter_pattern = re.compile(filter_pattern.encode('utf-8'), re.DOTALL) if filter_pattern else None
        # self.filter_pattern = re.compile(filter_pattern.encode('utf-8'), re.DOTALL) if filter_pattern else None
        # self.filter_pattern = re.compile(rb'(?<=\{\{).*?(?=\}\})', re.DOTALL)
        self.filter_pattern = re.compile(rb'nanopb:\s*\{\{(.*?)\}\}', re.DOTALL) if filter_pattern else None


        self.verbose = verbose
        self.sock = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)

        self._log = logging.getLogger(self.__class__.__name__)

        try:
            self.serial_conn = serial.Serial(self.serial_port, self.baud_rate, timeout=1)
            self._log.info(f"Connected to {self.serial_port} at {self.baud_rate} baud")
        except Exception as e:
            self._log.error(f"Failed to open serial port: {e}")
            exit(1)

        self.byte_count = 0
        self.start_time = time.time()
        self.buffer = b""  # Buffer to store incomplete data

    def start(self):
        """Read from serial and send to UDP"""
        try:
            while True:
                if self.serial_conn.in_waiting > 0:
                    raw_data = self.serial_conn.read(self.serial_conn.in_waiting)  # Read all available data
                    self.byte_count += len(raw_data)
                    self.buffer += raw_data  # Append data to buffer

                    if self.verbose:
                        self._log.info(f"Incoming data: {raw_data}")

                    # Process the buffer
                    self.process_buffer()

                    # Calculate and display data rate
                    elapsed_time = time.time() - self.start_time
                    if elapsed_time > 0:
                        data_rate = self.byte_count / elapsed_time
                        self._log.info(f"Data rate: {data_rate:.2f} bytes/sec")
        except KeyboardInterrupt:
            self._log.info("Stopping bridge...")
            self.close()

    def process_buffer(self):
        """Process the buffer to extract complete protobuf messages"""
        if self.filter_pattern:
            # Find all complete protobuf matches in the buffer
            matches = self.filter_pattern.findall(self.buffer)
            if matches:
                for match in matches:
                    self._log.info(f"Forwarding filtered content: {match}")
                    #  match = match[0]
                    decoded_match = match.decode('utf-8') +"}"
                    self._log.info(f"Decoded match: {decoded_match}")
                    self.sock.sendto(decoded_match.encode('utf-8'), (self.udp_host, self.udp_port))
                # Remove matched data from buffer
                self.buffer = self.filter_pattern.sub(b"", self.buffer)
            else:
                # Handle partial matches or incomplete data
                self._log.debug(f"Current buffer state: {self.buffer}")
        else:
            self._log.warning("No filter pattern specified. Buffer ignored.")

    def close(self):
        self.serial_conn.close()
        self.sock.close()



if __name__ == "__main__":
    parser = argparse.ArgumentParser(description="Bridge Arduino serial data to PlotJuggler over UDP using Protobuf (raw forwarding)")
    parser.add_argument("--serial_port", type=str, required=True, help="Serial port to read from")
    parser.add_argument("--baud_rate", type=int, default=115200, help="Baud rate for the serial port")
    parser.add_argument("--udp_host", type=str, default="127.0.0.1", help="UDP host to send data to")
    parser.add_argument("--udp_port", type=int, default=5005, help="UDP port to send data to")
    parser.add_argument("--filter", type=str, help="Regex pattern to filter forwarded data (e.g., '(?<=\{\{).*?(?=\}\})'  broken right now, just put anything and it will work)")
    parser.add_argument("--verbose", action="store_true", help="Print all incoming data to the terminal")

    args = parser.parse_args()

    logging.basicConfig(level=logging.INFO)
    bridge = SerialToUDPBridge(
        serial_port=args.serial_port,
        baud_rate=args.baud_rate,
        udp_host=args.udp_host,
        udp_port=args.udp_port,
        filter_pattern=args.filter,
        verbose=args.verbose
    )
    bridge.start()
