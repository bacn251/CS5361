#!/usr/bin/env python3
import serial
import serial.tools.list_ports
import argparse
import time
import os
from datetime import datetime

class RawDataCapture:
    def __init__(self, port=None, baud_rate=115200, duration=None, output_file=None):
        """
        Initialize the raw data capture
        
        Args:
            port (str): COM port for the STM32 device
            baud_rate (int): Baud rate for serial communication
            duration (float): Recording duration in seconds (None for infinite)
            output_file (str): Output raw file path
        """
        self.port = port
        self.baud_rate = baud_rate
        self.duration = duration
        
        # Generate default filename if not provided
        if output_file is None:
            timestamp = datetime.now().strftime("%Y%m%d_%H%M%S")
            self.output_file = f"stm32_raw_data_{timestamp}.raw"
            self.txt_file = f"stm32_raw_data_{timestamp}.txt"
        else:
            self.output_file = output_file
            # Create txt filename by replacing or adding .txt extension
            base_name = os.path.splitext(output_file)[0]
            self.txt_file = base_name + ".txt"
        
        self.serial_port = None
        self.running = False
        self.bytes_recorded = 0
        
        # Auto-detect port if not specified
        if self.port is None:
            self.port = self.find_stm32_port()
            if self.port:
                print(f"STM32 device found on port: {self.port}")
            else:
                print("No STM32 device found. Please specify port manually.")
    
    def find_stm32_port(self):
        """Find the COM port for STM32 device"""
        ports = serial.tools.list_ports.comports()
        for port in ports:
            # Check for STM32 Virtual COM Port or similar description
            if "STM32" in port.description or "Virtual" in port.description:
                return port.device
        return None
    
    def connect(self):
        """Connect to the STM32 device"""
        try:
            self.serial_port = serial.Serial(self.port, self.baud_rate, timeout=1)
            print(f"Connected to {self.port} at {self.baud_rate} baud")
            return True
        except serial.SerialException as e:
            print(f"Error connecting to serial port: {e}")
            return False
    
    def start_capture(self):
        """Start capturing raw data to file"""
        if not self.connect():
            return False
        
        try:
            # Open both raw and text files
            with open(self.output_file, 'wb') as raw_file, \
                 open(self.txt_file, 'w') as txt_file:
                
                print(f"Capturing raw data to {self.output_file}")
                print(f"Saving readable data to {self.txt_file}")
                
                if self.duration:
                    print(f"Capture will stop after {self.duration} seconds")
                else:
                    print("Capturing until stopped (Ctrl+C to stop)")
                
                # Write header to text file
                txt_file.write("Value\n")
                
                # Flush any existing data
                self.serial_port.reset_input_buffer()
                
                self.running = True
                self.bytes_recorded = 0
                start_time = time.time()
                last_status_time = start_time
                
                while self.running:
                    # Đảm bảo đủ 4 byte header
                    header = self.serial_port.read(4)
                    if len(header) < 4:
                        continue

                    if header[0] == 0xAA and header[1] == 0x55:
                        count = (header[2] << 8) | header[3]
                        data_len = count * 3

                        # Đọc phần data
                        data = self.serial_port.read(data_len)
                        if len(data) == data_len:
                            # Write to raw file
                            raw_file.write(data)
                            self.bytes_recorded += len(data)
                            
                            # Write to text file in readable format
                            current_time = time.time() - start_time
                            for i in range(0, len(data), 3):
                                # First byte is lowest, which means little-endian
                                value = int.from_bytes(data[i:i+3], byteorder='little')
                                txt_file.write(f"0x{value:06x}\n")
                            
                            # Flush text file periodically
                            if current_time - last_status_time >= 1.0:
                                txt_file.flush()
                    else:
                        # Không đúng header -> bỏ byte đầu tiên, dịch xuống 1 byte để sync lại
                        self.serial_port.read(1)

                    # Status
                    current_time = time.time()
                    if current_time - last_status_time >= 1.0:
                        elapsed = current_time - start_time
                        rate = self.bytes_recorded / elapsed if elapsed > 0 else 0
                        print(f"\rCapturing: {elapsed:.1f}s - {self.bytes_recorded} bytes ({rate:.1f} bytes/s)", end="")
                        last_status_time = current_time

                    # Check duration if specified
                    if self.duration and (current_time - start_time) >= self.duration:
                        self.running = False
                        print("\nCapture duration reached")
                
                return True
                
        except KeyboardInterrupt:
            print("\nCapture stopped by user")
        except Exception as e:
            print(f"\nError during capture: {e}")
        finally:
            self.stop_capture()
            return True
    
    def stop_capture(self):
        """Stop capturing and clean up"""
        self.running = False
        
        if self.serial_port and self.serial_port.is_open:
            self.serial_port.close()
            print("\nSerial port closed")
        
        print(f"Saved {self.bytes_recorded} bytes of raw data to {self.output_file}")
        print(f"Saved readable data to {self.txt_file}")

def main():
    parser = argparse.ArgumentParser(description='Capture raw data from STM32 to binary file')
    parser.add_argument('-p', '--port', help='COM port for STM32 device')
    parser.add_argument('-b', '--baud', type=int, default=115200, help='Baud rate')
    parser.add_argument('-d', '--duration', type=float, help='Capture duration in seconds')
    parser.add_argument('-o', '--output', help='Output raw file path')
    
    args = parser.parse_args()
    
    capture = RawDataCapture(
        port=args.port,
        baud_rate=args.baud,
        duration=args.duration,
        output_file=args.output
    )
    
    capture.start_capture()

if __name__ == "__main__":
    main() 