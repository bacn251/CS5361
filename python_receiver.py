#!/usr/bin/env python3
import serial
import numpy as np
import matplotlib.pyplot as plt
from matplotlib.animation import FuncAnimation
import struct
import time
import argparse

class AudioReceiver:
    def __init__(self, port, baudrate=115200, buffer_size=8192):
        self.ser = serial.Serial(port, baudrate, timeout=1)
        self.buffer = bytearray()
        self.buffer_size = buffer_size
        self.samples = np.zeros(buffer_size, dtype=np.float32)
        self.sample_index = 0
        self.PACKET_START_MARKER1 = 0xAA
        self.PACKET_START_MARKER2 = 0x55
        
    def read_data(self):
        """Đọc dữ liệu từ cổng serial và tìm gói tin"""
        if self.ser.in_waiting:
            data = self.ser.read(self.ser.in_waiting)
            self.buffer.extend(data)
            
        # Xử lý các gói tin trong buffer
        while len(self.buffer) >= 8:  # Kiểm tra đủ header
            # Tìm đánh dấu bắt đầu gói
            if self.buffer[0] != self.PACKET_START_MARKER1 or self.buffer[1] != self.PACKET_START_MARKER2:
                self.buffer.pop(0)  # Loại bỏ byte đầu tiên nếu không phải đánh dấu
                continue
                
            # Đọc header
            samples_count = (self.buffer[2] << 8) | self.buffer[3]
            version = self.buffer[4]
            bytes_per_sample = self.buffer[5]
            
            # Kiểm tra đủ dữ liệu cho toàn bộ gói
            packet_size = 8 + samples_count * bytes_per_sample
            if len(self.buffer) < packet_size:
                break  # Chưa đủ dữ liệu cho gói tin này
                
            # Xử lý dữ liệu 24-bit
            for i in range(samples_count):
                offset = 8 + i * 3
                # Ghép 3 byte thành số 24-bit
                value = (self.buffer[offset+2] << 16) | (self.buffer[offset+1] << 8) | self.buffer[offset]
                
                # Xử lý số âm (two's complement)
                if value & 0x800000:
                    value = value - 0x1000000
                    
                # Chuẩn hóa về khoảng [-1, 1]
                normalized = value / 8388608.0  # 2^23
                
                # Lưu vào mảng samples
                self.samples[self.sample_index] = normalized
                self.sample_index = (self.sample_index + 1) % self.buffer_size
                
            # Loại bỏ gói tin đã xử lý
            self.buffer = self.buffer[packet_size:]
            
    def get_current_samples(self, count=1000):
        """Lấy mẫu hiện tại để hiển thị"""
        if count > self.buffer_size:
            count = self.buffer_size
            
        if self.sample_index >= count:
            return self.samples[self.sample_index-count:self.sample_index]
        else:
            # Trường hợp vòng quanh buffer
            return np.concatenate((self.samples[self.buffer_size-(count-self.sample_index):], 
                                  self.samples[:self.sample_index]))
    
    def save_to_wav(self, filename, duration=5, sample_rate=96000):
        """Ghi dữ liệu vào file WAV"""
        import wave
        import struct
        
        samples_to_record = int(duration * sample_rate)  # Chuyển thành số nguyên
        recorded_samples = np.zeros(samples_to_record, dtype=np.float32)
        
        print(f"Recording {duration} seconds of audio...")
        sample_count = 0
        
        while sample_count < samples_to_record:
            prev_index = self.sample_index
            self.read_data()
            new_samples = (self.sample_index - prev_index) % self.buffer_size
            
            if new_samples > 0:
                if sample_count + new_samples <= samples_to_record:
                    if prev_index < self.sample_index:
                        recorded_samples[sample_count:sample_count+new_samples] = self.samples[prev_index:self.sample_index]
                    else:
                        # Trường hợp vòng quanh buffer
                        first_part = self.buffer_size - prev_index
                        recorded_samples[sample_count:sample_count+first_part] = self.samples[prev_index:]
                        recorded_samples[sample_count+first_part:sample_count+new_samples] = self.samples[:self.sample_index]
                    
                    sample_count += new_samples
                    print(f"Progress: {sample_count}/{samples_to_record} samples", end="\r")
                else:
                    remaining = samples_to_record - sample_count
                    recorded_samples[sample_count:] = self.samples[prev_index:prev_index+remaining]
                    sample_count = samples_to_record
            
            time.sleep(0.001)  # Tránh CPU cao
            
        print("\nSaving to WAV file...")
        
        # Chuyển sang int16 để lưu WAV
        int16_samples = (recorded_samples * 32767).astype(np.int16)
        
        with wave.open(filename, 'w') as wav_file:
            wav_file.setnchannels(1)
            wav_file.setsampwidth(2)  # 2 bytes = 16 bits
            wav_file.setframerate(sample_rate)
            wav_file.writeframes(int16_samples.tobytes())
            
        print(f"Saved to {filename}")
        
    def close(self):
        self.ser.close()

def update_plot(frame, audio_receiver, line):
    """Cập nhật đồ thị theo thời gian thực"""
    audio_receiver.read_data()
    samples = audio_receiver.get_current_samples(1000)
    line.set_ydata(samples)
    return line,

def main():
    parser = argparse.ArgumentParser(description='Receive and process 24-bit audio data from MCU')
    parser.add_argument('--port', type=str, required=True, help='Serial port (e.g., COM3 or /dev/ttyACM0)')
    parser.add_argument('--save', type=str, help='Save to WAV file')
    parser.add_argument('--duration', type=float, default=5.0, help='Recording duration in seconds')
    parser.add_argument('--plot', action='store_true', help='Show real-time plot')
    args = parser.parse_args()
    
    receiver = AudioReceiver(args.port)
    
    if args.save:
        receiver.save_to_wav(args.save, args.duration)
        
    if args.plot:
        fig, ax = plt.subplots(figsize=(12, 6))
        ax.set_ylim(-1.1, 1.1)
        ax.set_xlim(0, 1000)
        ax.set_title('Real-time 24-bit Audio Data')
        ax.set_xlabel('Sample')
        ax.set_ylabel('Amplitude')
        ax.grid(True)
        
        # Khởi tạo đường vẽ
        line, = ax.plot(np.zeros(1000))
        
        # Cập nhật đồ thị theo thời gian thực
        ani = FuncAnimation(fig, update_plot, fargs=(receiver, line), interval=50, blit=True)
        plt.tight_layout()
        plt.show()
    
    receiver.close()

if __name__ == "__main__":
    main() 