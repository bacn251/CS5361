#!/usr/bin/env python3
import numpy as np
import matplotlib.pyplot as plt
from scipy import signal
from scipy.io import wavfile
import argparse
import os

class AudioAnalyzer:
    def __init__(self, data=None, sample_rate=48000):
        self.data = data
        self.sample_rate = sample_rate
        
    def load_wav(self, filename):
        """Đọc file WAV"""
        sample_rate, data = wavfile.read(filename)
        self.sample_rate = sample_rate
        
        # Chuẩn hóa dữ liệu về khoảng [-1, 1]
        if data.dtype == np.int16:
            self.data = data.astype(np.float32) / 32768.0
        elif data.dtype == np.int32:
            self.data = data.astype(np.float32) / 2147483648.0
        elif data.dtype == np.float32:
            self.data = data
        
        return self.data
        
    def plot_waveform(self, title="Audio Waveform"):
        """Vẽ dạng sóng audio"""
        if self.data is None:
            print("No data to plot")
            return
            
        plt.figure(figsize=(12, 6))
        time = np.arange(0, len(self.data)) / self.sample_rate
        plt.plot(time, self.data)
        plt.title(title)
        plt.xlabel("Time (s)")
        plt.ylabel("Amplitude")
        plt.grid(True)
        plt.tight_layout()
        
    def plot_spectrogram(self, title="Audio Spectrogram"):
        """Vẽ phổ tần số theo thời gian"""
        if self.data is None:
            print("No data to plot")
            return
            
        plt.figure(figsize=(12, 6))
        
        # Tính toán và vẽ spectrogram
        f, t, Sxx = signal.spectrogram(self.data, fs=self.sample_rate, 
                                      nperseg=1024, noverlap=512)
        
        # Chuyển sang dB
        Sxx_db = 10 * np.log10(Sxx + 1e-10)
        
        plt.pcolormesh(t, f, Sxx_db, shading='gouraud')
        plt.title(title)
        plt.ylabel('Frequency (Hz)')
        plt.xlabel('Time (s)')
        plt.colorbar(label='Intensity (dB)')
        plt.tight_layout()
        
    def plot_fft(self, title="Frequency Spectrum", max_freq=None):
        """Vẽ phổ tần số (FFT)"""
        if self.data is None:
            print("No data to plot")
            return
            
        plt.figure(figsize=(12, 6))
        
        # Tính FFT
        n = len(self.data)
        fft_data = np.abs(np.fft.rfft(self.data))
        
        # Chuẩn hóa
        fft_data = fft_data / n
        
        # Chuyển sang dB
        fft_data_db = 20 * np.log10(fft_data + 1e-10)
        
        # Tạo trục tần số
        freqs = np.fft.rfftfreq(n, 1/self.sample_rate)
        
        # Giới hạn tần số hiển thị nếu cần
        if max_freq is not None and max_freq < self.sample_rate/2:
            idx = np.where(freqs <= max_freq)[0]
            freqs = freqs[idx]
            fft_data_db = fft_data_db[idx]
        
        plt.plot(freqs, fft_data_db)
        plt.title(title)
        plt.xlabel('Frequency (Hz)')
        plt.ylabel('Magnitude (dB)')
        plt.grid(True)
        plt.tight_layout()
        
    def calculate_snr(self):
        """Tính tỉ lệ tín hiệu trên nhiễu (SNR)"""
        if self.data is None:
            print("No data to calculate SNR")
            return None
            
        # Tính FFT
        fft_data = np.abs(np.fft.rfft(self.data))
        
        # Tìm đỉnh tín hiệu (giả sử đỉnh cao nhất là tín hiệu)
        peak_idx = np.argmax(fft_data)
        
        # Tính công suất tín hiệu (peak và các bin lân cận)
        window = 5  # Số bin lân cận để tính
        start_idx = max(0, peak_idx - window)
        end_idx = min(len(fft_data), peak_idx + window + 1)
        signal_power = np.sum(fft_data[start_idx:end_idx]**2)
        
        # Tính công suất nhiễu (phần còn lại)
        noise_power = np.sum(fft_data**2) - signal_power
        
        # Tính SNR
        if noise_power > 0:
            snr = 10 * np.log10(signal_power / noise_power)
        else:
            snr = float('inf')
            
        return snr
        
    def calculate_thd(self, fundamental_freq=None):
        """Tính độ méo hài tổng (THD)"""
        if self.data is None:
            print("No data to calculate THD")
            return None
            
        # Tính FFT
        n = len(self.data)
        fft_data = np.abs(np.fft.rfft(self.data))
        freqs = np.fft.rfftfreq(n, 1/self.sample_rate)
        
        # Nếu không chỉ định tần số cơ bản, tìm đỉnh cao nhất
        if fundamental_freq is None:
            peak_idx = np.argmax(fft_data)
            fundamental_freq = freqs[peak_idx]
        else:
            # Tìm bin gần nhất với tần số cơ bản
            peak_idx = np.argmin(np.abs(freqs - fundamental_freq))
            
        # Lấy giá trị tần số cơ bản chính xác
        fundamental_freq = freqs[peak_idx]
        fundamental_amp = fft_data[peak_idx]
        
        # Tìm các hài
        harmonics_amp = []
        for i in range(2, 6):  # Tính đến bậc 5
            harmonic_freq = fundamental_freq * i
            if harmonic_freq < self.sample_rate/2:  # Đảm bảo trong dải Nyquist
                # Tìm bin gần nhất
                h_idx = np.argmin(np.abs(freqs - harmonic_freq))
                # Lấy biên độ
                harmonics_amp.append(fft_data[h_idx])
                
        # Tính THD
        if fundamental_amp > 0:
            thd = np.sqrt(np.sum(np.array(harmonics_amp)**2)) / fundamental_amp
            thd_db = 20 * np.log10(thd)
            thd_percent = thd * 100
        else:
            thd_db = float('inf')
            thd_percent = float('inf')
            
        return {
            'fundamental_freq': fundamental_freq,
            'thd_ratio': thd,
            'thd_db': thd_db,
            'thd_percent': thd_percent,
            'harmonics': harmonics_amp
        }
        
    def analyze_file(self, filename, max_freq=20000):
        """Phân tích toàn diện file âm thanh"""
        print(f"Analyzing file: {filename}")
        self.load_wav(filename)
        
        # Tính các thông số
        snr = self.calculate_snr()
        thd = self.calculate_thd()
        
        # In kết quả
        print(f"Sample Rate: {self.sample_rate} Hz")
        print(f"Duration: {len(self.data)/self.sample_rate:.2f} seconds")
        print(f"SNR: {snr:.2f} dB")
        print(f"THD: {thd['thd_percent']:.4f}% ({thd['thd_db']:.2f} dB)")
        print(f"Fundamental Frequency: {thd['fundamental_freq']:.2f} Hz")
        
        # Vẽ đồ thị
        self.plot_waveform(f"Waveform - {os.path.basename(filename)}")
        self.plot_spectrogram(f"Spectrogram - {os.path.basename(filename)}")
        self.plot_fft(f"Frequency Spectrum - {os.path.basename(filename)}", max_freq)
        
        plt.show()

def main():
    parser = argparse.ArgumentParser(description='Analyze audio files with focus on 24-bit quality')
    parser.add_argument('file', type=str, help='WAV file to analyze')
    parser.add_argument('--max-freq', type=int, default=20000, help='Maximum frequency to display in FFT plot')
    args = parser.parse_args()
    
    analyzer = AudioAnalyzer()
    analyzer.analyze_file(args.file, args.max_freq)

if __name__ == "__main__":
    main() 