# Hệ thống thu thập và phân tích dữ liệu âm thanh 24-bit

Dự án này bao gồm phần firmware cho MCU STM32F4 và các công cụ Python để thu thập, hiển thị và phân tích dữ liệu âm thanh 24-bit từ bộ chuyển đổi CS5361.

## Tổng quan hệ thống

1. **MCU (STM32F4)**: Thu thập dữ liệu 24-bit từ CS5361 qua giao thức I2S, đóng gói và gửi qua USB CDC
2. **Python Receiver**: Nhận dữ liệu từ cổng USB, giải mã và hiển thị theo thời gian thực hoặc lưu thành file WAV
3. **Python Analyzer**: Phân tích chất lượng âm thanh, tính SNR, THD và hiển thị đồ thị

## Định dạng gói tin

Dữ liệu được truyền qua USB với định dạng gói tin như sau:

```
+--------+--------+--------+--------+--------+--------+--------+--------+-------------+
| Marker1| Marker2| Len_H  | Len_L  | Version| BytePS | Res1   | Res2   | Audio Data  |
| (0xAA) | (0x55) |        |        | (0x01) | (0x03) | (0x00) | (0x00) |             |
+--------+--------+--------+--------+--------+--------+--------+--------+-------------+
```

- **Marker1, Marker2**: Đánh dấu bắt đầu gói tin (0xAA, 0x55)
- **Len_H, Len_L**: Số lượng mẫu trong gói (16-bit, MSB first)
- **Version**: Phiên bản định dạng gói tin (hiện tại là 0x01)
- **BytePS**: Số byte mỗi mẫu (0x03 cho 24-bit)
- **Res1, Res2**: Dự trữ cho tương lai
- **Audio Data**: Dữ liệu âm thanh 24-bit, mỗi mẫu 3 byte (LSB first)

## Cài đặt

### Yêu cầu

- STM32CubeIDE để nạp firmware cho MCU
- Python 3.6+ với các thư viện:
  - pyserial
  - numpy
  - matplotlib
  - scipy

### Cài đặt thư viện Python

```bash
pip install pyserial numpy matplotlib scipy
```

## Sử dụng

### 1. Nạp firmware cho MCU

- Mở dự án trong STM32CubeIDE
- Build và nạp firmware cho STM32F4

### 2. Thu thập dữ liệu với Python Receiver

```bash
# Hiển thị dữ liệu theo thời gian thực
python python_receiver.py --port COM3 --plot

# Ghi dữ liệu vào file WAV (5 giây)
python python_receiver.py --port COM3 --save audio.wav --duration 5

# Vừa hiển thị vừa ghi file
python python_receiver.py --port COM3 --plot --save audio.wav --duration 10
```

Thay `COM3` bằng cổng COM tương ứng của thiết bị (Windows) hoặc `/dev/ttyACM0` (Linux).

### 3. Phân tích dữ liệu với Audio Analyzer

```bash
# Phân tích file WAV đã ghi
python audio_analyzer.py audio.wav

# Giới hạn tần số hiển thị đến 10kHz
python audio_analyzer.py audio.wav --max-freq 10000
```

## Phân tích chất lượng âm thanh

Audio Analyzer cung cấp các thông số quan trọng để đánh giá chất lượng âm thanh 24-bit:

- **SNR (Signal-to-Noise Ratio)**: Tỉ lệ tín hiệu trên nhiễu, càng cao càng tốt
- **THD (Total Harmonic Distortion)**: Độ méo hài tổng, càng thấp càng tốt
- **Frequency Spectrum**: Phổ tần số để phân tích dải tần
- **Spectrogram**: Biểu diễn phổ tần số theo thời gian

## Tối ưu hiệu suất

Để đạt hiệu suất tốt nhất khi truyền dữ liệu 24-bit:

1. **MCU**:
   - Sử dụng FIFO buffer đủ lớn để tránh mất mẫu
   - Tối ưu kích thước gói USB để cân bằng giữa độ trễ và hiệu suất
   - Thêm cơ chế phục hồi khi USB bị nghẽn

2. **Python**:
   - Sử dụng buffer đủ lớn để xử lý các gói không đồng bộ
   - Xử lý dữ liệu trong thread riêng để không ảnh hưởng đến hiển thị
   - Lưu ý đến việc chuyển đổi định dạng 24-bit sang định dạng phù hợp cho xử lý

## Xử lý sự cố

1. **Không nhận được dữ liệu**:
   - Kiểm tra kết nối USB
   - Kiểm tra cổng COM đúng chưa
   - Đảm bảo CS5361 được cấu hình đúng

2. **Dữ liệu bị nhiễu hoặc không liên tục**:
   - Tăng kích thước FIFO buffer
   - Kiểm tra xung clock I2S
   - Đảm bảo nguồn điện ổn định

3. **Mất mẫu hoặc dữ liệu không đồng bộ**:
   - Kiểm tra header gói tin
   - Tăng tốc độ xử lý ở phía Python
   - Giảm kích thước gói USB nếu cần

## Đóng góp

Mọi đóng góp đều được hoan nghênh! Vui lòng tạo issue hoặc pull request. 