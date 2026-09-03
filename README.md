# Arduino Tremor Monitoring and Suppression Prototype
A prototype for detecting hand tremor using IMU sensor data and
frequency-domain signal processing.

The system was developed using an Arduino Nano 33 IoT. Acceleration
measurements are filtered using an 8th-order Butterworth filter and
analyzed in the frequency domain using a Fast Fourier Transform (FFT).

The system was designed to detect tremor frequencies in the 3–12 Hz
range and provide a basis for controlling a vibration motor in response
to the detected tremor.

- `tremor_sensor_inator_motor.ino` - main application and hardware control
- `butterworth.cpp/hpp` - 8th-order Butterworth filter
- `biquad.cpp/hpp` - second-order filter implementation
- `fast_fourier.cpp/hpp` - FFT and tremor detection
- `PE1MEW_Timer.cpp/hpp` - sampling timer
- `ringbuffer.h` - circular buffer utility
