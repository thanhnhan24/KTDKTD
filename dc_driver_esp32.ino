#define PWM_PIN 32 // Chân GPIO sử dụng cho PWM
#define PWM_CHANNEL 0 // Kênh PWM (0 đến 15)
#define PWM_FREQ 1000 // Tần số PWM (5kHz)
#define PWM_RESOLUTION 8 // Độ phân giải PWM (8-bit, từ 0 đến 255)

void setup() {
  // Thiết lập kênh PWM
  ledcSetup(PWM_CHANNEL, PWM_FREQ, PWM_RESOLUTION);
  
  // Gắn chân GPIO vào kênh PWM
  ledcAttachPin(PWM_PIN, PWM_CHANNEL);
}

void loop() {
  // Tăng dần giá trị PWM từ 0 đến 255
  for (int dutyCycle = 0; dutyCycle <= 255; dutyCycle++) {
    ledcWrite(PWM_CHANNEL, dutyCycle); // Ghi giá trị PWM vào kênh
    delay(10); // Trễ 10ms giữa các giá trị
  }
  
  // Giảm dần giá trị PWM từ 255 xuống 0
  for (int dutyCycle = 255; dutyCycle >= 0; dutyCycle--) {
    ledcWrite(PWM_CHANNEL, dutyCycle); // Ghi giá trị PWM vào kênh
    delay(10); // Trễ 10ms giữa các giá trị
  }
}
