#define TRIG_PIN 25
#define ECHO_PIN 33

void setup() {
  // Khởi tạo serial monitor
  Serial.begin(115200);
  
  // Khởi tạo chân Trig là OUTPUT
  pinMode(TRIG_PIN, OUTPUT);
  
  // Khởi tạo chân Echo là INPUT
  pinMode(ECHO_PIN, INPUT);
}

void loop() {
  // Biến để lưu thời gian và khoảng cách
  long duration;
  float distance;
  
  
  // Đảm bảo chân Trig ở mức thấp
  digitalWrite(TRIG_PIN, LOW);
  delayMicroseconds(2);
  
  // Phát xung Trig
  digitalWrite(TRIG_PIN, HIGH);
  delayMicroseconds(10);
  digitalWrite(TRIG_PIN, LOW);
  
  // Đo độ rộng xung Echo
  duration = pulseIn(ECHO_PIN, HIGH);
  
  // Tính khoảng cách
  distance = duration * 0.034 / 2;
  
  // Hiển thị khoảng cách lên Serial Monitor
  Serial.print("Distance: ");
  Serial.print(distance);
  Serial.println(" cm");
  
  // Đợi 1 giây trước khi đo lần tiếp theo
  delay(1000);
}
