// Pin kết nối cảm biến S201
const int sensorPin = 2;

// Biến để lưu số xung từ cảm biến
volatile int pulseCount = 0;

// Biến để tính toán lưu lượng nước
float flowRate = 0.0;
float totalFlow = 0.0;
unsigned long oldTime = 0;

void setup() {
  // Cấu hình pin cảm biến là đầu vào
  pinMode(sensorPin, INPUT);
  digitalWrite(sensorPin, HIGH);
  
  // Cấu hình ngắt cho cảm biến
  attachInterrupt(digitalPinToInterrupt(sensorPin), pulseCounter, FALLING);
  
  // Khởi động Serial để in ra kết quả
  Serial.begin(9600);
  
  // Khởi tạo thời gian
  oldTime = millis();
}

void loop() {
  // Kiểm tra mỗi giây
  if(millis() - oldTime > 1000) {
    detachInterrupt(digitalPinToInterrupt(sensorPin));
    
    // Tính toán lưu lượng nước (lit/phút)
    flowRate = ((1000.0 / (millis() - oldTime)) * pulseCount) / 7.5;
    oldTime = millis();
    pulseCount = 0;
    
    // Tính tổng lưu lượng nước đã chảy qua
    totalFlow += flowRate / 60.0;
    
    // In ra kết quả
    Serial.print("Flow rate: ");
    Serial.print(flowRate);
    Serial.print(" L/min, Total flow: ");
    Serial.print(totalFlow);
    Serial.println(" L");
    
    attachInterrupt(digitalPinToInterrupt(sensorPin), pulseCounter, FALLING);
  }
}

// Hàm ngắt để đếm số xung
void pulseCounter() {
  pulseCount++;
}
