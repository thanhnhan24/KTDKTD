// Pin definitions
#define trigPin  25
#define echoPin 33
#define PWM_PIN 13 // Chân GPIO sử dụng cho PWM
#define PWM_CHANNEL 0 // Kênh PWM (0 đến 15)
#define PWM_FREQ 1000 // Tần số PWM (5kHz)
#define PWM_RESOLUTION 8 // Độ phân giải PWM (8-bit, từ 0 đến 255)

// Variables
double Setpoint, Output;
double Kp = 70.0, Ki = 0.0455, Kd = 0.6;
double integral = 0;
double lastError = 0;
unsigned long previousTime = 0;
float Input;
const double tankHeight = 20.0;        // Chiều cao của bồn nước (cm)
const double startPumpingDistance = 5.0; // Khoảng cách để bắt đầu bơm (cm)
const double fullTankDistance = 8.0;   // Khoảng cách khi bồn đầy (cm)

bool swingUpDone = false; 
// Biến thời gian để thay thế hàm delay
unsigned long previousMillis = 0;
const long interval = 100;  // Thời gian chờ giữa các lần thực hiện hành động (500ms)

// New constants for the range check
const double minValidDistance = tankHeight - fullTankDistance-7;
const double maxValidDistance = tankHeight;

void setup() {
  pinMode(trigPin, OUTPUT);
  pinMode(echoPin, INPUT);
  // Thiết lập kênh PWM
  ledcSetup(PWM_CHANNEL, PWM_FREQ, PWM_RESOLUTION);
  // Gắn chân GPIO vào kênh PWM
  ledcAttachPin(PWM_PIN, PWM_CHANNEL);

  Serial.begin(115200);

  Setpoint = fullTankDistance;  // Mức nước mong muốn khi bồn đầy
}

void loop() {
    unsigned long currentMillis = millis();

    // Kiểm tra nếu đã đến thời gian thực hiện hành động
    if (currentMillis - previousMillis >= interval) {
        // Cập nhật previousMillis để theo dõi khoảng thời gian
        previousMillis = currentMillis;

        // Tính toán thời gian trôi qua
        unsigned long currentTime = millis();
        double timeChange = (double)(currentTime - previousTime);

        // Đọc giá trị từ cảm biến siêu âm
        float distance = readUltrasonicDistance();

        // Check if the distance is within the desired range
        if (distance < minValidDistance || distance > maxValidDistance) {
            // Distance is outside of the valid range; ignore this reading
            Serial.println("Invalid distance reading. Skipping PID calculation.");
            return;
        }

        // Chuyển đổi khoảng cách đo được thành mức nước
        double waterLevel = tankHeight - distance;

        // Kiểm tra xem hàm swingUp đã chạy chưa
        if (!swingUpDone) {
            // Chạy hàm swingUp để đưa mực nước gần đến Setpoint
            swingUp(waterLevel, Setpoint);
            swingUpDone = true;  // Đánh dấu hàm swingUp đã hoàn tất
        }

        // Set PI input
        Input = distance;

        // Tính toán điều khiển PID
        float error = Setpoint - (tankHeight - Input);
        integral += (error * timeChange);
        integral = constrain(integral,-500,500);
        double derivative = (error - lastError) / timeChange;
        // Tính toán giá trị điều khiển (PID Output)
        Output = Kp * error + Ki * integral + Kd * derivative;
        
        // Ensure output stays within PWM limits
        Output = constrain(Output, 65, 255);

        // Update PWM signal
        ledcWrite(PWM_CHANNEL, Output);

        // Debugging output
        Serial.print(" Setpoint: ");
        Serial.print(Setpoint);
        Serial.print(" Error: ");
        Serial.print(error,2);
        Serial.print(" Output: ");
        Serial.println(Output);

        // Update error and time
        lastError = error;
        previousTime = currentTime;
    }
}

float readUltrasonicDistance() {
    // Gửi xung siêu âm
    digitalWrite(trigPin, LOW);
    delayMicroseconds(2);
    digitalWrite(trigPin, HIGH);
    delayMicroseconds(10);
    digitalWrite(trigPin, LOW);

    // Đo thời gian xung phản hồi
    unsigned long duration = pulseIn(echoPin, HIGH);

    // Tính toán khoảng cách theo cm
    float distance = duration * 0.034 / 2;

    // Kiểm tra xem giá trị có nằm trong phạm vi hợp lệ không
    if (distance < tankHeight - Setpoint - 2 || distance > tankHeight+2) {
        // Trả về một giá trị đặc biệt để chỉ ra rằng giá trị này không hợp lệ
        return maxValidDistance + 1;  // Một giá trị ngoài phạm vi hợp lệ
    }

    return distance;
}

// Hàm swingUp để bơm nước lên thấp hơn Setpoint 1 đơn vị
void swingUp(double currentLevel, double targetLevel) {
    while (currentLevel < (targetLevel - 1)) {
        // Bơm nước ở tốc độ tối đa để đạt đến mục tiêu
        ledcWrite(PWM_CHANNEL, 255);  // Chạy bơm ở tốc độ tối đa

        // Đọc giá trị từ cảm biến siêu âm
        float distance = readUltrasonicDistance();

        // Cập nhật mức nước hiện tại
        currentLevel = tankHeight - distance;

        // Debugging output
        Serial.print("Swing-Up in progress. Water Level: ");
        Serial.println(currentLevel);

        delay(100);  // Đợi một khoảng thời gian ngắn trước khi kiểm tra lại
    }

    // Khi đã đạt đến mục tiêu, dừng bơm tạm thời
    ledcWrite(PWM_CHANNEL, 0);
}
