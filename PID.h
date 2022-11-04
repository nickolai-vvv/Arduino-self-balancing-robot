#ifndef _PID_h
#define _PID_h

class PID {

    uint32_t dt_ = 100; // Время итерации в мс
    double ds_ = 0.1;   // Время итерации в c
    uint32_t timer_ = 0;                    // Внутренний таймер в мс
    int min_output_ = 0, max_output_ = 255; // Ограничение управляющего воздействия
    double prevInput = 0.0;                 // Предыдущее значение входа

public:

    double Kp = 0.0;          // коэффициент P
    double Ki = 0.0;          // коэффициент I
    double Kd = 0.0;          // коэффициент D  

    double setpoint = 0.0;    // установка, которую должен поддерживать регулятор
    double input = 0.0;       // сигнал с датчика (улог наклона MPU6050)
    double output = 0.0;      // управляющее воздействие / выход с регулятора на управляющее устройство
    double summ_integr = 0.0; // интегральная сумма

    PID() {}

    /// @brief Конструктор с параметрами
    /// @param Пропорциональная составляющая
    /// @param Интегрирующая составляющая
    /// @param Дифференцирующая составляющая
    /// @param Время итерации в мс / период дискретизации
    PID(double kp, double ki, double kd, int16_t dt = 100)
        : dt_(dt), ds_(dt_ / 1000.0),
        , Kp(kp), Ki(ki), Kd(kd)
    {
        Serial.print("Kp: "); Serial.print(Kp);
        Serial.print("| Ki: "); Serial.print(Ki);
        Serial.print("| Kd: "); Serial.print(Kd);
        Serial.print("| dt: "); Serial.println(dt_);
    }//PID(double kp,....)

    ~PID() {}

    /// @brief Устанавливает ограничение на управляющее воздействие
    /// @param Минимальное значение упр. возд.
    /// @param Максимальное значение упр. возд.
    /// @deprecated микроконтроллер Arduino поддерживает восьмибитную ШИМ, что позволяет выбрать переменную в широком диапазоне значений от 0 до 255.
    void setLimits(int min_out, int max_out) {
        min_output_ = min_out;
        max_output_ = max_out;
    }//setLimits

    /// @brief Устанавливает время итерации в мс
    void setDt(uint32_t dt) { dt_ = dt; }
    /// @brief Вовзращает время итерации в мс
    uint32_t getDt() const { return dt_; }

    /// @brief Расчитывает управляющее воздействие
    double compute() {
        double error = setpoint - input;        // ошибка регулирования
        double delta_input = prevInput - input; // изменение входного сигнала за dt
        prevInput = input;

        output = error * Kp;                      // пропорциональая составляющая
        output += delta_input * Kd / ds_;   // дифференциальная составляющая

        summ_integr += error * Ki * ds_;

        summ_integr = constrain(summ_integr, min_output_, max_output_); // ограничиваем инт. сумму
        output += summ_integr;                 // интегральная составляющая

        output = constrain(output, min_output_, max_output_);   // ограничиваем выход

        return output;
    }//compute

    /// @brief Возвращает новое значение не ранее, чем через dt миллисекунд (встроенный таймер с периодом dt)
    double calculate_with_timer() {
        if (millis() - timer_ >= dt_) {
            timer_ = millis();
            compute();
        }//if
        return output;
    }//calculate_with_timer

    /// @brief Считает выход по реальному прошедшему времени между вызовами функции
    double calculate_real_time() {
        setDt(millis() - timer_);
        timer_ = millis();
        return compute();
    }//calculate_real_time
};
#endif