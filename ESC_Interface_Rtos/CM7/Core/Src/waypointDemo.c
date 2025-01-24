/*
 * waypointDemo.c
 *
 *  Created on: 23 nov 2024
 *      Author: javim_1oc1nxl
 */
#include "waypointDemo.h"


TIM_HandleTypeDef HTimESC;
TIM_HandleTypeDef HTimServo;

// Function to move servo to fully turn left or right
/*
void testingServoLimits() {
	setServoAngle(90); // Neutral
	osDelay(1000);

	setServoAngle(165); // Max right
	osDelay(1000);

	setServoAngle(15); // Max left
	osDelay(4000);

	setServoAngle(90); // Neutral
	osDelay(1000);
}
void testingESCLimits() {
	setServoAngle(90);   // Move to 90° position

	setEscSpeed(700); // Forward - small velocity
	osDelay(1000);

	setEscSpeed(1250); // Forward - max velocity
	osDelay(1000);

	setEscSpeed(1500); // Stop
	osDelay(4000);
	//setEscSpeed(1550);   // backwards - small velocity
	//HAL_Delay(1000);

	//setEscSpeed(1500);   // Stop
	//HAL_Delay(1000);
}
*/
void initServoTim(TIM_HandleTypeDef hTimServo){
	HTimServo = hTimServo;

	if (HAL_TIM_PWM_Start(&HTimServo, TIM_CHANNEL_1) != HAL_OK) {
		  Error_Handler();
	}
	__HAL_TIM_SET_COMPARE(&HTimServo, TIM_CHANNEL_1, 1500); // 1.5 ms pulse width for neutral
}


void initESCTim(TIM_HandleTypeDef hTimESC){
	HTimESC = hTimESC;

	if (HAL_TIM_PWM_Start(&HTimESC, TIM_CHANNEL_1) != HAL_OK) {
		  Error_Handler();
	  }
	__HAL_TIM_SET_COMPARE(&HTimESC, TIM_CHANNEL_1, 1500); // 1.5 ms pulse width for neutral
}

void setServoAngle(uint16_t angle) {
    // Map angle (0° to 180°) to pulse width (1 ms to 2 ms)
    uint16_t pulse_width = 1000 + (angle * 1000) / 180; // Scale angle to 1000-2000 μs

    // Update PWM duty cycle
    __HAL_TIM_SET_COMPARE(&HTimServo, TIM_CHANNEL_1, pulse_width);
}


void setEscSpeed(uint16_t pulse_width) {
    // Update PWM duty cycle for ESC
    __HAL_TIM_SET_COMPARE(&HTimESC, TIM_CHANNEL_1, pulse_width);
}


void waypointPathHardcode() {
    stopCar();
    osDelay(3000);

    setServoAngle(90); // Move forward
    setEscSpeed(700);
    osDelay(1000);

    setServoAngle(170); // Turn
    setEscSpeed(1100);
    osDelay(1386);

    setServoAngle(90); // Straight
    setEscSpeed(700);
    osDelay(1000);
}

void waypointPathEncoder(float position, uint16_t x, uint16_t y, uint16_t a) {
    // Detener el coche inicialmente
    //stopCar();
    // Variables para rango de posición
    float rangeStart = 0.0f;  // Inicio del rango
    float rangeEnd = 0.8f;    // Fin del rango

    // Variables de control
    bool isWithinStartZone = (x >= 0 && x < 20) && (y > 115 && y <= 120);
    bool isParabolicZone = (x >= 80 && x < 90);
    //setEscSpeed(1300);   // Velocidad hacia adelante

    // 1. Forward movement in start zone
    if (isWithinStartZone) {
        if (position >= rangeStart && position <= rangeEnd) {
            setServoAngle(70);    // Ángulo neutral del servo
            setEscSpeed(1300);   // Velocidad hacia adelante
            printf("Camera coordinates T3: x=%u, y=%u, a=%u\n", x, y, a);
            printf("Distance Traveled: %.3f\n", position);
            osDelay(500);
        }
        else if (position > rangeEnd) {
            stopCar();  // Detener el coche si excede el rango
            printf("Out of range! Position exceeded: %.3f\n", position);
        }
    }
    // 2. Parabolic trajectory in specified zone
    else if (isParabolicZone) {
        HAL_GPIO_TogglePin(LEDWay_GPIO_Port, LEDWay_Pin);
        osDelay(200);

        // Parámetros de la parábola
        float a_coeff = 0.005f;  // Mayor curvatura
        float b_coeff = 0.5f;    // Ajusta pendiente según sea necesario
        float c_coeff = 0.0f;    // Offset inicial

        // Cálculo de la trayectoria parabólica
        float parabolicY = a_coeff * x * x + b_coeff * x + c_coeff;

        // Ajuste del servo y velocidad según el valor calculado
        if (parabolicY > 20) {
            setServoAngle(50);   // Gira a la derecha
            setEscSpeed(1350);  // Reducir velocidad
        } else if (parabolicY < 10) {
            setServoAngle(130);  // Gira a la izquierda
            setEscSpeed(1350);  // Ajuste de velocidad
        } else {
            setServoAngle(90);   // Recto
            setEscSpeed(1350);   // Velocidad estándar
        }

        printf("Parabolic trajectory: x = %u, y = %.2f, a = %u\n", x, parabolicY, a);
        osDelay(500); // Delay para suavidad de movimiento
    }
    // 3. Handling other zones or fallback behavior
    else {
        osDelay(500); // Ajustar posición si es necesario
    }
    stopCar();
    // Delay final para permitir estabilidad
    osDelay(500);
}

// Function to stop the car
void stopCar() {
	setServoAngle(90);
    setEscSpeed(1500); // Neutral position (no movement)
}
