/* USER CODE BEGIN Header */
/**
  ******************************************************************************
  * @file           : main.c
  * @brief          : Main program body
  ******************************************************************************
  * @attention
  *
  * Copyright (c) 2026 STMicroelectronics.
  * All rights reserved.
  *
  * This software is licensed under terms that can be found in the LICENSE file
  * in the root directory of this software component.
  * If no LICENSE file comes with this software, it is provided AS-IS.
  *
  ******************************************************************************
  */
/* USER CODE END Header */
/* Includes ------------------------------------------------------------------*/
#include "main.h"
#include "i2c.h"
#include "usart.h"
#include "gpio.h"

/* Private includes ----------------------------------------------------------*/
/* USER CODE BEGIN Includes */
#include "pca9685.h"
#include "mpu6050.h"
#include <stdlib.h>
#include "math.h"
#include "stdio.h"
#include <string.h>
#include <stdarg.h>
#include <ctype.h>
/* USER CODE END Includes */

/* Private typedef -----------------------------------------------------------*/
/* USER CODE BEGIN PTD */

/* USER CODE END PTD */

/* Private define ------------------------------------------------------------*/
/* USER CODE BEGIN PD */
// Define the lengths of the robotic leg segments in millimeters
#define L1 56.0f  // Upper leg length (mm)
#define L2 74.0f  // Lower leg length (mm)
#define L3 15.0f  // Vertical hip length (mm)
#define L4 50.0f  // Horizontal hip length (mm)
/* USER CODE END PD */

/* Private macro -------------------------------------------------------------*/
/* USER CODE BEGIN PM */

/* USER CODE END PM */

/* Private variables ---------------------------------------------------------*/

/* USER CODE BEGIN PV */
MPU6050_t mpu_data;
/* USER CODE END PV */

/* Private function prototypes -----------------------------------------------*/
void SystemClock_Config(void);
/* USER CODE BEGIN PFP */

/* USER CODE END PFP */

/* Private user code ---------------------------------------------------------*/
/* USER CODE BEGIN 0 */
#ifndef M_PI
#define M_PI 3.14159265358979323846f
#endif

/**
 * @brief Calculates 2-DOF Inverse Kinematics for a robotic leg.
 * @param x Target X position relative to hip origin.
 * @param y Target Y position relative to hip origin.
 * @param theta1 Pointer to store calculated hip angle (in degrees).
 * @param theta2 Pointer to store calculated knee angle (in degrees).
 * @return 0 on success, 1 if target is unreachable.
 */
uint8_t IK_Calculate2DOF(float x, float y, float *theta1, float *theta2)
{
    float d_sq = x * x + y * y;
    float d = sqrtf(d_sq);

    // Reachability Check: target out of maximum extension or minimum retraction
    if (d > (L1 + L2) || d < fabsf(L1 - L2) || d == 0.0f) {
        return 1; // Unreachable target
    }

    // Solve Knee Angle (theta2) via Law of Cosines
    float cos_theta2 = (d_sq - L1 * L1 - L2 * L2) / (2.0f * L1 * L2);
    if (cos_theta2 > 1.0f) cos_theta2 = 1.0f;
    if (cos_theta2 < -1.0f) cos_theta2 = -1.0f;

    float theta2_rad = acosf(cos_theta2);

    // Solve Hip Angle (theta1)
    float alpha = atan2f(y, x);
    float cos_beta = (d_sq + L1 * L1 - L2 * L2) / (2.0f * d * L1);
    if (cos_beta > 1.0f) cos_beta = 1.0f;
    if (cos_beta < -1.0f) cos_beta = -1.0f;

    float beta = acosf(cos_beta);
    float theta1_rad = alpha + beta; // Knee-forward configuration

    // Convert radians to degrees
    *theta1 = theta1_rad * (180.0f / M_PI);
    *theta2 = theta2_rad * (180.0f / M_PI);

    return 0; // Success
}

/**
 * @brief Formats and transmits MPU6050 sensor readings over UART1.
 */
void MPU6050_UART_Print(UART_HandleTypeDef *huart, const MPU6050_t *data) {
    char msg[128];
    
    // Format: Accel (g) [X, Y, Z] | Gyro (deg/s) [X, Y, Z] | Temp (°C)
    int len = snprintf(msg, sizeof(msg),
                       "ACC[g]: %6.2f, %6.2f, %6.2f | GYR[dps]: %6.1f, %6.1f, %6.1f | T: %.1f C\r\n",
                       data->accel_g[0], data->accel_g[1], data->accel_g[2],
                       data->gyro_dps[0], data->gyro_dps[1], data->gyro_dps[2],
                       data->temp_c);

    if (len > 0) {
        HAL_UART_Transmit(huart, (uint8_t *)msg, (uint16_t)len, 100);
    }
}

typedef struct {
    float current_angle;
    float target_angle;
    float step_increment;
} JointState_t;

JointState_t dog_joints[12];

/**
 * @brief Sets the target angle for a specific joint and calculates the incremental step for smooth movement.
 */
void Set_Posture_Target(uint8_t joint_id, float new_target, float duration_s) {
    dog_joints[joint_id].target_angle = new_target;
    
    // 計算 20ms (0.02s) control loop 之下總共要行幾多步
    float total_steps = duration_s / 0.020f; 
    
    // 計算每 20ms 要 increment 幾多
    dog_joints[joint_id].step_increment = (new_target - dog_joints[joint_id].current_angle) / total_steps;
}

/**
 * @brief Updates all joints towards their target angles using ramping for smooth transitions.
 */
void Update_Joints_Ramping(void) {
    for (int i = 0; i < 12; i++) {
      if (dog_joints[i].step_increment == 0.0f) continue; // 已經到咗 target，skip 運算

      float error = dog_joints[i].target_angle - dog_joints[i].current_angle;
        
      // 如果剩低嘅 error 距離，細過或等於一步可以行嘅距離
      if (fabsf(error) <= fabsf(dog_joints[i].step_increment)) {
        dog_joints[i].current_angle = dog_joints[i].target_angle; // 直接 Snap 過去
        dog_joints[i].step_increment = 0.0f;                      // 清零停低
      }else {
        dog_joints[i].current_angle += dog_joints[i].step_increment;
      }
      
      float safe_angle = dog_joints[i].current_angle;
      if (safe_angle > 180.0f) safe_angle = 180.0f;
      if (safe_angle < 0.0f) safe_angle = 0.0f;
      PCA9685_SetServoAngle(&hi2c3, i, safe_angle);
    }
}

/* claude code added*/
void Apply_All_Joints(void)
{
  for (int i = 0; i < 12; i++)
  {
    PCA9685_SetServoAngle(&hi2c3, i, dog_joints[i].current_angle);
  }
}

typedef struct {
    const char *name;
    uint8_t hip_ch, knee_ch;
    float hip_off, knee_off;
    float hip_sign, knee_sign;
    uint8_t ankle_ch;
    float ankle_angle;      // angle at which the lower leg is vertical
} Leg_t;

/* servo = sign * IK + offset */
Leg_t legs[4] = {
    /* name  hip knee  hip_off  knee_off  hip_sign knee_sign  ankle_ch ankle_angle */
    { "FR",   1,  0,   133.24f,  94.31f,   1.0f,    1.0f,       2,      90.0f },
    { "FL",   4,  5,    66.76f,  85.69f,  -1.0f,   -1.0f,       3,      90.0f },
    { "BR",   7,  6,   128.24f,  69.31f,   1.0f,    1.0f,       8,     120.0f },
    { "BL",  10, 11,    46.76f, 100.69f,  -1.0f,   -1.0f,       9,      90.0f }
};
 
/* printf-style helper over UART1 */
void Uart_Print(const char *fmt, ...)
{
    char buf[160];
    va_list args;
    va_start(args, fmt);
    int len = vsnprintf(buf, sizeof(buf), fmt, args);
    va_end(args);
    if (len > 0) {
        if (len > (int)sizeof(buf) - 1) len = sizeof(buf) - 1;
        HAL_UART_Transmit(&huart1, (uint8_t *)buf, (uint16_t)len, 100);
    }
}

static uint8_t leg_verbose = 1;
/* Move one foot to (x, y). Returns 0 on success.
 * Refuses to move (instead of clamping) if a servo would leave 0..180. */
uint8_t Leg_SetFoot(uint8_t leg, float x, float y, float duration_s)
{
    float hip_ik, knee_ik;
 
    if (IK_Calculate2DOF(x, y, &hip_ik, &knee_ik) != 0) {
        Uart_Print("%s: X=%.1f Y=%.1f unreachable\r\n", legs[leg].name, x, y);
        return 1;
    }
 
    float hip  = legs[leg].hip_sign  * hip_ik  + legs[leg].hip_off;
    float knee = legs[leg].knee_sign * knee_ik + legs[leg].knee_off;
  
    if (leg_verbose) {
      Uart_Print("%s: X=%.1f Y=%.1f | IK Hip=%.2f Knee=%.2f | Servo Hip=%.2f Knee=%.2f\r\n",
                      legs[leg].name, x, y, hip_ik, knee_ik, hip, knee);
    }
 
    if (hip < 0.0f || hip > 180.0f || knee < 0.0f || knee > 180.0f) {
        Uart_Print("%s: SERVO OUT OF RANGE, not moving\r\n", legs[leg].name);
        return 2;
    }
 
    Set_Posture_Target(legs[leg].hip_ch,  hip,  duration_s);
    Set_Posture_Target(legs[leg].knee_ch, knee, duration_s);
    Set_Posture_Target(legs[leg].ankle_ch, legs[leg].ankle_angle, duration_s); 
    return 0;
}

/* Same target on one leg (leg 0..3) or all legs (leg = -1) */
void Test_Legs(int leg, float x, float y)
{
    if (leg >= 0 && leg < 4) {
        Leg_SetFoot((uint8_t)leg, x, y, 2.0f);
    } else {
        for (uint8_t i = 0; i < 4; i++) {
            Leg_SetFoot(i, x, y, 2.0f);
        }
    }
}

/* wlaking*/
#define GAIT_X_STAND   1.2f     /* calibrated standing X (mm)            */
#define GAIT_Y_STAND  (-120.0f) /* foot height while on the ground (mm)  */
#define GAIT_Y_LIFT   (-105.0f) /* foot height while swinging (15 mm up) */
#define GAIT_STEP_MM   15.0f    /* half stride: foot travels X_STAND +/- this */
#define GAIT_X_FRONT  (GAIT_X_STAND + GAIT_STEP_MM)
#define GAIT_SHIFT_MM (2.0f * GAIT_STEP_MM / 3.0f)  /* stance feet move back this much per phase */
 
#define GAIT_SWING_MS 300       /* time to lift + swing forward */
#define GAIT_LOWER_MS 200       /* time to put the foot down    */
 
/* leg index: 0=FR 1=FL 2=BR 3=BL.  Swing order FR -> BL -> FL -> BR */
static const uint8_t swing_order[4] = { 0, 3, 1, 2 };
 
static float    foot_x[4];
static uint8_t  gait_active = 0;
static uint8_t  gait_phase  = 0;   /* 0..3 : which leg swings        */
static uint8_t  gait_sub    = 0;   /* 0 = lift+swing, 1 = lower      */
static uint32_t gait_next_tick = 0;
 
void Gait_Stop(void)
{
    gait_active = 0;
    leg_verbose = 1;
    Test_Legs(-1, GAIT_X_STAND, GAIT_Y_STAND);   /* back to standing */
    Uart_Print("Gait stopped\r\n");
}
 
void Gait_Start(void)
{
    /* Stagger the feet so the leg that swings first is furthest back. */
    leg_verbose = 0;
    for (uint8_t p = 0; p < 4; p++) {
        uint8_t leg = swing_order[p];
        foot_x[leg] = GAIT_X_FRONT - (3 - p) * GAIT_SHIFT_MM;
        if (Leg_SetFoot(leg, foot_x[leg], GAIT_Y_STAND, 1.5f) != 0) {
            gait_active = 0;
            leg_verbose = 1;
            Uart_Print("Gait start failed\r\n");
            return;
        }
    }
    gait_phase = 0;
    gait_sub = 0;
    gait_next_tick = HAL_GetTick() + 2000;   /* wait for the setup move */
    gait_active = 1;
    Uart_Print("Gait started\r\n");
}
 
/* Call this every 20 ms from the main loop. Non-blocking. */
void Gait_Update(void)
{
    if (!gait_active) return;
    if ((int32_t)(HAL_GetTick() - gait_next_tick) < 0) return;
 
    uint8_t swing = swing_order[gait_phase];
    uint8_t err = 0;
 
    if (gait_sub == 0) {
        /* Swing leg lifts and goes to the front, other three push back. */
        for (uint8_t i = 0; i < 4; i++) {
            if (i == swing) {
                foot_x[i] = GAIT_X_FRONT;
                err |= Leg_SetFoot(i, foot_x[i], GAIT_Y_LIFT, GAIT_SWING_MS / 1000.0f);
            } else {
                foot_x[i] -= GAIT_SHIFT_MM;
                err |= Leg_SetFoot(i, foot_x[i], GAIT_Y_STAND, GAIT_SWING_MS / 1000.0f);
            }
        }
        gait_next_tick = HAL_GetTick() + GAIT_SWING_MS;
        gait_sub = 1;
    } else {
        /* Put the swing foot back on the ground. */
        err |= Leg_SetFoot(swing, foot_x[swing], GAIT_Y_STAND, GAIT_LOWER_MS / 1000.0f);
        gait_next_tick = HAL_GetTick() + GAIT_LOWER_MS;
        gait_sub = 0;
        gait_phase = (gait_phase + 1) & 3;
    }
 
    if (err) {            /* unreachable or servo out of range */
        Gait_Stop();
    }
}
/*walking*/
/* Handles a text command such as "s", "f", "b", "u", "f1", "s3"
 *   first letter : s = stand (1.2,-120)   f = forward (40,-100)
 *                  b = back (-20,-115)    u = up (10,-100)
 *   optional digit: 0=FR 1=FL 2=BR 3=BL, no digit = all four legs
 * Returns 1 if the command was recognised. */
uint8_t Handle_Test_Command(const char *cmd)
{
  if (tolower((unsigned char)cmd[0]) != 'g') gait_active = 0;
  float x, y;

  switch (tolower((unsigned char)cmd[0])) {
      case 's': x =  1.2f; y = -120.0f; break;
      case 'f': x = 40.0f; y = -100.0f; break;
      case 'b': x = -20.0f; y = -115.0f; break;
      case 'u': x = 10.0f; y = -100.0f; break;
      case 'g': Gait_Start(); return 1;
      case 'x': Gait_Stop();  return 1;
      default:  return 0;
  }

  int leg = -1;
  if (cmd[1] >= '0' && cmd[1] <= '3') {
      leg = cmd[1] - '0';
  }

  Test_Legs(leg, x, y);
  return 1;
}
/* claude code added*/

/**
 * @brief Initializes all dog joints to a neutral position (90 degrees) and resets their state.
 */
void init_dog_joints(void)
{
  const float stand_pose[12] =
  {
    140.0f,  // CH0 FR knee
    70.0f,  // CH1 FR hip
    90.0f,  // CH2 FR ankle
    90.0f,  // CH3 LF ankle

    130.0f,  // CH4 FL hip
    40.0f,  // CH5 FL knee
    115.0f,  // CH6 BR knee
    65.0f,  // CH7 BR hip

    120.0f,  // CH8 BR ankle
    90.0f,  // CH9 BL ankle
    110.0f,  // CH10 BL hip
    55.0f   // CH11 BL knee
  };

  for (int i = 0; i < 12; i++)
  {
    dog_joints[i].current_angle = stand_pose[i];
    dog_joints[i].target_angle = stand_pose[i];
    dog_joints[i].step_increment = 0.0f;
  }
}

void Set_Calibrated_Standing_Pose(float duration_s)
{
  const float stand_pose[12] =
  {
    140.0f,
    70.0f,
    90.0f,
    90.0f,

    130.0f,
    40.0f,
    115.0f,
    65.0f,

    90.0f,
    90.0f,
    110.0f,
    55.0f
  };

  for (int i = 0; i < 12; i++)
  {
    Set_Posture_Target(i, stand_pose[i], duration_s);
  }
}

typedef enum {
    POSTURE_STANDING,
    POSTURE_SITTING
} DogPosture_t;

/**
 * @brief Sets the dog's posture by calculating the required joint angles using inverse kinematics.
 * @param posture Desired posture (standing or sitting).
 */
void Set_Dog_Posture(DogPosture_t posture)
{
  float target_x;
  float target_y;
  float hip_target;
  float knee_target;

  // Select posture
  uint8_t stand_pose[12] =
  {
    140,    // CH0 RF knee
    70,    // CH1 RF hip
    90,    // CH2 RF ankle
    90,    // CH3 LF ankle (unplugged)
    130,    // CH4 LF hip
    40,    // CH5 LF knee
    115,    // CH6 RB knee
    65,    // CH7 RB hip
    90,    // CH8 RB ankle
    90,    // CH9 LB ankle
    110,    // CH10 LB hip
    55     // CH11 LB knee
  };

  switch (posture)
  {
    case POSTURE_STANDING:
      target_x = 1.2f;
      target_y = -120.0f;
      break;

    case POSTURE_SITTING:
      target_x = 90.0f;
      target_y = 90.0f;
      break;

    default:
      return;
  }

  // Calculate IK
  if (IK_Calculate2DOF(target_x, target_y, &hip_target, &knee_target) == 0)
  {
    // Map IK angles to servo angles
    float hip_mapped  = hip_target  + 133.24f;
    float knee_mapped = knee_target + 94.31f;
    float duration = 2.0f; // Duration in seconds for smooth transition

    // Front right
    Set_Posture_Target(0, knee_mapped, duration);
    Set_Posture_Target(1, hip_mapped, duration);

    // Back right
    Set_Posture_Target(6, knee_mapped, duration);
    Set_Posture_Target(7, hip_mapped, duration);

    float hip_mirrored = 180.0f - hip_mapped;
    float knee_mirrored = 180.0f - knee_mapped;

    // Front left
    Set_Posture_Target(4, hip_mirrored, duration);
    Set_Posture_Target(5, knee_mirrored, duration);

    // Back left
    Set_Posture_Target(10, hip_mirrored, duration);
    Set_Posture_Target(11, knee_mirrored, duration);
  }
}

void Test_FR_IK_Standing(void)
{
    float x = 10.0f;
    float y = -100.0f;

    float hip_target;
    float knee_target;

    if (IK_Calculate2DOF(x, y, &hip_target, &knee_target) != 0)
    {
        char msg[] = "FR IK target unreachable!\r\n";

        HAL_UART_Transmit(
            &huart1,
            (uint8_t *)msg,
            strlen(msg),
            100
        );

        return;
    }

    /*
     * Convert mathematical IK angles
     * to the calibrated FR servo angles.
     */
    float hip_servo  = hip_target  + 133.24f;
    float knee_servo = knee_target + 94.31f;

    char msg[160];

    int len = sprintf(
        msg,
        "FR IK: X=%.1f Y=%.1f | "
        "IK Hip=%.2f Knee=%.2f | "
        "Servo Hip=%.2f Knee=%.2f\r\n",
        x,
        y,
        hip_target,
        knee_target,
        hip_servo,
        knee_servo
    );

    HAL_UART_Transmit(
        &huart1,
        (uint8_t *)msg,
        len,
        100
    );

    /*
     * Front-right:
     * CH0 = knee
     * CH1 = hip
     */

    Set_Posture_Target(1, hip_servo, 2.0f);
    Set_Posture_Target(0, knee_servo, 2.0f);
}
/* USER CODE END 0 */

/**
  * @brief  The application entry point.
  * @retval int
  */
int main(void)
{

  /* USER CODE BEGIN 1 */

  /* USER CODE END 1 */

  /* MCU Configuration--------------------------------------------------------*/

  /* Reset of all peripherals, Initializes the Flash interface and the Systick. */
  HAL_Init();

  /* USER CODE BEGIN Init */

  /* USER CODE END Init */

  /* Configure the system clock */
  SystemClock_Config();

  /* USER CODE BEGIN SysInit */

  /* USER CODE END SysInit */

  /* Initialize all configured peripherals */
  MX_GPIO_Init();
  MX_USART1_UART_Init();
  MX_I2C3_Init();
  //MX_I2C4_Init();
  /* USER CODE BEGIN 2 */
  HAL_Delay(100); // Allow time for peripherals to stabilize

  // Initialize PCA9685 and set initial servo angles
  PCA9685_Init(&hi2c3);
  init_dog_joints();
  Apply_All_Joints();  
  HAL_Delay(500);
  //Test_FR_IK_Standing();

  //Set_Calibrated_Standing_Pose(3.0f);
  //Set_Dog_Posture(POSTURE_SITTING); // Start in sitting posture

  char rx_buffer[20];
  char tx_buffer[64];
  uint8_t rx_index = 0;
  uint8_t rx_byte = 0;
  int channel = 0;
  float target_x = 0.0f;
  float target_y = 0.0f;
  float hip_deg = 90.0f;
  float knee_deg = 90.0f;

  float test_x = 1.2f;
  float test_y = -120.0f;
  float test_hip = 0.0f;
  float test_knee = 0.0f;

  if (IK_Calculate2DOF(test_x, test_y, &test_hip, &test_knee) == 0)
  {
      float hip_mapped = test_hip + 180.0f;

      sprintf(tx_buffer,
              "IK: X=%.1f Y=%.1f | Hip=%.2f Knee=%.2f | ServoHip=%.2f ServoKnee=%.2f\r\n",
              test_x,
              test_y,
              test_hip,
              test_knee,
              hip_mapped,
              test_knee);

      HAL_UART_Transmit(
          &huart1,
          (uint8_t *)tx_buffer,
          strlen(tx_buffer),
          100
      );
  }

  // Initialize MPU-6050 using I2C4
  // if (MPU6050_Init(&hi2c4) != HAL_OK) {
  //   char err_msg[] = "MPU6050 Initialization Failed on I2C4!\r\n";
  //   HAL_UART_Transmit(&huart1, (uint8_t *)err_msg, sizeof(err_msg) - 1, 100);
  //   Error_Handler();
  // }else {
  //   char ok_msg[] = "MPU6050 Initialized Successfully on I2C4.\r\n";
  //   HAL_UART_Transmit(&huart1, (uint8_t *)ok_msg, sizeof(ok_msg) - 1, 100);
  // }
  /* USER CODE END 2 */

  /* Infinite loop */
  /* USER CODE BEGIN WHILE */
  uint32_t last_control_tick = 0;
  const uint32_t CONTROL_PERIOD_MS = 20; // 50Hz update rate
  while (1)
  {
    uint32_t current_tick = HAL_GetTick();

    // Check if 20ms have passed since the last control execution
    if ((current_tick - last_control_tick) >= CONTROL_PERIOD_MS) 
    {
      last_control_tick = current_tick; // Update timestamp

      // // 1. Read Sensors
      // if (MPU6050_Read_All(&hi2c4, &mpu_data) == HAL_OK) {
      //   // Process IMU data
      //   MPU6050_UART_Print(&huart1, &mpu_data);
      // }

      // 2. Compute Kinematics & Update Actuators
      Gait_Update();
      Update_Joints_Ramping(); // Update all joints towards their targets
    }

    //check_uart();
    if (HAL_UART_Receive(&huart1, &rx_byte, 1, 10) == HAL_OK)
    {
        if (rx_byte == '\r')
        {
          continue;
        }

        if (rx_byte == ',')
        {
          rx_buffer[rx_index] = '\0';

          if (rx_index > 0)
          {
            channel = atoi(rx_buffer);
          }

          rx_index = 0;
        }
        else if (rx_byte == '\n')
        {
          rx_buffer[rx_index] = '\0';

          if (rx_index > 0 && isalpha((unsigned char)rx_buffer[0]))
          {
            if (!Handle_Test_Command(rx_buffer))
            {
              Uart_Print("Unknown command\r\n");
            }
          }
          else if (rx_index > 0)
          {
            float angle = atof(rx_buffer);

            if (channel >= 0 && channel < 12 && angle >= 0.0f && angle <= 180.0f)
            {
              Set_Posture_Target(channel, angle, 0.5f);

              int len = sprintf(
                tx_buffer,
                "Servo %d target -> %.1f deg\r\n",
                channel,
                angle
              );

              HAL_UART_Transmit(&huart1, (uint8_t *)tx_buffer, len, 100);
            }
            else
            {
              char error[] = "Invalid servo or angle\r\n";
              HAL_UART_Transmit(&huart1, (uint8_t *)error, sizeof(error) - 1, 100);
            }
          }

          rx_index = 0;
        }
        else if (rx_index < sizeof(rx_buffer) - 1)
        {
          rx_buffer[rx_index++] = rx_byte;
        }
    }
    /* USER CODE END WHILE */

    /* USER CODE BEGIN 3 */
  }
  /* USER CODE END 3 */
}

/**
  * @brief System Clock Configuration
  * @retval None
  */
void SystemClock_Config(void)
{
  RCC_OscInitTypeDef RCC_OscInitStruct = {0};
  RCC_ClkInitTypeDef RCC_ClkInitStruct = {0};

  /** Configure the main internal regulator output voltage
  */
  HAL_PWREx_ControlVoltageScaling(PWR_REGULATOR_VOLTAGE_SCALE1);

  /** Initializes the RCC Oscillators according to the specified parameters
  * in the RCC_OscInitTypeDef structure.
  */
  RCC_OscInitStruct.OscillatorType = RCC_OSCILLATORTYPE_HSI;
  RCC_OscInitStruct.HSIState = RCC_HSI_ON;
  RCC_OscInitStruct.HSICalibrationValue = RCC_HSICALIBRATION_DEFAULT;
  RCC_OscInitStruct.PLL.PLLState = RCC_PLL_ON;
  RCC_OscInitStruct.PLL.PLLSource = RCC_PLLSOURCE_HSI;
  RCC_OscInitStruct.PLL.PLLM = RCC_PLLM_DIV1;
  RCC_OscInitStruct.PLL.PLLN = 9;
  RCC_OscInitStruct.PLL.PLLP = RCC_PLLP_DIV2;
  RCC_OscInitStruct.PLL.PLLQ = RCC_PLLQ_DIV2;
  RCC_OscInitStruct.PLL.PLLR = RCC_PLLR_DIV2;
  if (HAL_RCC_OscConfig(&RCC_OscInitStruct) != HAL_OK)
  {
    Error_Handler();
  }

  /** Initializes the CPU, AHB and APB buses clocks
  */
  RCC_ClkInitStruct.ClockType = RCC_CLOCKTYPE_HCLK|RCC_CLOCKTYPE_SYSCLK
                              |RCC_CLOCKTYPE_PCLK1|RCC_CLOCKTYPE_PCLK2;
  RCC_ClkInitStruct.SYSCLKSource = RCC_SYSCLKSOURCE_PLLCLK;
  RCC_ClkInitStruct.AHBCLKDivider = RCC_SYSCLK_DIV1;
  RCC_ClkInitStruct.APB1CLKDivider = RCC_HCLK_DIV1;
  RCC_ClkInitStruct.APB2CLKDivider = RCC_HCLK_DIV1;

  if (HAL_RCC_ClockConfig(&RCC_ClkInitStruct, FLASH_LATENCY_2) != HAL_OK)
  {
    Error_Handler();
  }
}

/* USER CODE BEGIN 4 */

/* USER CODE END 4 */

/**
  * @brief  This function is executed in case of error occurrence.
  * @retval None
  */
void Error_Handler(void)
{
  /* USER CODE BEGIN Error_Handler_Debug */
  /* User can add his own implementation to report the HAL error return state */
  __disable_irq();
  while (1)
  {
  }
  /* USER CODE END Error_Handler_Debug */
}
#ifdef USE_FULL_ASSERT
/**
  * @brief  Reports the name of the source file and the source line number
  *         where the assert_param error has occurred.
  * @param  file: pointer to the source file name
  * @param  line: assert_param error line source number
  * @retval None
  */
void assert_failed(uint8_t *file, uint32_t line)
{
  /* USER CODE BEGIN 6 */
  /* User can add his own implementation to report the file name and line number,
     ex: printf("Wrong parameters value: file %s on line %d\r\n", file, line) */
  /* USER CODE END 6 */
}
#endif /* USE_FULL_ASSERT */
