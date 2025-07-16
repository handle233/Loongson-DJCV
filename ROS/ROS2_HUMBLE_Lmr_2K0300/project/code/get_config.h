
#ifndef _get_config_h_
#define _get_config_h_

#include "tcp_send_ladar.h"
#include "tcp_send_encoder.h"
#include "tcp_recv_control.h"

// IP地址 和 端口号
#define SERVER_IP "192.168.3.24" // 服务器IP地址

// #define SEND_LADAR_PORT     ( 8888 )
#define SEND_LADAR_PORT     ( 9999 )
#define SEND_IMU_PORT       ( 7776 )
#define SEND_ENCODER_PORT   ( 7774 )
#define SEND_SENSOR_PORT    ( 8889 )
#define RECV_CONTROL_PORT   ( 8890 )
#define USB_PATH  "/dev/ttyUSB0"
#define UART_PATH "/dev/ttyS1"
// #define UART_PATH "/dev/ttyUSB0"
#define UART_BAUD B230400

#define ENCODER_1           "/dev/zf_encoder_1"
#define ENCODER_2           "/dev/zf_encoder_2"

#define MOTOR1_DIR          "/dev/zf_driver_gpio_motor_1"
#define MOTOR1_PWM          "/dev/zf_device_pwm_motor_1"

#define MOTOR2_DIR          "/dev/zf_driver_gpio_motor_2"
#define MOTOR2_PWM          "/dev/zf_device_pwm_motor_2"

#define SERVO_MOTOR1_PWM            "/dev/zf_device_pwm_servo"

// 优先级1-99，值越大优先级越高
#define PIT_THD_PRO                 ( 10 )
#define TCP_SEND_LADAR_THD_PRO      ( 9  )
#define TCP_SEND_SENSOR_THD_PRO     ( 8  )
#define TCP_RECV_CONTROL_THD_PRO    ( 7  )



#define ENCODER_THD_PRO             ( 1  )
#define IMU_THD_PRO                 ( 1  )


extern pthread_mutex_t g_tcp_sensor_mutex;
// extern tcp_xfer_sensor_dat_TypeDef g_tcp_sensor;
extern car_control_typedef g_car_control;

#endif
