/**
 * @file fdilink_data_struct.h
 * @author FDILink
 * @brief 定义了FDILink IMU设备数据帧的各种结构体。
 * @version 1.0
 * @date 2024-07-29
 * 
 * @copyright Copyright (c) 2024
 * 
 */
#ifndef FDILINK_DATA_STRUCT_H_
#define FDILINK_DATA_STRUCT_H_

#include <iostream>
namespace FDILink{

/**
 * @brief FDILink数据帧的头部结构体。
 */
#pragma pack(1)
struct fdilink_header
{
	uint8_t  header_start;   /**< 帧头起始字节。 */
	uint8_t  data_type;      /**< 数据类型。 */
	uint8_t  data_size;      /**< 数据载荷大小。 */
	uint8_t  serial_num;     /**< 序列号。 */
	uint8_t  header_crc8;    /**< 头部CRC8校验和。 */
    uint8_t  header_crc16_h; /**< 头部CRC16校验和高字节。 */
	uint8_t  header_crc16_l; /**< 头部CRC16校验和低字节。 */
};
#pragma pack()

/**
 * @brief IMU数据包结构体。
 */
#pragma pack(1)
struct IMUData_Packet_t
{
	float gyroscope_x;          /**< 陀螺仪X轴角速度，单位: rad/s。 */
	float gyroscope_y;          /**< 陀螺仪Y轴角速度，单位: rad/s。 */
	float gyroscope_z;          /**< 陀螺仪Z轴角速度，单位: rad/s。 */
	float accelerometer_x;      /**< 加速度计X轴加速度，单位: m/s^2。 */
	float accelerometer_y;      /**< 加速度计Y轴加速度，单位: m/s^2。 */
	float accelerometer_z;      /**< 加速度计Z轴加速度，单位: m/s^2。 */
	float magnetometer_x;       /**< 磁力计X轴数据，单位: mG。 */
	float magnetometer_y;       /**< 磁力计Y轴数据，单位: mG。 */
	float magnetometer_z;       /**< 磁力计Z轴数据，单位: mG。 */
	float imu_temperature;      /**< IMU温度，单位: 摄氏度。 */
	float Pressure;             /**< 气压，单位: Pa。 */
	float pressure_temperature; /**< 气压传感器温度，单位: 摄氏度。 */
	int64_t Timestamp;          /**< 时间戳，单位: us。 */
};
#pragma pack()

/**
 * @brief AHRS数据包结构体。
 */
struct AHRSData_Packet_t
{
	float RollSpeed;   /**< 翻滚角速度，单位: rad/s。 */
	float PitchSpeed;  /**< 俯仰角速度，单位: rad/s。 */
	float HeadingSpeed;/**< 航向角速度，单位: rad/s。 */
	float Roll;        /**< 翻滚角，单位: rad。 */
	float Pitch;       /**< 俯仰角，单位: rad。 */
	float Heading;     /**< 航向角，单位: rad。 */
	float Qw;          /**< 四元数w分量。 */
	float Qx;          /**< 四元数x分量。 */
	float Qy;          /**< 四元数y分量。 */
	float Qz;          /**< 四元数z分量。 */
	int64_t Timestamp; /**< 时间戳，单位: us。 */
};
#pragma pack(4)

/**
 * @brief INSGPS数据包结构体。
 */
struct INSGPSData_Packet_t
{
	float BodyVelocity_X;       /**< 载体X轴速度。 */
	float BodyVelocity_Y;       /**< 载体Y轴速度。 */
	float BodyVelocity_Z;       /**< 载体Z轴速度。 */
	float BodyAcceleration_X;   /**< 载体X轴加速度。 */
	float BodyAcceleration_Y;   /**< 载体Y轴加速度。 */
	float BodyAcceleration_Z;   /**< 载体Z轴加速度。 */
	double Location_North;      /**< 北向位置。 */
	double Location_East;       /**< 东向位置。 */
	double Location_Down;       /**< 天向位置。 */
	float Velocity_North;       /**< 北向速度。 */
	float Velocity_East;        /**< 东向速度。 */
	float Velocity_Down;        /**< 天向速度。 */
	float Acceleration_North;   /**< 北向加速度。 */
	float Acceleration_East;    /**< 东向加速度。 */
	float Acceleration_Down;    /**< 天向加速度。 */
	float Pressure_Altitude;    /**< 气压高度。 */
	int64_t Timestamp;          /**< 时间戳，单位: us。 */
};
#pragma pack()

// IMU数据相关结构体。
#pragma pack(1)
/**
 * @brief 用于读取IMU数据帧的结构体。
 */
struct read_imu_struct{
  fdilink_header     header;    /**< 帧头。 */
  union data
  {
	IMUData_Packet_t   data_pack; /**< IMU数据包。 */
	uint8_t            data_buff[56]; /**< 原始数据缓冲区。 */
  }data;
  uint8_t            frame_end; /**< 帧尾。 */
};

/**
 * @brief IMU数据临时读取结构体。
 */
struct read_imu_tmp{
  uint8_t frame_header[7]; /**< 帧头缓冲区。 */
  uint8_t read_msg[57];    /**< 消息缓冲区。 */
};

/**
 * @brief IMU数据帧联合体，方便不同方式访问。
 */
union imu_frame_read{
  struct read_imu_struct frame; /**< 结构体方式访问。 */
  read_imu_tmp read_buf;        /**< 临时缓冲区方式访问。 */
  uint8_t read_tmp[64];         /**< 原始字节方式访问。 */
};
#pragma pack()

// AHRS数据相关结构体。
#pragma pack(1)
/**
 * @brief 用于读取AHRS数据帧的结构体。
 */
struct read_ahrs_struct{
  fdilink_header     header;    /**< 帧头。 */
  union data
  {
	AHRSData_Packet_t  data_pack; /**< AHRS数据包。 */
	uint8_t            data_buff[48]; /**< 原始数据缓冲区。 */
  }data;
  uint8_t            frame_end; /**< 帧尾。 */
};

/**
 * @brief AHRS数据临时读取结构体。
 */
struct read_ahrs_tmp{
  uint8_t frame_header[7]; /**< 帧头缓冲区。 */
  uint8_t read_msg[49];    /**< 消息缓冲区。 */
};

/**
 * @brief AHRS数据帧联合体，方便不同方式访问。
 */
union ahrs_frame_read{
  struct read_ahrs_struct frame; /**< 结构体方式访问。 */
  read_ahrs_tmp read_buf;        /**< 临时缓冲区方式访问。 */
  uint8_t read_tmp[56];          /**< 原始字节方式访问。 */
};
#pragma pack()

// INSGPS数据相关结构体。
#pragma pack(1)
/**
 * @brief 用于读取INSGPS数据帧的结构体。
 */
struct read_insgps_struct{
  fdilink_header     header;    /**< 帧头。 */
  union data
  {
	INSGPSData_Packet_t  data_pack; /**< INSGPS数据包。 */
	uint8_t              data_buff[84]; /**< 原始数据缓冲区。 */
  }data;
  uint8_t            frame_end; /**< 帧尾。 */
};

/**
 * @brief INSGPS数据临时读取结构体。
 */
struct read_insgps_tmp{
  uint8_t frame_header[7]; /**< 帧头缓冲区。 */
  uint8_t read_msg[85];    /**< 消息缓冲区。 */
};

/**
 * @brief INSGPS数据帧联合体，方便不同方式访问。
 */
union insgps_frame_read{
  struct read_insgps_struct frame; /**< 结构体方式访问。 */
  read_insgps_tmp read_buf;        /**< 临时缓冲区方式访问。 */
  uint8_t read_tmp[92];          /**< 原始字节方式访问。 */
};


}//namespace FDILink
#endif//FDILINK_DATA_STRUCT_H_