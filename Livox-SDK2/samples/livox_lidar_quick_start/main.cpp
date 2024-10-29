//
// The MIT License (MIT)
//
// Copyright (c) 2022 Livox. All rights reserved.
//
// Permission is hereby granted, free of charge, to any person obtaining a copy
// of this software and associated documentation files (the "Software"), to deal
// in the Software without restriction, including without limitation the rights
// to use, copy, modify, merge, publish, distribute, sublicense, and/or sell
// copies of the Software, and to permit persons to whom the Software is
// furnished to do so, subject to the following conditions:
//
// The above copyright notice and this permission notice shall be included in
// all copies or substantial portions of the Software.
//
// THE SOFTWARE IS PROVIDED "AS IS", WITHOUT WARRANTY OF ANY KIND, EXPRESS OR
// IMPLIED, INCLUDING BUT NOT LIMITED TO THE WARRANTIES OF MERCHANTABILITY,
// FITNESS FOR A PARTICULAR PURPOSE AND NONINFRINGEMENT. IN NO EVENT SHALL THE
// AUTHORS OR COPYRIGHT HOLDERS BE LIABLE FOR ANY CLAIM, DAMAGES OR OTHER
// LIABILITY, WHETHER IN AN ACTION OF CONTRACT, TORT OR OTHERWISE, ARISING FROM,
// OUT OF OR IN CONNECTION WITH THE SOFTWARE OR THE USE OR OTHER DEALINGS IN THE
// SOFTWARE.
//

#include "livox_lidar_def.h"
#include "livox_lidar_api.h"

#ifdef _WIN32
#include <winsock2.h>
#else
#include <unistd.h>
#include <arpa/inet.h>
#endif

#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include <thread>
#include <chrono>
#include <iostream>
#include <fstream>
#include <ctime>
#include <sstream>

bool isFileEmpty(const std::string& filename) {
    std::ifstream file(filename, std::ios::ate); // 開啟檔案並移動到檔案末尾
    return file.tellg() == 0; // 如果檔案大小為 0，則檔案為空
}

std::string generateFilename() {

    std::time_t now = std::time(nullptr);
    std::tm* now_tm = std::localtime(&now);
    
    // 文件名格式：YYYY-MM-DD_HH-MM.csv
    std::ostringstream filename;
    filename << (now_tm->tm_year + 1900) << "-"         // 年
             << (now_tm->tm_mon + 1) << "-"             // 月
             << now_tm->tm_mday << "_"                  // 日
             << now_tm->tm_hour << "-"                   // 时
             << now_tm->tm_min << ".csv";               // 分

    return filename.str();
}

std::ofstream openCsvFile() {
    std::string filename = generateFilename();  // 获取生成的文件名
    std::ifstream file(filename);

    std::ofstream csv_file(filename, std::ios::app);  // 打开文件

    if (isFileEmpty(filename)) {
        csv_file << "index,timestamp,x,y,z,reflectivity,r,theta,phi\n";  // 写入表头
    }

    if (!csv_file.is_open()) {
        std::cerr << "Unable to open file for writing: " << filename << std::endl;
    }
    return csv_file;
}

void SaveAsCsv(LivoxLidarCartesianHighRawPoint *p_point_data, int dot_num, uint64_t timestamp_64){
  
  std::ofstream csv_file = openCsvFile();

  for (int i = 0; i < dot_num; i++) {
      int index = i;
      int32_t x = p_point_data[i].x;
      int32_t y = p_point_data[i].y;
      int32_t z = p_point_data[i].z;
      uint32_t reflectivity = p_point_data[i].reflectivity;

      double r = sqrt(x*x + y*y + z*z);
      double theta = 0.0, phi = 0.0;
      if (r != 0) {
          theta = acos(z / r);  // 仰角
          phi = atan2(y, x);    // 方位角
      }

      printf("tag: %d Point %d: x: %u, y: %u, z: %u, reflectivity: %u,r: %f,theta: %f, phi: %f\n",p_point_data->tag, i, x, y, z, reflectivity,r,theta,phi);
      csv_file << index << "," << timestamp_64 << "," << x << "," << y << "," << z << "," << reflectivity << "," << r << "," << theta << "," << phi <<"\n";
  }
  csv_file.close();  // 完成后关闭文件
}

void PointCloudCallback(uint32_t handle, const uint8_t dev_type, LivoxLidarEthernetPacket* data, void* client_data) {
  if (data == nullptr) {
    return;
  }

  if (data->data_type == kLivoxLidarCartesianCoordinateHighData) {  
    LivoxLidarCartesianHighRawPoint *p_point_data = (LivoxLidarCartesianHighRawPoint *)data->data;
    
    u_int64 timestamp_64 = 0;
    for(int i = 0; i < 8; i++){
      timestamp_64 |= static_cast<uint64_t>(data->timestamp[i]) << ((i) * 8); //先轉成 uint64_t 再左移會比較準確及安全
      printf("%d ", data->timestamp[i]);
    }
    
    printf("\nPointCloud_timestamp: %llu\n", timestamp_64);

    SaveAsCsv(p_point_data, data->dot_num, timestamp_64);

  }
  else if (data->data_type == kLivoxLidarCartesianCoordinateLowData) {
    LivoxLidarCartesianLowRawPoint *p_point_data = (LivoxLidarCartesianLowRawPoint *)data->data;
    for (uint32_t i = 0; i < data->dot_num; i++) {
      // 处理低精度点
      printf("Point %d: x: %u, y: %u, z: %u\n", i, p_point_data[i].x, p_point_data[i].y, p_point_data[i].z);
    }
  } else if (data->data_type == kLivoxLidarSphericalCoordinateData) {
    LivoxLidarSpherPoint* p_point_data = (LivoxLidarSpherPoint *)data->data;
    for (uint32_t i = 0; i < data->dot_num; i++) {
      // 处理球坐标点
      printf("Point %d: depth: %u, theta: %u, phi: %u\n", i, p_point_data[i].depth, p_point_data[i].theta, p_point_data[i].phi);
    }
  }
}

void ImuDataCallback(uint32_t handle, const uint8_t dev_type,  LivoxLidarEthernetPacket* data, void* client_data) {
  if (data == nullptr) {
    return;
  } 
  if (data->data_type == kLivoxLidarImuData) {    //確認是否為imu數據
    LivoxLidarImuRawPoint *imu_point_data = (LivoxLidarImuRawPoint *)data->data;
     u_int64 timestamp_64 = 0;
    for(int i = 0; i < 8; i++){
      timestamp_64 |= static_cast<uint64_t>(data->timestamp[i]) << (i * 8);
    }
    // printf("IMU_timestamp: %llu\n", timestamp_64);
    // // printf("IMU_time_interval: %d * 0.1 us\n", data->time_interval);

    // for (uint32_t i = 0; i < data->dot_num; i++) {
    //   printf("Imu Point %d: gyro_x: %f, gyro_y: %f, gyro_z: %f, acc_x: %f, acc_y: %f, acc_z: %f\n", i, 
    //     imu_point_data[i].gyro_x, imu_point_data[i].gyro_y, imu_point_data[i].gyro_z, 
    //     imu_point_data[i].acc_x, imu_point_data[i].acc_y, imu_point_data[i].acc_z);
    // }
  }
}

// void OnLidarSetIpCallback(livox_vehicle_status status, uint32_t handle, uint8_t ret_code, void*) {
//   if (status == kVehicleStatusSuccess) {
//     printf("lidar set ip slot: %d, ret_code: %d\n",
//       slot, ret_code);
//   } else if (status == kVehicleStatusTimeout) {
//     printf("lidar set ip number timeout\n");
//   }
// }

void WorkModeCallback(livox_status status, uint32_t handle,LivoxLidarAsyncControlResponse *response, void *client_data) {
  if (response == nullptr) {
    return;
  }
  printf("WorkModeCallack, status:%u, handle:%u, ret_code:%u, error_key:%u",
      status, handle, response->ret_code, response->error_key);

}

void RebootCallback(livox_status status, uint32_t handle, LivoxLidarRebootResponse* response, void* client_data) {
  if (response == nullptr) {
    return;
  }
  printf("RebootCallback, status:%u, handle:%u, ret_code:%u",
      status, handle, response->ret_code);
}

void SetIpInfoCallback(livox_status status, uint32_t handle, LivoxLidarAsyncControlResponse *response, void *client_data) {
  if (response == nullptr) {
    return;
  }
  printf("LivoxLidarIpInfoCallback, status:%u, handle:%u, ret_code:%u, error_key:%u",
      status, handle, response->ret_code, response->error_key);

  if (response->ret_code == 0 && response->error_key == 0) {
    LivoxLidarRequestReboot(handle, RebootCallback, nullptr);
  }
}

void QueryInternalInfoCallback(livox_status status, uint32_t handle, 
    LivoxLidarDiagInternalInfoResponse* response, void* client_data) {
  if (status != kLivoxLidarStatusSuccess) {
    printf("Query lidar internal info failed.\n");
    QueryLivoxLidarInternalInfo(handle, QueryInternalInfoCallback, nullptr);
    return;
  }

  if (response == nullptr) {
    return;
  }

  uint8_t host_point_ipaddr[4] {0};
  uint16_t host_point_port = 0;
  uint16_t lidar_point_port = 0;

  uint8_t host_imu_ipaddr[4] {0};
  uint16_t host_imu_data_port = 0;
  uint16_t lidar_imu_data_port = 0;

  uint16_t off = 0;
  for (uint8_t i = 0; i < response->param_num; ++i) {
    LivoxLidarKeyValueParam* kv = (LivoxLidarKeyValueParam*)&response->data[off];
    if (kv->key == kKeyLidarPointDataHostIpCfg) {
      memcpy(host_point_ipaddr, &(kv->value[0]), sizeof(uint8_t) * 4);
      memcpy(&(host_point_port), &(kv->value[4]), sizeof(uint16_t));
      memcpy(&(lidar_point_port), &(kv->value[6]), sizeof(uint16_t));
    } else if (kv->key == kKeyLidarImuHostIpCfg) {
      memcpy(host_imu_ipaddr, &(kv->value[0]), sizeof(uint8_t) * 4);
      memcpy(&(host_imu_data_port), &(kv->value[4]), sizeof(uint16_t));
      memcpy(&(lidar_imu_data_port), &(kv->value[6]), sizeof(uint16_t));
    }
    off += sizeof(uint16_t) * 2;
    off += kv->length;
  }

  printf("Host point cloud ip addr:%u.%u.%u.%u, host point cloud port:%u, lidar point cloud port:%u.\n",
      host_point_ipaddr[0], host_point_ipaddr[1], host_point_ipaddr[2], host_point_ipaddr[3], host_point_port, lidar_point_port);

  printf("Host imu ip addr:%u.%u.%u.%u, host imu port:%u, lidar imu port:%u.\n",
    host_imu_ipaddr[0], host_imu_ipaddr[1], host_imu_ipaddr[2], host_imu_ipaddr[3], host_imu_data_port, lidar_imu_data_port);

}

void LidarInfoChangeCallback(const uint32_t handle, const LivoxLidarInfo* info, void* client_data) {
  if (info == nullptr) {
    printf("lidar info change callback failed, the info is nullptr.\n");
    return;
  } 
  printf("LidarInfoChangeCallback Lidar handle: %u SN: %s\n", handle, info->sn);
  
  // set the work mode to kLivoxLidarNormal, namely start the lidar
  SetLivoxLidarWorkMode(handle, kLivoxLidarNormal, WorkModeCallback, nullptr);

  QueryLivoxLidarInternalInfo(handle, QueryInternalInfoCallback, nullptr);

  // LivoxLidarIpInfo lidar_ip_info;
  // strcpy(lidar_ip_info.ip_addr, "192.168.1.10");
  // strcpy(lidar_ip_info.net_mask, "255.255.255.0");
  // strcpy(lidar_ip_info.gw_addr, "192.168.1.1");
  // SetLivoxLidarLidarIp(handle, &lidar_ip_info, SetIpInfoCallback, nullptr);
}

void LivoxLidarPushMsgCallback(const uint32_t handle, const uint8_t dev_type, const char* info, void* client_data) {
  struct in_addr tmp_addr;
  tmp_addr.s_addr = handle;  
  std::cout << "handle: " << handle << ", ip: " << inet_ntoa(tmp_addr) << ", push msg info: " << std::endl;
  std::cout << info << std::endl;
  return;
}




int main(int argc, const char *argv[]) {
  if (argc != 2) {
    printf("Params Invalid, must input config path.\n");
    return -1;
  }
  const std::string path = argv[1];
  printf("Livox Quick Start Demo Begin!\n");
  // REQUIRED, to init Livox SDK2
  if (!LivoxLidarSdkInit(path.c_str())) {
    printf("Livox Init Failed\n");
    LivoxLidarSdkUninit();
    return -1;
  }


  // REQUIRED, to get point cloud data via 'PointCloudCallback'
  SetLivoxLidarPointCloudCallBack(PointCloudCallback, nullptr);
  
  // OPTIONAL, to get imu data via 'ImuDataCallback'
  // some lidar types DO NOT contain an imu component
  // SetLivoxLidarImuDataCallback(ImuDataCallback, nullptr);
  
  SetLivoxLidarInfoCallback(LivoxLidarPushMsgCallback, nullptr);
  
  // REQUIRED, to get a handle to targeted lidar and set its work mode to NORMAL
  SetLivoxLidarInfoChangeCallback(LidarInfoChangeCallback, nullptr);

  
#ifdef WIN32
  Sleep(300000);
#else
  sleep(300);
#endif
  LivoxLidarSdkUninit();
  printf("Livox Quick Start Demo End!\n");
  return 0;
}
