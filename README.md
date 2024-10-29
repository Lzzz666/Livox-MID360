# Livox-MID360

# 1. Introduction

Livox SDK2 is a software development kit designed for all Livox lidars such as HAP and Mid-360. It is developed based on C/C++ following Livox SDK2 Communication Protocol, and provides easy-to-use C style APIs. With the Livox SDK2, users can quickly connect to the Livox Lidars and receive point cloud data.

Livox SDK2 consists of [Livox SDK2 core code](sdk_core/), [Livox SDK2 APIs](include/livox_lidar_api.h) and three [samples](samples/).

## Livox SDK2 API

Livox SDK2 API provides a set of C-style APIs, which can be conveniently integrated in C/C++ programs. Please refer to the **[Livox SDK2 APIs](include/livox_lidar_api.h)**.

## Livox SDK2 Communication Protocol

Livox SDK2 communication protocol opens to all users. It is the communication protocol between user programs and livox products. The protocol consists of control commands and data format, please refer to the documents below:

**Mid-360**:

* [Mid-360 Communication protocol](https://livox-wiki-cn.readthedocs.io/zh_CN/latest/tutorials/new_product/mid360/mid360.html) (中文)
* [Mid-360 Communication protocol](https://livox-wiki-en.readthedocs.io/en/latest/tutorials/new_product/mid360/mid360.html) (English)

# 2. Installation

## 2.1 Prerequisites

* OS:
  * Windows 10 / 11

* Tools:
  * compilers that support C++11
  * cmake 3.0+

* Arch:
  * x86
  * ARM

## 2.2 Instruction for Windows 10

1. Dependencies:

* Visual Studio 2019
* [CMake 3.0.0+](https://cmake.org/)

2. Preparation:

```cmd
> git clone https://github.com/Livox-SDK/Livox-SDK2.git
> cd Livox-SDK2
> md build && cd build
```

3. Generate a project
* 64-bit project:

```cmd
> cmake .. -G "Visual Studio 16 2019" -A x64
```

4. Compiling:

You can now compile the Livox-SDK2 in Visual Studio 2019. (or use extension in VSCode)


# 3. Run the Samples

Livox SDK2 includes three samples, which are "livox_lidar_quick_start", "logger" and "multi_lidars_upgrade".

## 3.1 Livox lidar quick start sample

### In Windows 10
After compiling the Livox SDK2 as shown in Installation above, you can find '**.\livox_lidar_quick_start.exe**' in the directory of '**Livox-SDK2\\build\\samples\\livox_lidar_quick_start\\Debug(or Release)\\**'.

Copy the config file '**Livox-SDK2\\samples\\livox_lidar_quick_start\\[config file]**' into the directory containing the program '**livox_lidar_quick_start.exe**', and run:

```cmd
> .\livox_lidar_quick_start.exe .\config.json 
```

Then you can see the information as below:

```shell
> [info] Data Handle Init Succ.  [data_handler.cpp] [Init] [42]
> [info] Create detection channel detection socket:0  [device_manager.cpp] [CreateDetectionChannel] [232]
```
notice: You must insert the etherNet line into your PC  
**Note** : 
1. [config file] in the command above represents the config file name, you can choose different config file depends on your needs.  
example: (you need to set your host_ip and check the lidar_ip)
```
   {
  "MID360": {
    "lidar_net_info" : {
      "cmd_data_port"  : 56100,
      "push_msg_port"  : 56200,
      "point_data_port": 56300,
      "imu_data_port"  : 56400,
      "log_data_port"  : 56500
    },
    "host_net_info" : [
      {
        "lidar_ip"       : ["192.168.1.1XX"],
        "host_ip"        : "192.168.1.50",
        "cmd_data_port"  : 56101,
        "push_msg_port"  : 56201,
        "point_data_port": 56301,
        "imu_data_port"  : 58401,
        "log_data_port"  : 59501
      }
    ]
  }
}
```

# 4. Get Raw Data (CloudPoint, IMUPoint, Time Stamp)
in /samples/livox_lidar_quick_start/main.cpp
### get CloudPoint data
![image](https://github.com/user-attachments/assets/d594b7f2-4cd9-4746-b316-766955b82e93)

### get IMU data
![image](https://github.com/user-attachments/assets/5b457158-1e14-420c-9718-b878a412a5ca)



# 5. Download as a .csv document


# 6. data
in /include/livox_lidar_def.h
```
typedef struct {
  uint8_t version;
  uint16_t length;
  uint16_t time_interval;     // 時間間隔，單位是 0.1 微秒（us），這可能表示該數據包中的點之間的時間間隔。
  uint16_t dot_num;           // 點雲數據的點數量，表示該數據包中包含了多少個點。
  uint16_t udp_cnt;           // UDP 數據包計數器，用於追蹤或標記這個數據包是該系列數據包中的第幾個。
  uint8_t frame_cnt;          // fps
  uint8_t data_type;
  uint8_t time_type;
  uint8_t rsvd[12];
  uint32_t crc32;             // 這是數據包的 CRC32 校驗碼，用來檢測數據在傳輸過程中是否損壞。
  uint8_t timestamp[8];       // 8 字節的時間戳 可能是 64 位的納秒或微秒精度
  uint8_t data[1];             /**< Point cloud data. */
} LivoxLidarEthernetPacket;
```
