# TrainPilot-Client

:bullettrain_side: Open a New Era of Train Autopilot.

# 快速上手
## 系统

- ubuntu 系统

## 依赖库
下面的依赖库必不可少
- protobuf 3.6.1
- boost 1.66
- opencv 3.4.0
- protobuf 3.6.1
- QT 5.10.0 
- VTK 8.1.0
- PCL 1.8.1
## 快速编译

### clone源码(git方式)
    git clone git@192.168.100.11:AutoTrain/AutoTrainDevelop/trainpilot-client.git
    cd trainpilot-client 

 > git方式无需输入用户名密码，推荐使用。
 >参考[生成ssh key并添加到gitlab](http://192.168.100.11:8090/AutoTrain/AutoTrainDevelop/trainpilot/blob/master/README.md)


###  clone源码(http方式)
    git clone http://192.168.100.11:8090/AutoTrain/AutoTrainDevelop/trainpilot-client
    cloning 'trainpilot'...
    Username for 'http://192.168.100.11:8090': abc@watrix.ai
    Password for 'http://abc@watrix.ai@192.168.100.11:8090': 
    .......

> http方式每次pull,push需输入用户名密码，不推荐使用。

### proto文件

>在根目录下执行 ./run_protobuf.sh，在src/proto目录下生成的    
    camera_image.pb.cc
    camera_image.pb.h
    point_cloud.pb.cc
    point_cloud.pb.h
    adapter_config.pb.h
    ###注意：如果你系统使用protobuf不是3.6.1，请使用protoc自行生成保持和trainpilot项目中的protobuf文件一致性

### QT设置

sudo gedit  /opt/Qt5.10.1/5.10.1/gcc_64/lib/cmake/Qt5Gui/Qt5Gui_QGtk3ThemePlugin.cmake
全部注释掉
    
### 编译

    mkdir build && cd build
    cmake-gui ..
    cmake ..
    make -j4

## 启动说明
### 第一步
    在trainpilot-client/bin目录下
    ./trainpilot-client
    启动后设置IP地址，默认为本机测试使用
    点击connect按钮开始监听
### 第二步
- trainpilot端配置
    /trainpilot/cfg/nodes/config/node_config.pb.txt
    跟据硬件环境选择simulation还是真实硬件设备，simulation:1 为仿真模式 0为连接物理硬件
    camera: {
    simulation: 1
    camera_name: "left"
    image_folder: "../data/simulation/camera/"
    }
    lidar: {
    simulation: 1
    save_config:{
        save2disk:  0
        save_folder: "./pcl/"
    }
    ipconfig:{
        ip: "192.168.1.201"
        port: 2368
    }
    pcd_folder: "../data/pcd/"
    }

- trainpilot端程序启动     
    在trainpilot/bin目录下，依次启动如下节点：
    ./node_perception
    ./node_sync
    ./node_client_sender
    ./node_camera_publisher
    ./node_lidar_publisher
    
## 界面功能说明
>如图           软件界面
![png](data/mainPage.png)
 
    VIDEO_1 原始图像
    VIDEO_2 分割的原始图像
    VIDEO_3 YOLO图像
    VIDEO_4 分割的二进制图像
    VIDEO_5 未使用
    VIDEO_1下方为点云显示窗口
    左侧都是点云的相关控制
>---------------------------
# :warning: git协同开发规范 :warning:
 >参见[git协同开发规范](http://192.168.100.11:8090/AutoTrain/AutoTrainDevelop/trainpilot/blob/master/README.md)

