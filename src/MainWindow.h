#pragma once

#include "../build/src/ui_mainwindow.h"
#include "./include/net_packet.h"
// QT
#include <QMainWindow>
#include <QTcpServer>
#include <QTcpSocket>
#include <QUdpSocket>
#include <QFileDialog>
#include <QImageReader>
// Opencv
#include <opencv2/core/core.hpp>
#include <opencv2/highgui/highgui.hpp>
#include <opencv2/imgproc/imgproc.hpp>
// Point Cloud Library
#include <pcl/point_cloud.h>
#include <pcl/point_types.h>
#include <pcl/io/ply_io.h>
#include <pcl/io/pcd_io.h>          //I/O操作头文件
#include <pcl/point_types.h>        //点类型定义头文件
#define BOOST_TYPEOF_EMULATION  //要加在#include <pcl/registration/icp.h>前
#include <pcl/registration/icp.h>   //ICP配准类相关头文件
#include <pcl/visualization/cloud_viewer.h>//点云查看窗口头文件
#include <pcl/visualization/pcl_visualizer.h>//可视化头文
#include <pcl/filters/filter.h>


using namespace watrix::proto;

//typedef pcl::PointXYZRGBA PointT;
//typedef pcl::PointXYZRGB PointT;
typedef pcl::PointXYZRGB PointT;
typedef pcl::PointCloud<PointT> PointCloudT;

typedef pcl::PointXYZ PointT_icp;
typedef pcl::PointCloud<PointT_icp> PointCloudT_icp;



typedef struct
{
    float x_s;
    float x_e;
    float y_s;
    float y_e;
    float z_s;
    float z_e;
}CUBE_T;

namespace Ui {
class MainWindow;
}

class MainWindow : public QMainWindow
{
    Q_OBJECT

public:
    explicit MainWindow(QWidget *parent = 0);
    ~MainWindow();

private:
    Ui::MainWindow *ui;
    QTimer *m_pTimer;  

public:
    QImage cvMat2QImage(const cv::Mat& mat);
    long GetMillisec();
public slots: 
    void camera_move_forward();    
private:
    uint32_t transfer_time=0;//计算发送端，到接收到的时间
    QTcpServer *server;//创建TCP的服务器对象
    QTcpSocket *socket;//创建TCP的客户端对象
    //网络数据包相关结构
    NetPacketHeader *packet_head_;//收到的数据包的包头
    uint32_t img_size_;//信息的实际有效长度
    char * get_info_buf_;//有效信息的buffer存储    
    uint32_t packet_size_;
    uint32_t curr_posize_;//当前收到的有效信息的位置 （一个有效信息可能由几个数据包组成）
    uint8_t  packet_type_;//信息的类型 POINT_CLOUD  CAMERA_IMAGE  TRAIN_SEG_RESULT  YOLO_DETECTION_RESULT

    CUBE_T   zone_cube_;//点云选择的立方体区域
    void lidar_resault_display(char *get_img_buf , uint32_t img_size_);
    int comp3dicp();
    int vdk_head();
    //void KeyboardEventOccurred(const pcl::visualization::KeyboardEvent& event, void* nothing);
    void print4x4Matrix(const Eigen::Matrix4d & matrix);
    uint32_t Gray2Color(uint8_t val);

private slots:
    void server_New_Connect();//服务器接收到客户端消息信号，触发连接槽函数（连接客户端）
    //void socket_Disconnect();//断开服务器与客户端连接槽函数
    //void socket_Read_Data();//服务器读客户端传输过来的数据 

    //void acceptConnection();
////////////////////////////////
    //void on_pushButton_connect_clicked();
    void on_pushButton_LoadPCD_clicked();
    void on_pushButton_set_camera_clicked();
    void on_pushButton_setZone_clicked();

protected:
    /** @brief The PCL visualizer object */
    boost::shared_ptr<pcl::visualization::PCLVisualizer> viewer_;
    /** @brief The point cloud displayed */
    PointCloudT::Ptr cloud_;
    /** @brief 0 = x | 1 = y | 2 = z */
    int filtering_axis_;
    /** @brief Holds the color mode for @ref colorCloudDistances */
    int color_mode_;
    /** @brief Color point cloud on X,Y or Z axis using a Look-Up Table (LUT)
     * Computes a LUT and color the cloud accordingly, available color palettes are :
     *
     *  Values are on a scale from 0 to 255:
     *  0. Blue (= 0) -> Red (= 255), this is the default value
     *  1. Green (= 0) -> Magenta (= 255)
     *  2. White (= 0) -> Red (= 255)
     *  3. Grey (< 128) / Red (> 128)
     *  4. Blue -> Green -> Red (~ rainbow)
     *
     * @warning If there's an outlier in the data the color may seem uniform because of this outlier!
     * @note A boost rounding exception error will be thrown if used with a non dense point cloud
     */
    void colorCloudDistances ();
};