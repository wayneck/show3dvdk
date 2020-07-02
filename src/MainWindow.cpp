#include "MainWindow.h"
#include "./proto/camera_image.pb.h"
#include "./proto/point_cloud.pb.h"

#include <QMessageBox>
#include <QDebug>
#include <QBuffer>
#include <QString>
// Visualization Toolkit (VTK)
#include <vtkSmartPointer.h>
#include <vtkStructuredPointsReader.h>
#include <vtkRenderer.h>
#include <vtkRenderWindow.h>
#include <vtkRenderWindowInteractor.h>
#include <vtkMarchingCubes.h>
#include <vtkPolyDataMapper.h>
#include <vtkActor.h>
#include <vtkInteractorStyleTrackballCamera.h>
// Boost
#include <boost/math/special_functions/round.hpp>
#include <boost/asio.hpp>
// Std
#include <iostream>
#define RGBPARM 128
#define PACKAGE_USED 1
#define PACKAGE_UNUSED 0
#define PACKAGE_RECEIVE_BEGIN 0
#define PACKAGE_RECEIVE_ING 1
#define PACKAGE_RECEIVE_END 2

using namespace cv;
using namespace boost::asio;


bool next_iteration = false;
void static KeyboardEventOccurred(const pcl::visualization::KeyboardEvent& event, void* nothing)
{  //使用空格键来增加迭代次数，并更新显示
	if (event.getKeySym() == "space" && event.keyDown())
		next_iteration = true;
}


MainWindow::MainWindow(QWidget *parent) :
    QMainWindow(parent),
    ui(new Ui::MainWindow)
{
    ui->setupUi(this);
    /////////////////////////
    packet_head_ = new NetPacketHeader();
    server = new QTcpServer();//初始化QTcpServer对象
    //当服务器被客户端访问时，会触发newConnection信号，可以利用该信号作为触发信号，绑定槽函数server_New_Connect
    connect(server,&QTcpServer::newConnection,this,&MainWindow::server_New_Connect);//关联信号和槽函数
    ui->lineEdit_IP->setText("127.0.0.1");//设置默认IP
    //ui->lineEdit_Port->setText("6666");//设置默认端口号

    /////////////////////////////////////
    /////////////////////////////////////
    ////////////////////////////////////////
    // Setup the cloud pointer
    cloud_.reset (new PointCloudT);
    // The number of points in the cloud
    cloud_->resize (500);

    // Fill the cloud with random points
    for (size_t i = 0; i < cloud_->points.size (); ++i)
    {
      cloud_->points[i].x = 1024 * rand () / (RAND_MAX + 1.0f);
      cloud_->points[i].y = 1024 * rand () / (RAND_MAX + 1.0f);
      cloud_->points[i].z = 1024 * rand () / (RAND_MAX + 1.0f);
    }
    // Set up the QVTK window
    viewer_.reset (new pcl::visualization::PCLVisualizer ("viewer", false));
    viewer_->setBackgroundColor (0.1, 0.1, 0.1);
    ui->qvtkWidget->SetRenderWindow (viewer_->getRenderWindow ());
    viewer_->setupInteractor (ui->qvtkWidget->GetInteractor (), ui->qvtkWidget->GetRenderWindow ());
    ui->qvtkWidget->update ();
    // Color the randomly generated cloud
    colorCloudDistances ();
    viewer_->addPointCloud (cloud_, "cloud");
    viewer_->resetCamera ();
    ui->qvtkWidget->update ();


    // m_pTimer = new QTimer(this);  
    // connect(m_pTimer, SIGNAL(timeout()), this, SLOT(camera_move_forward()));  
    // m_pTimer->start(200);  
}

MainWindow::~MainWindow()
{
    server->close();
    server->deleteLater();
    delete ui;
}

void MainWindow::server_New_Connect()
{
    socket = server->nextPendingConnection();
    //QObject::connect(socket , &QTcpSocket::readyRead , this , &MainWindow::socket_Read_Data);
    //QObject::connect(socket , &QTcpSocket::disconnected , this , &MainWindow::socket_Disconnect);
}



QImage MainWindow::cvMat2QImage(const cv::Mat& mat)
{
    // 8-bits unsigned, NO. OF CHANNELS = 1
    if(mat.type() == CV_8UC1)
    {
        QImage image(mat.cols, mat.rows, QImage::Format_Indexed8);
        // Set the color table (used to translate colour indexes to qRgb values)
        image.setColorCount(256);
        for(int i = 0; i < 256; i++)
        {
            image.setColor(i, qRgb(i, i, i));
        }
        // Copy input Mat
        uchar *pSrc = mat.data;
        for(int row = 0; row < mat.rows; row ++)
        {
            uchar *pDest = image.scanLine(row);
            memcpy(pDest, pSrc, mat.cols);
            pSrc += mat.step;
        }
        return image;
    }
    // 8-bits unsigned, NO. OF CHANNELS = 3
    else if(mat.type() == CV_8UC3)
    {
        // Copy input Mat
        const uchar *pSrc = (const uchar*)mat.data;
        // Create QImage with same dimensions as input Mat
        QImage image(pSrc, mat.cols, mat.rows, mat.step, QImage::Format_RGB888);
        return image.rgbSwapped();
    }
    else if(mat.type() == CV_8UC4)
    {
        qDebug() << "CV_8UC4";
        // Copy input Mat
        const uchar *pSrc = (const uchar*)mat.data;
        // Create QImage with same dimensions as input Mat
        QImage image(pSrc, mat.cols, mat.rows, mat.step, QImage::Format_ARGB32);
        return image.copy();
    }
    else
    {
        qDebug() << "ERROR: Mat could not be converted to QImage.";
        return QImage();
    }
}


void MainWindow::lidar_resault_display(char *get_img_buf , uint32_t img_size_)
{
    watrix::proto::PointCloud pcl_data;
    pcl_data.ParseFromArray(get_img_buf , img_size_);

    PointCloudT::Ptr cloud (new PointCloudT);
    cloud->width    = pcl_data.points_size();
    cloud->height   = 1;
    cloud->is_dense = false;  //不是稠密型的
    cloud->points.resize (cloud->width * cloud->height);  //点云总数大小
    uint16_t e_point = 0;

    for (size_t i = 0; i < cloud->points.size(); ++i)
    {
      const watrix::proto::LidarPoint pt =  pcl_data.points(i);
      //set zone
        if((pt.x()>zone_cube_.x_s) & (pt.x()< zone_cube_.x_e)){
            if((pt.y()>zone_cube_.y_s) & (pt.y()< zone_cube_.y_e)){
                if((pt.z()>zone_cube_.z_s) && (pt.z()< zone_cube_.z_e)){
                    cloud->points[i].x = pt.x();
                    cloud->points[i].y = pt.y();
                    cloud->points[i].z = pt.z();
                    cloud->points[i].r = 0;//1024 * rand () / (RAND_MAX + 1.0f);
                    cloud->points[i].g = 255;//1024 * rand () / (RAND_MAX + 1.0f);
                    cloud->points[i].b = 0;//1024 * rand () / (RAND_MAX + 1.0f);
                    //cloud.points[e_point].intensity =pt.intensity();
                    e_point++;
                }
            }
        }else{
            //refresh null point
            cloud->points[i].x = pt.x();
            cloud->points[i].y = pt.y();
            cloud->points[i].z = pt.z();
            cloud->points[i].r = 255;//1024 * rand () / (RAND_MAX + 1.0f);
            cloud->points[i].g = 0;//1024 * rand () / (RAND_MAX + 1.0f);
            cloud->points[i].b = 1;//1024 * rand () / (RAND_MAX + 1.0f);
        }
    }
      pcl::copyPointCloud (*cloud, *cloud_);
     //  colorCloudDistances ();
     // viewer_->addPointCloud (cloud, "cloud");
      viewer_->updatePointCloud (cloud_, "cloud");
      ui->qvtkWidget->update ();
}


void MainWindow::on_pushButton_LoadPCD_clicked()
{
    // You might want to change "/home/" if you're not on an *nix platform
    QString filename = QFileDialog::getOpenFileName (this, tr ("Open point cloud"), "/home/wayne/data/", tr ("Point cloud data (*.pcd *.ply)"));
    PCL_INFO("File chosen: %s\n", filename.toStdString ().c_str ());
    PointCloudT::Ptr cloud_tmp (new PointCloudT);
    pcl::PointCloud<pcl::PointXYZI>::Ptr cloud_intensity (new pcl::PointCloud<pcl::PointXYZI>);
    if (filename.isEmpty ())
      return;

    int return_status;
    if (filename.endsWith (".pcd", Qt::CaseInsensitive))
      return_status = pcl::io::loadPCDFile (filename.toStdString (), *cloud_intensity);
    else
      return_status = pcl::io::loadPLYFile (filename.toStdString (), *cloud_intensity);

	  //cloud->points.resize(cloud_tmp->points.size()/3+1);
    qDebug()<<"cloud_tmp size= "<<cloud_intensity->points.size()<<endl;
    uint32_t rgb;
    uint8_t r = 0, g = 0, b = 0;    // Example: Red color
    uint32_t a0=0,a1=0,a2=0,a3=0,a4=0,a5=0;
    for (int i = 0; i < cloud_intensity->points.size(); i++)
    {
 
      rgb = Gray2Color(cloud_intensity->points[i].intensity); 
      pcl::PointXYZRGB  point_xyzi;
      point_xyzi.x = cloud_intensity->points[i].x; 
      point_xyzi.y = cloud_intensity->points[i].y;
      point_xyzi.z =cloud_intensity->points[i].z;

      point_xyzi.rgb =*reinterpret_cast<float*>(&rgb);//cloud_in->points[i].intensity;
      cloud_tmp->push_back(point_xyzi);

       //rgb = Gray2Color(cloud_tmp->points[i].intensity);
       //cloud_tmp->points[i].rgb = *reinterpret_cast<float*>(&rgb);   
    }
    qDebug()<<"a0= "<< a0<<"a1= "<< a1<<"a2= "<< a2<<"a3= "<< a3<<"a4= "<< a4<<" a5= "<<a5<<endl;
    if (return_status != 0)
    {
      PCL_ERROR("Error reading point cloud %s\n", filename.toStdString ().c_str ());
      return;
    }

    // If point cloud contains NaN values, remove them before updating the visualizer point cloud
    if (cloud_tmp->is_dense){
        qDebug()<<"cloud_tmp->is_dense"<<endl;
      pcl::copyPointCloud (*cloud_tmp, *cloud_);
    }
    else
    {
     qDebug()<<"Cloud is not dense"<<endl;
      PCL_WARN("Cloud is not dense! Non finite points will be removed\n");
      std::vector<int> vec;
      pcl::removeNaNFromPointCloud (*cloud_tmp, *cloud_, vec);
    }

    //colorCloudDistances ();
    viewer_->updatePointCloud (cloud_, "cloud");
    viewer_->resetCamera ();
    viewer_->setCameraPosition (0.673091, 0.382046, -21.2402 ,0.999942, -0.00602988, 0.00890303 ,0);

    ui->qvtkWidget->update ();
}

int MainWindow::vdk_head() {
    QString filename = QFileDialog::getOpenFileName (this, tr ("Open point cloud"), "/home/wayne/mycode/", tr ("Point cloud data (*)"));
    PCL_INFO("File chosen: %s\n", filename.toStdString ().c_str ());
    //读入Structured_Points类型的vtk文件。
    auto reader = vtkSmartPointer<vtkStructuredPointsReader>::New();
    //reader->SetFileName("./head.vtk");
    reader->SetFileName(filename.toStdString ().c_str ());

    // 用移动立方体法提取等值面。
    auto marchingCubes = vtkSmartPointer<vtkMarchingCubes>::New();
    marchingCubes->SetInputConnection(reader->GetOutputPort());
    marchingCubes->SetValue(0, 500);

    // 将生成的等值面数据进行Mapper
    auto mapper = vtkSmartPointer<vtkPolyDataMapper>::New();
    mapper->SetInputConnection(marchingCubes->GetOutputPort());

    // 把Mapper的输出送入渲染引擎进行显示
    auto actor = vtkSmartPointer<vtkActor>::New();
    actor->SetMapper(mapper);

    // 渲染
    auto renderer = vtkSmartPointer<vtkRenderer>::New();
    renderer->AddActor(actor);
    renderer->SetBackground(1.0, 1.0, 1.0);

    // 设置渲染窗口
    auto renWin = vtkSmartPointer<vtkRenderWindow>::New();
    renWin->AddRenderer(renderer);

    // 添加交互样式
    auto interactor = vtkSmartPointer<vtkRenderWindowInteractor>::New();
    interactor->SetRenderWindow(renWin);
    renWin->Render();
    auto style = vtkSmartPointer<vtkInteractorStyleTrackballCamera>::New();
    interactor->SetInteractorStyle(style);

    interactor->Initialize();
    interactor->Start();

    return 0;
}



void MainWindow::colorCloudDistances ()
{
  // Find the minimum and maximum values along the selected axis
  double min, max;
  // Set an initial value
  switch (filtering_axis_)
  {
    case 0:  // x
      min = cloud_->points[0].x;
      max = cloud_->points[0].x;
      break;
    case 1:  // y
      min = cloud_->points[0].y;
      max = cloud_->points[0].y;
      break;
    default:  // z
      min = cloud_->points[0].z;
      max = cloud_->points[0].z;
      break;
  }

  // Search for the minimum/maximum
  for (PointCloudT::iterator cloud_it = cloud_->begin (); cloud_it != cloud_->end (); ++cloud_it)
  {
    switch (filtering_axis_)
    {
      case 0:  // x
        if (min > cloud_it->x)
          min = cloud_it->x;

        if (max < cloud_it->x)
          max = cloud_it->x;
        break;
      case 1:  // y
        if (min > cloud_it->y)
          min = cloud_it->y;

        if (max < cloud_it->y)
          max = cloud_it->y;
        break;
      default:  // z
        if (min > cloud_it->z)
          min = cloud_it->z;

        if (max < cloud_it->z)
          max = cloud_it->z;
        break;
    }
  }

  // Compute LUT scaling to fit the full histogram spectrum
  double lut_scale = 255.0 / (max - min);  // max is 255, min is 0

  if (min == max)  // In case the cloud is flat on the chosen direction (x,y or z)
    lut_scale = 1.0;  // Avoid rounding error in boost

  for (PointCloudT::iterator cloud_it = cloud_->begin (); cloud_it != cloud_->end (); ++cloud_it)
  {
    int value;
    switch (filtering_axis_)
    {
      case 0:  // x
        value = boost::math::iround ( (cloud_it->x - min) * lut_scale);  // Round the number to the closest integer
        break;
      case 1:  // y
        value = boost::math::iround ( (cloud_it->y - min) * lut_scale);
        break;
      default:  // z
        value = boost::math::iround ( (cloud_it->z - min) * lut_scale);
        break;
    }

    // Apply color to the cloud
    switch (color_mode_)
    {
      case 0:
        // Blue (= min) -> Red (= max)
        cloud_it->r = value;
        cloud_it->g = 0;
        cloud_it->b = 255 - value;
        break;
      case 1:
        // Green (= min) -> Magenta (= max)
        cloud_it->r = value;
        cloud_it->g = 255 - value;
        cloud_it->b = value;
        break;
      case 2:
        // White (= min) -> Red (= max)
        cloud_it->r = 255;
        cloud_it->g = 255 - value;
        cloud_it->b = 255 - value;
        break;
      case 3:
        // Grey (< 128) / Red (> 128)
        if (value > 128)
        {
          cloud_it->r = 255;
          cloud_it->g = 0;
          cloud_it->b = 0;
        }
        else
        {
          cloud_it->r = 128;
          cloud_it->g = 128;
          cloud_it->b = 128;
        }
        break;
      default:
        // Blue -> Green -> Red (~ rainbow)
        cloud_it->r = value > 128 ? (value - 128) * 2 : 0;  // r[128] = 0, r[255] = 255
        cloud_it->g = value < 128 ? 2 * value : 255 - ( (value - 128) * 2);  // g[0] = 0, g[128] = 255, g[255] = 0
        cloud_it->b = value < 128 ? 255 - (2 * value) : 0;  // b[0] = 255, b[128] = 0
    }
  }

}

void MainWindow::on_pushButton_set_camera_clicked()
{
    float camera_x = ui->lineEdit_C_X->text().toFloat();
    float camera_y = ui->lineEdit_C_Y->text().toFloat();
    float camera_z = ui->lineEdit_C_Z->text().toFloat();
    float view_x = ui->lineEdit_V_X->text().toFloat();
    float view_y = ui->lineEdit_V_Y->text().toFloat();
    float view_z = ui->lineEdit_V_Z->text().toFloat();
    //(0 , 0 , 50 ,0 ,0 ,0)
    viewer_->setCameraPosition (camera_x,camera_y,camera_z,view_x,view_y,view_z,0);
       
    //setCameraParameters (const Camera &camera, int viewport=0)
//    vtkSmartPointer<vtkCamera> cam = renderer->GetActiveCamera ();
//    cam->SetPosition (pos_vec[0], pos_vec[1], pos_vec[2]);
//    cam->SetFocalPoint (focal_vec[0], focal_vec[1], focal_vec[2]);
//    cam->SetViewUp (up_vec[0], up_vec[1], up_vec[2]);
//    cam->SetUseHorizontalViewAngle (0);
//    cam->SetViewAngle (fovy);
//    cam->SetClippingRange (0.01, 1000.01);
    
    //comp3dicp();
    
    vdk_head();
}

void MainWindow::camera_move_forward()
{

 
    std::vector<pcl::visualization::Camera> cams;
    viewer_->getCameras (cams);
    if (cams.size() !=1){
      std::cout << "n cams not 1 exiting\n"; // for now in case ...
     return; 
    }
    pcl::visualization::Camera cam = cams[0];
    qDebug() << cam.pos[0] << " " 
               << cam.pos[1] << " " 
               << cam.pos[2] << " p\n" << cam.view[0] << " " 
               << cam.view[1] << " " 
               << cam.view[2] << " v\n";	  

 viewer_->setCameraPosition (cam.pos[0],cam.pos[1],(cam.pos[2]+1),cam.view[0],cam.view[1],cam.view[2],0);

}

void MainWindow::on_pushButton_setZone_clicked()
{

    std::vector<pcl::visualization::Camera> cams;
    viewer_->getCameras (cams);
    if (cams.size() !=1){
      std::cout << "n cams not 1 exiting\n"; // for now in case ...
     return; 
    }
    pcl::visualization::Camera cam = cams[0];
    qDebug() << cam.pos[0] << " " 
               << cam.pos[1] << " " 
               << cam.pos[2] << " p\n";	   
    qDebug() << cam.view[0] << " " 
               << cam.view[1] << " " 
               << cam.view[2] << " v\n";	  
}






void MainWindow::print4x4Matrix(const Eigen::Matrix4d & matrix)    //打印旋转矩阵和平移矩阵
{
	printf("Rotation matrix :\n");
	printf("    | %6.3f %6.3f %6.3f | \n", matrix(0, 0), matrix(0, 1), matrix(0, 2));
	printf("R = | %6.3f %6.3f %6.3f | \n", matrix(1, 0), matrix(1, 1), matrix(1, 2));
	printf("    | %6.3f %6.3f %6.3f | \n", matrix(2, 0), matrix(2, 1), matrix(2, 2));
	printf("Translation vector :\n");
	printf("t = < %6.3f, %6.3f, %6.3f >\n\n", matrix(0, 3), matrix(1, 3), matrix(2, 3));
}


int MainWindow::comp3dicp()
{
  // 创建点云指针
	PointCloudT_icp::Ptr cloud_in(new PointCloudT_icp);  // 原始点云
	PointCloudT_icp::Ptr cloud_icp(new PointCloudT_icp);  // ICP 输出点云
	PointCloudT_icp::Ptr cloud_tr(new PointCloudT_icp);  // 匹配点云

	//读取pcd文件
	if (pcl::io::loadPCDFile<pcl::PointXYZ>("pc1.pcd", *cloud_in) == -1)
	{
		PCL_ERROR("Couldn't read file1 \n");
		return (-1);
	}
	std::cout << "Loaded " << cloud_in->size() << " data points from file1" << std::endl;

	if (pcl::io::loadPCDFile<pcl::PointXYZ>("pc2.pcd", *cloud_icp) == -1)
	{
		PCL_ERROR("Couldn't read file2 \n");
		return (-1);
	}
	std::cout << "Loaded " << cloud_icp->size() << " data points from file2" << std::endl;

	int iterations = 1;  // 默认的ICP迭代次数
	*cloud_tr = *cloud_icp;
	//icp配准
	pcl::IterativeClosestPoint<pcl::PointXYZ, pcl::PointXYZ> icp; //创建ICP对象，用于ICP配准
	icp.setMaximumIterations(iterations);    //设置最大迭代次数iterations=true
	icp.setInputCloud(cloud_tr); //设置输入点云
	icp.setInputTarget(cloud_in); //设置目标点云（输入点云进行仿射变换，得到目标点云）
	icp.align(*cloud_icp);          //匹配后源点云
	//pcl::io::savePLYFile("Final.ply",*cloud_icp); //保存
	icp.setMaximumIterations(1);  // 设置为1以便下次调用
	std::cout << "Applied " << iterations << " ICP iteration(s)" << std::endl;
	if (icp.hasConverged())//icp.hasConverged ()=1（true）输出变换矩阵的适合性评估
	{
		std::cout << "\nICP has converged, score is " << icp.getFitnessScore() << std::endl;
		std::cout << "\nICP transformation " << iterations << " : cloud_icp -> cloud_in" << std::endl;
	}
	else
	{
		PCL_ERROR("\nICP has not converged.\n");
		return (-1);
	}


	//可视化
	pcl::visualization::PCLVisualizer viewer("ICP demo");
	// 创建两个观察视点
	int v1(0);
	int v2(1);
	viewer.createViewPort(0.0, 0.0, 0.5, 1.0, v1);
	viewer.createViewPort(0.5, 0.0, 1.0, 1.0, v2);
	// 定义显示的颜色信息
	float bckgr_gray_level = 0.0;  // Black
	float txt_gray_lvl = 1.0 - bckgr_gray_level;
	// 原始的点云设置为白色的
	pcl::visualization::PointCloudColorHandlerCustom<PointT_icp> cloud_in_color_h(cloud_in, (int)255 * txt_gray_lvl, (int)255 * txt_gray_lvl,
		(int)255 * txt_gray_lvl);

	viewer.addPointCloud(cloud_in, cloud_in_color_h, "cloud_in_v1", v1);//设置原始的点云都是显示为白色
	viewer.addPointCloud(cloud_in, cloud_in_color_h, "cloud_in_v2", v2);

	// 转换后的点云显示为绿色
	pcl::visualization::PointCloudColorHandlerCustom<PointT_icp> cloud_tr_color_h(cloud_tr, 20, 180, 20);
	viewer.addPointCloud(cloud_tr, cloud_tr_color_h, "cloud_tr_v1", v1);

	// ICP配准后的点云为红色
	pcl::visualization::PointCloudColorHandlerCustom<PointT_icp> cloud_icp_color_h(cloud_icp, 180, 20, 20);
	viewer.addPointCloud(cloud_icp, cloud_icp_color_h, "cloud_icp_v2", v2);

	// 加入文本的描述在各自的视口界面
	//在指定视口viewport=v1添加字符串“white 。。。”其中"icp_info_1"是添加字符串的ID标志，（10，15）为坐标16为字符大小 后面分别是RGB值
	viewer.addText("White: Original point cloud\nGreen: Matrix transformed point cloud", 10, 15, 16, txt_gray_lvl, txt_gray_lvl, txt_gray_lvl, "icp_info_1", v1);
	viewer.addText("White: Original point cloud\nRed: ICP aligned point cloud", 10, 15, 16, txt_gray_lvl, txt_gray_lvl, txt_gray_lvl, "icp_info_2", v2);

	std::stringstream ss;
	ss << iterations;            //输入的迭代的次数
	std::string iterations_cnt = "ICP iterations = " + ss.str();
	viewer.addText(iterations_cnt, 10, 60, 16, txt_gray_lvl, txt_gray_lvl, txt_gray_lvl, "iterations_cnt", v2);

	// 设置背景颜色
	viewer.setBackgroundColor(bckgr_gray_level, bckgr_gray_level, bckgr_gray_level, v1);
	viewer.setBackgroundColor(bckgr_gray_level, bckgr_gray_level, bckgr_gray_level, v2);

	// 设置相机的坐标和方向
	viewer.setCameraPosition(-3.68332, 2.94092, 5.71266, 0.289847, 0.921947, -0.256907, 0);
	viewer.setSize(1280, 1024);  // 可视化窗口的大小

	// 注册按键回调函数
	viewer.registerKeyboardCallback(&KeyboardEventOccurred, (void*)NULL);

	//显示
	while (!viewer.wasStopped())
	{
		viewer.spinOnce();

		//按下空格键的函数
		if (next_iteration)
		{
			// 最近点迭代算法
			icp.align(*cloud_icp);
			if (icp.hasConverged())
			{
				printf("\033[11A");  // Go up 11 lines in terminal output.
				printf("\nICP has converged, score is %+.0e\n", icp.getFitnessScore());
				std::cout << "\nICP transformation " << ++iterations << " : cloud_icp -> cloud_in" << std::endl;

				ss.str("");
				ss << iterations;
				std::string iterations_cnt = "ICP iterations = " + ss.str();
				viewer.updateText(iterations_cnt, 10, 60, 16, txt_gray_lvl, txt_gray_lvl, txt_gray_lvl, "iterations_cnt");
				viewer.updatePointCloud(cloud_icp, cloud_icp_color_h, "cloud_icp_v2");
			}
			else
			{
				PCL_ERROR("\nICP has not converged.\n");
				return (-1);
			}
		}
		next_iteration = false;
	}


	return 0;
}



long MainWindow::GetMillisec(){
    boost::posix_time::ptime start_time =boost::posix_time::microsec_clock::local_time();
    const boost::posix_time::time_duration td = start_time.time_of_day();
    long millisecond = td.total_milliseconds() - ((td.hours() * 3600 + td.minutes() * 60 + td.seconds()) * 1000) + td.seconds()*1000;
    return millisecond;
}
uint32_t MainWindow::Gray2Color(uint8_t val)
	{ 
		uint8_t r,g,b; 
    uint32_t rgb;
		//red 
		if (val<128) {
			r = 0; 
		} 
		else if (val<192) {
			r = 255/RGBPARM*(val-128);
		} 
		else { r=255; 
		} 
		//green 
		if (val<64) { 
			g = 255/RGBPARM*val;
		} 
		else if (val<192) {
			g = 255; 
		} 
		else { 
			g= -255/RGBPARM*(val - 192)+255;
		}

		//blue 
		if (val<64) { 
			b = 255; 
		} 
		else if (val<128) { 
			b = -255/RGBPARM*(val - 128); 
		} 
		else { 
			b=0;
		}
		
    rgb = ((uint32_t)r << 16 | (uint32_t)g << 8 | (uint32_t)b);
		return rgb; 
	}