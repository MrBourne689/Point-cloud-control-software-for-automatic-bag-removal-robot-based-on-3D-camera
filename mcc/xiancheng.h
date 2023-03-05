# pragma execution_character_set("utf-8")

#ifndef XIANCHENG_H
#define XIANCHENG_H

#include <QMainWindow>
#include <vector>
#include <string>
#include <iostream>
#include <thread>
#include <atomic>
#include <pcl/io/pcd_io.h>
#include <pcl/point_types.h>
#include <pcl/point_cloud.h>
#include <pcl/filters/passthrough.h>
#include <pcl/visualization/pcl_visualizer.h>
#include <boost/thread/thread.hpp>
#include <pcl/filters/voxel_grid.h>
#include <pcl/filters/radius_outlier_removal.h>
#include <pcl/io/ply_io.h>
#include <vector>
#include <pcl/search/search.h>
#include <pcl/common/angles.h>
#include <pcl/features/normal_3d_omp.h>
#include <pcl/segmentation/region_growing.h>	//区域生长
#include <pcl/features/moment_of_inertia_estimation.h>
#include <pcl/visualization/cloud_viewer.h>
#include <pcl/common/transforms.h>	//pcl::transformPointCloud
#include <time.h>

//机器人
#include <QTimer>
#include <QModbusDataUnit> //存储接收和发送数据的类，数据类型为1bit和16bit
#include <QModbusReply> //客户端访问服务器后得到的回复（如客户端读服务器数据时包含数据信息）

class QModbusClient;
class QModbusReply;

//相机
#include <sstream>
#if defined(_WIN32)
#include <windows.h>
#elif defined (__linux__)
    #include <unistd.h>
#endif

#include "include/PhoXi.h"

#if defined(_WIN32)
    #define LOCAL_CROSS_SLEEP(Millis) Sleep(Millis)
    #define DELIMITER "\\"
#elif defined (__linux__) || defined(__APPLE__)
    #define LOCAL_CROSS_SLEEP(Millis) usleep(Millis * 1000)
    #define DELIMITER "/"
#endif

//#include <QVTKWidget.h>
#include <vtkImageViewer2.h>
#include <vtkJPEGReader.h>
#include <vtkPNGReader.h>
#include <vtkRenderWindow.h>
#include <vtkRenderer.h>
#include <vtkImageActor.h>

#include <vtkVersion.h>
#include <vtkPlaneSource.h>
#include <vtkPolyData.h>
#include <vtkSmartPointer.h>
#include <vtkPolyDataMapper.h>
#include <vtkActor.h>
#include <vtkRenderWindowInteractor.h>
//解决no override found for vtkpolydatamapper

//初始化VTK
#include <vtkAutoInit.h>
//VTK_MODULE_INIT(vtkRenderingOpenGL2);
//VTK_MODULE_INIT(vtkRenderingFreeType);
//VTK_MODULE_INIT(vtkInteractionStyle);
//VTK_MODULE_INIT(vtkRenderingVolumeOpenGL2);
#include<pcl/visualization/cloud_viewer.h>
#include <pcl/features/normal_3d.h>
//#include "QVTKWidget.h"
#include "vtkRenderWindow.h"
#include <QFileDialog>
#include <vtkRenderWindow.h>
#include "qlabel.h"
#include <QList>
#include <QThread>
#include <QDebug>
#include <QModbusTcpClient>
#include <QSurfaceFormat>
#include <QVTKOpenGLNativeWidget.h>
#include <vtkSphereSource.h>
#include <vtkPolyDataMapper.h>
#include <vtkActor.h>
#include <vtkRenderer.h>
#include <vtkRenderWindow.h>
#include <vtkGenericOpenGLRenderWindow.h>
#include <vtkNamedColors.h>
#include <vtkProperty.h>
#include <vtkSmartPointer.h>

using namespace std;
typedef pcl::PointXYZ PointT;
typedef pcl::PointCloud<PointT> PointCloudT;

//相机1线程
class cam1 : public QObject
{
    Q_OBJECT
public:
    explicit cam1(QObject *parent = nullptr);

public slots:
    void get_cloud();//接收
};

//机器人2线程
class rob2 : public QObject
{
    Q_OBJECT
public:
    explicit rob2(QObject *parent = nullptr);
    static int number_clusters;//聚类数量
    static float S_box2;
    //全局点云声明
    PointCloudT::Ptr cloud_Original0;
    PointCloudT::Ptr cloud_Original;
    PointCloudT::Ptr cloud_tisu;
    PointCloudT::Ptr cloud_zhitong;
    PointCloudT::Ptr cloud_banjing;
    PointCloudT::Ptr cloud_quyu;
    PointCloudT::Ptr cloud_baoweihe;

    PointCloudT::Ptr cloud_filtered1;
    PointCloudT::Ptr cloud_filtered2;
    pcl::PointCloud<pcl::PointXYZRGB>::Ptr colored_cloud;
    //初始化
    void initialVtkWidget();
    //点云处理
    void readpcd();
    void readply();
    void tisu(PointCloudT::Ptr& cloud_in,PointCloudT::Ptr& cloud_out,float line_value);
    void zhitonglvbo(PointCloudT::Ptr& cloud_in,PointCloudT::Ptr& cloud_out,float value_x1,float value_x2,float value_y1,float value_y2,float value_z1,float value_z2);
    void banjinglvbo(PointCloudT::Ptr& cloud_in,PointCloudT::Ptr& cloud_out,float value_0,float value_1);
    void quyushengzhang(PointCloudT::Ptr& cloud_in,PointCloudT::Ptr& cloud_out,float value_0,float value_1,float value_2,float value_3);
    void biaoding(PointCloudT::Ptr& cloud_in,PointCloudT::Ptr& cloud_out);
    void calculationAABB2(pcl::PointCloud<pcl::PointXYZ>::Ptr& cloud_in,float theta_in, float S_out);
    void baowei2(int a);

signals:
    void send_string(QString);//主页打印
    void refresh_cloud();//点云刷新
    void send_LED(int label,int color);//程序状态
    //void send_dianwei(int,int,int,int,int,int,int);//发送点位
    void finished();//点云计算完成
    void stop_signal();//停止

public slots:
    //处理
    void doing();

};

#endif // XIANCHENG_H
