# pragma execution_character_set("utf-8")

#ifndef MAINWINDOW_H
#define MAINWINDOW_H

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
VTK_MODULE_INIT(vtkRenderingOpenGL2);
VTK_MODULE_INIT(vtkRenderingFreeType);
VTK_MODULE_INIT(vtkInteractionStyle);
VTK_MODULE_INIT(vtkRenderingVolumeOpenGL2);
#include<pcl/visualization/cloud_viewer.h>
#include <pcl/features/normal_3d.h>
#include "QVTKWidget.h"
#include "vtkRenderWindow.h"
#include <QFileDialog>
#include <vtkRenderWindow.h>
#include "qlabel.h"
#include <QList>
#include <QThread>
#include "xiancheng.h"
#include <QDateTime>
#include <QFile>

using namespace std;
typedef pcl::PointXYZ PointT;
typedef pcl::PointCloud<PointT> PointCloudT;

QT_BEGIN_NAMESPACE
namespace Ui { class MainWindow; }
QT_END_NAMESPACE

class MainWindow : public QMainWindow
{
    Q_OBJECT

public:
    MainWindow(QWidget *parent = nullptr);
    ~MainWindow();
    static int b0,b1,b2,b3,b4,b5;
    static float S_box;
    PointCloudT::Ptr show_Original;
    PointCloudT::Ptr show_tisu;
    PointCloudT::Ptr show_zhitong;
    PointCloudT::Ptr show_banjing;
    PointCloudT::Ptr show_quyu;
    PointCloudT::Ptr show_baoweihe;
    pcl::PointCloud<pcl::PointXYZRGB>::Ptr colored_show;

    static int robot_bool;//机器人是否运作状态量
    static int robot_bool2;//辅助判断是否 从1变为0
    static int stop_bool;//stop

    void initial_show();

    void getPointCloud();//获取点云

    void savePointCloud();//保存点云

    void on_cam_cut_clicked();

    void on_rob_cut_clicked();

    void calculationAABB(pcl::PointCloud<pcl::PointXYZ>::Ptr& cloud_in,float theta_in, float S_out);

    void baowei(int a);

signals:
    void send_cloud();//开始处理点云

public slots:

    void get_string(QString);//打印

    void showcloud();//显示点云

    void setLED(int label,int color);//机器人状态指示灯

    void setLED2(QLabel* label, int color, int size);//机器人状态指示灯

private slots:

    void resetText();//初始化参数
    void setText();//保存参数

    //相机
    void on_cam_search_clicked();
    void on_cam_connect_clicked();
    void PrintFrameInfo(const pho::api::PFrame &Frame);
    void PrintFrameData(const pho::api::PFrame &Frame);
    void PrintCapturingSettings(const pho::api::PhoXiCapturingSettings &CapturingSettings);
    void PrintProcessingSettings(const pho::api::PhoXiProcessingSettings &ProcessingSettings);
    void PrintCoordinatesSettings(const pho::api::PhoXiCoordinatesSettings &CoordinatesSettings);
    void PrintCalibrationSettings(const pho::api::PhoXiCalibrationSettings &CalibrationSettings);
    void PrintMatrix(const std::string &name, const pho::api::CameraMatrix64f &matrix);
    void PrintVector(const std::string &name, const pho::api::Point3_64f &vector);

    //机器人
    void on_rob_connect_clicked();

    void write_dianwei(int dizhi,int value);

    void read_bool();

    void xieru_robot();

    void readReady();

    void on_pushButton_9_clicked();

    void on_start_clicked();

    void on_pushButton_6_clicked();

    void set_bool2();//接收子线程完成信号，开始写入

    void on_stop_clicked();


private:
    Ui::MainWindow *ui;

    QTimer *my_timer;//延时
    QThread* t1=new QThread;
    QThread* t2=new QThread;
    cam1* c1=new cam1;
    rob2* c2=new rob2;
    //点云显示
    boost::shared_ptr<pcl::visualization::PCLVisualizer> viewer;
    //相机
    pho::api::PhoXiFactory Factory;
    pho::api::PPhoXi PhoXiDevice;
    pho::api::PFrame SampleFrame;
    std::vector <pho::api::PhoXiDeviceInformation> DeviceList;
    std::string FileCameraFolder = "";
    std::string OutputFolder = "";

    template<class T>
    bool ReadLine(T &Output) const
    {
        std::string Input;
        std::getline(std::cin, Input);
        std::stringstream InputSteam(Input);
        return (InputSteam >> Output) ? true : false;
    }
    bool ReadLine(std::string &Output) const
    {
        std::getline(std::cin, Output);
        return true;
    }
    //机器人
    QModbusClient *modbusDevice = nullptr;

};
#endif // MAINWINDOW_H
