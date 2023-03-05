#include "mainwindow.h"
#include "ui_mainwindow.h"
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

#pragma execution_character_set("utf-8")

int MainWindow::b0;
int MainWindow::b1;
int MainWindow::b2;
int MainWindow::b3;
int MainWindow::b4;
int MainWindow::b5;
int MainWindow::robot_bool=0;//机器人是否运作状态量
int MainWindow::robot_bool2=0;
int MainWindow::stop_bool=0;//程序是否执行状态量
float MainWindow::S_box;

MainWindow::MainWindow(QWidget *parent)
    : QMainWindow(parent)
    , ui(new Ui::MainWindow)
{
    SetConsoleOutputCP(CP_UTF8);
    ui->setupUi(this);
    this->resize(1600,960);
    //主窗口背景颜色及字体颜色
    this->setStyleSheet("color:#0bbdc2;"
                        "background-color:#141626;"
                        "QPushButton{color:#0bbdc2;background-color: rgb(42, 59, 81);border:2px groove gray;border-radius:10px;padding:2px 4px;border-style: outset;}"
                        "QPushButton:hover{background-color:rgb(20, 22, 38); color: #bfe0ed;}"
                        "QPushButton:pressed{background-color:grey;color: white;border-style: inset;}"
                        );
    this->setWindowIcon(QIcon(":/images/robot2.png"));
    //按钮样式
    ui->groupBox_7->setStyleSheet("QPushButton{color:#0bbdc2;background-color: rgb(42, 59, 81);border:2px groove gray;border-radius:10px;padding:2px 4px;border-style: outset;}"
                                  "QPushButton:hover{background-color:rgb(20, 22, 38); color: #bfe0ed;}"
                                  "QPushButton:pressed{background-color:grey;color: white;border-style: inset;}");
    ui->dockWidgetContents_2->setStyleSheet("color:#0bbdc2;"
                                            "background-color:#141626;");
    ui->groupBox_2->setStyleSheet("QPushButton{color:#0bbdc2;background-color: rgb(42, 59, 81);border:2px groove gray;border-radius:10px;padding:2px 4px;border-style: outset;}"
                                  "QPushButton:hover{background-color:rgb(20, 22, 38); color: #bfe0ed;}"
                                  "QPushButton:pressed{background-color:grey;color: white;border-style: inset;}");
    ui->groupBox_3->setStyleSheet("QPushButton{color:#0bbdc2;background-color: rgb(42, 59, 81);border:2px groove gray;border-radius:10px;padding:2px 4px;border-style: outset;}"
                                  "QPushButton:hover{background-color:rgb(20, 22, 38); color: #bfe0ed;}"
                                  "QPushButton:pressed{background-color:grey;color: white;border-style: inset;}");
    ui->groupBox_4->setStyleSheet("QPushButton{color:#0bbdc2;background-color: rgb(42, 59, 81);border:2px groove gray;border-radius:10px;padding:2px 4px;border-style: outset;}"
                                  "QPushButton:hover{background-color:rgb(20, 22, 38); color: #bfe0ed;}"
                                  "QPushButton:pressed{background-color:grey;color: white;border-style: inset;}");
    ui->groupBox_5->setStyleSheet("QPushButton{color:#0bbdc2;background-color: rgb(42, 59, 81);border:2px groove gray;border-radius:10px;padding:2px 4px;border-style: outset;}"
                                  "QPushButton:hover{background-color:rgb(20, 22, 38); color: #bfe0ed;}"
                                  "QPushButton:pressed{background-color:grey;color: white;border-style: inset;}");
    ui->groupBox_6->setStyleSheet("QPushButton{color:#0bbdc2;background-color: rgb(42, 59, 81);border:2px groove gray;border-radius:10px;padding:2px 4px;border-style: outset;}"
                                  "QPushButton:hover{background-color:rgb(20, 22, 38); color: #bfe0ed;}"
                                  "QPushButton:pressed{background-color:grey;color: white;border-style: inset;}");
    ui->pushButton_setText->setStyleSheet("QPushButton{color:#0bbdc2;background-color: rgb(42, 59, 81);border:2px groove gray;border-radius:10px;padding:2px 4px;border-style: outset;}"
                                          "QPushButton:hover{background-color:rgb(20, 22, 38); color: #bfe0ed;}"
                                          "QPushButton:pressed{background-color:grey;color: white;border-style: inset;}");
    c1->moveToThread(t1);
    c2->moveToThread(t2);
    t1->start();
    t2->start();

    ui->dockWidget->hide();
    resetText();
    //创建QModbusDevice对象
    modbusDevice = new QModbusTcpClient(this);

    my_timer = new QTimer(this);
    connect(my_timer,SIGNAL(timeout()),this,SLOT(read_bool()));

    setLED(19,1);
    setLED(22,1);

    setLED2(ui->LED_1,1,16);
    setLED2(ui->LED_2,1,16);
    setLED2(ui->LED_3,1,16);
    setLED2(ui->LED_4,1,16);
    setLED2(ui->LED_5,0,16);
    setLED2(ui->LED_6,0,16);
    setLED2(ui->LED_7,0,16);
    initial_show();//初始化
    connect(ui->pushButton_setText,SIGNAL(clicked()),this,SLOT(setText()));//保存参数
    connect(ui->pushButton_10,SIGNAL(clicked()),this,SLOT(showcloud()));//刷新显示点云
    connect(this,&MainWindow::send_cloud,c2,&rob2::doing);//子线程开始计算
    connect(c2,&rob2::send_string,this,&MainWindow::get_string);//主页打印文字
    connect(c2,&rob2::send_LED,this,&MainWindow::setLED);//LED状态灯
    connect(c2,&rob2::finished,this,&MainWindow::set_bool2);//robot_bool2
    connect(c2,&rob2::stop_signal,this,&MainWindow::on_stop_clicked);//程序停止
}

MainWindow::~MainWindow()
{
    ui->vtkRenderWidget->GetInteractor()->SetRenderWindow(nullptr);
    ui->vtkRenderWidget->GetInteractor()->SetInteractorStyle(nullptr);
    my_timer->stop();
    delete ui;
}

void MainWindow::resetText()//初始化参数
{
    QList<QString> list_values;
    QFile file("values.txt");
    QTextStream textStream(&file);
    if(!file.open(QIODevice::ReadOnly))
    {
        //设置最小体素边长
        ui->line_tisu->setText("20");

        //直通滤波X范围1600,2700,-600,520,-180,3000
        ui->line_x1->setText("1600");
        ui->line_x2->setText("2700");
        //直通滤波Y范围
        ui->line_y1->setText("-650");
        ui->line_y2->setText("520");
        //直通滤波Z范围
        ui->line_z1->setText("-180");
        ui->line_z2->setText("3000");

        //设置半径
        ui->line_radius->setText("100");
        //设置查询点的邻域点集数
        ui->line_radius2->setText("50");

        //单个聚类最小点数
        ui->line_ju1->setText("900");
        //单个聚类最大点数
        ui->line_ju2->setText("2000");
        //搜索的邻域点个数
        ui->line_ju3->setText("30");
        //法向量夹角阈值
        ui->line_ju4->setText("4.5");

        //求第几个聚类的包围盒
        ui->line_NOju->setText("1");
        ui->line_NOju->setText("1");
        //Z方向补偿角度
        ui->line_Zbuchang->setText("62");

        qDebug()<<file.errorString();
        file.open(QIODevice::WriteOnly|QFile::Append);
        textStream<<ui->line_tisu->text()+"\r\n"
                 <<ui->line_x1->text()+"\r\n"<<ui->line_x2->text()+"\r\n"<<ui->line_y1->text()+"\r\n"
                <<ui->line_y2->text()+"\r\n"<<ui->line_z1->text()+"\r\n"<<ui->line_z2->text()+"\r\n"
               <<ui->line_radius->text()+"\r\n"<<ui->line_radius2->text()+"\r\n"
              <<ui->line_ju1->text()+"\r\n"<<ui->line_ju2->text()+"\r\n"<<ui->line_ju3->text()+"\r\n"<<ui->line_ju4->text()+"\r\n"
             <<ui->line_NOju->text()+"\r\n"<<ui->line_Zbuchang->text()+"\r\n";
    }
    else
    {
        textStream.setDevice(&file);
        while(!textStream.atEnd())
        {
            QString oneline;
            oneline = textStream.readLine();
            list_values.append(oneline);
        }

        qDebug()<<"list_values"<<list_values;
        //设置最小体素边长
        ui->line_tisu->setText(list_values[0]);

        //直通滤波X范围
        ui->line_x1->setText(list_values[1]);
        ui->line_x2->setText(list_values[2]);
        //直通滤波Y范围
        ui->line_y1->setText(list_values[3]);
        ui->line_y2->setText(list_values[4]);
        //直通滤波Z范围
        ui->line_z1->setText(list_values[5]);
        ui->line_z2->setText(list_values[6]);

        //设置半径
        ui->line_radius->setText(list_values[7]);
        //设置查询点的邻域点集数
        ui->line_radius2->setText(list_values[8]);

        //单个聚类最小点数
        ui->line_ju1->setText(list_values[9]);
        //单个聚类最大点数
        ui->line_ju2->setText(list_values[10]);
        //搜索的邻域点个数
        ui->line_ju3->setText(list_values[11]);
        //法向量夹角阈值
        ui->line_ju4->setText(list_values[12]);

        //求第几个聚类的包围盒
        ui->line_NOju->setText(list_values[13]);
        ui->line_NOju->setText(list_values[13]);
        //Z方向补偿角度
        ui->line_Zbuchang->setText(list_values[14]);
    }
    textStream.flush();
    file.close();
}

void MainWindow::setText()//保存参数
{
    QFile file("values.txt");
    QTextStream textStream(&file);

    file.open(QIODevice::WriteOnly|QIODevice::Truncate);
    textStream<<ui->line_tisu->text()+"\r\n"
             <<ui->line_x1->text()+"\r\n"<<ui->line_x2->text()+"\r\n"<<ui->line_y1->text()+"\r\n"
            <<ui->line_y2->text()+"\r\n"<<ui->line_z1->text()+"\r\n"<<ui->line_z2->text()+"\r\n"
           <<ui->line_radius->text()+"\r\n"<<ui->line_radius2->text()+"\r\n"
          <<ui->line_ju1->text()+"\r\n"<<ui->line_ju2->text()+"\r\n"<<ui->line_ju3->text()+"\r\n"<<ui->line_ju4->text()+"\r\n"
         <<ui->line_NOju->text()+"\r\n"<<ui->line_Zbuchang->text()+"\r\n";
    textStream.flush();
    file.close();
    resetText();
}

void MainWindow::initial_show()//初始化点云显示
{
    //点云初始化
    show_Original.reset(new PointCloudT);
    show_tisu.reset(new PointCloudT);
    show_zhitong.reset(new PointCloudT);
    show_banjing.reset(new PointCloudT);
    show_quyu.reset(new PointCloudT);
    show_baoweihe.reset(new PointCloudT);
    colored_show.reset(new pcl::PointCloud <pcl::PointXYZRGB>);

    ui->pushButton_2->setEnabled(false);
    ui->pushButton_3->setEnabled(false);
    ui->pushButton_4->setEnabled(false);
    ui->pushButton_5->setEnabled(false);

    showcloud();
}

void MainWindow::showcloud()//刷新点云
{
    ui->textEdit->append("------------开始导入点云");
    double time_cin1 = (double)clock();
    //初始化颜色及大小信息
    float red = 255;
    float  green = 255;
    float blue = 255;
    float  size = 10;

    if (pcl::io::loadPCDFile("cloud_tisu.pcd", *show_Original) < 0)
    {
        ui->textEdit->append("cloud_tisu.pcd为空");
        pcl::PLYReader reader;
        reader.read<pcl::PointXYZ>("bunny.ply", *show_Original);
        //qDebug() << "------------导入点云数量：" << cloud_tisu->points.size() ;
    }
    if (pcl::io::loadPCDFile("cloud_tisu.pcd", *show_tisu) < 0)
    {
        ui->textEdit->append("cloud_tisu.pcd为空");
    }
    if (pcl::io::loadPCDFile("cloud_zhitong.pcd", *show_zhitong) < 0)
    {
        ui->textEdit->append("cloud_zhitong.pcd为空");
    }
    if (pcl::io::loadPCDFile("cloud_banjing.pcd", *show_banjing) < 0)
    {
        ui->textEdit->append("cloud_banjing.pcd为空");
    }
    if (pcl::io::loadPCDFile("colored_pointCloud.pcd", *colored_show) < 0)
    {
        ui->textEdit->append("colored_pointCloud.pcd为空");
    }
    auto renderer2 = vtkSmartPointer<vtkRenderer>::New();
    auto renderWindow2 = vtkSmartPointer<vtkGenericOpenGLRenderWindow>::New();
    renderWindow2->AddRenderer(renderer2);
    viewer.reset(new pcl::visualization::PCLVisualizer(renderer2, renderWindow2, "viewer", false));

    pcl::visualization::PointCloudColorHandlerCustom<pcl::PointXYZ> single_color(show_Original, red, green, blue);//自定义点云颜色
    viewer->addPointCloud<pcl::PointXYZ>(show_Original, single_color, "cloud");
    viewer->setPointCloudRenderingProperties(pcl::visualization::PCL_VISUALIZER_POINT_SIZE, size, "cloud");//设置点云单个点的大小

    ui->vtkRenderWidget->setRenderWindow(viewer->getRenderWindow());
    viewer->setupInteractor(ui->vtkRenderWidget->interactor(), ui->vtkRenderWidget->renderWindow());
    ui->vtkRenderWidget->update();

    //------------------可视化---------------------------------

    int v1(0);
    viewer->createViewPort(0.0, 0.5, 0.33, 1.0, v1);
    viewer->setBackgroundColor(0, 0, 0, v1);
    viewer->addText("Raw point clouds", 10, 10, 16, 1, 1, 1, "v1_text", v1);
    int v2(0);
    viewer->createViewPort(0.33, 0.5, 0.66, 1.0, v2);
    viewer->setBackgroundColor(0.1, 0.1, 0.1, v2);
    viewer->addText("VoxelGrid", 10, 10, 16, 1, 1, 1, "v2_text", v2);
    int v3(0);
    viewer->createViewPort(0.66, 0.5, 1, 1, v3);
    viewer->setBackgroundColor(0, 0, 0, v3);
    viewer->addText("PassThrough", 10, 10, 16, 1, 1, 1, "v3_text", v3);
    int v4(0);
    viewer->createViewPort(0, 0, 0.33, 0.5, v4);
    viewer->setBackgroundColor(0.1, 0.1, 0.1, v4);
    viewer->addText("Radius_outlier_removeal", 10, 10, 16, 1, 1, 1, "v4_text", v4);
    int v5(0);
    viewer->createViewPort(0.33, 0, 0.66, 0.5, v5);
    viewer->setBackgroundColor(0.1, 0.1, 0.1, v5);
    viewer->addText("RegionGrowing", 10, 10, 16, 1, 1, 1, "v5_text", v5);

    viewer->addPointCloud<pcl::PointXYZ>(show_Original, "cloud1", v1);
    viewer->addPointCloud<pcl::PointXYZ>(show_tisu, "cloud_filtered", v2);
    viewer->addPointCloud<pcl::PointXYZ>(show_zhitong, "cloud_filtered3", v3);
    viewer->addPointCloud<pcl::PointXYZ>(show_banjing, "cloud_radius", v4);
    viewer->addPointCloud<pcl::PointXYZRGB>(colored_show, "cloud_color", v5);

    viewer->setPointCloudRenderingProperties(pcl::visualization::PCL_VISUALIZER_COLOR, 1, 1, 1, "cloud1", v1);
    viewer->setPointCloudRenderingProperties(pcl::visualization::PCL_VISUALIZER_COLOR, 1, 0, 0, "cloud_filtered", v2);
    viewer->setPointCloudRenderingProperties(pcl::visualization::PCL_VISUALIZER_COLOR, 0, 1, 0, "cloud_filtered3", v3);
    viewer->setPointCloudRenderingProperties(pcl::visualization::PCL_VISUALIZER_COLOR, 0, 0, 1, "cloud_radius", v4);

    int v6(0);
    viewer->createViewPort(0.66, 0, 1, 0.5, v6);
    viewer->setBackgroundColor(0.1, 0.1, 0.1, v6);
    viewer->addText("OBB", 10, 10, 16, 1, 1, 1, "v6_text", v6);
    //背景视角设置
    viewer->setBackgroundColor(0, 0, 0);
    //viewer->addCoordinateSystem(1.0);
    //viewer->initCameraParameters();
    //viewer->setCameraPosition(0,0,-1,0,0,0,0,1,0);
    //点云更新
    viewer->getRenderWindow()->Render();
    ui->textEdit->append("------------导入点云完成");

    baowei((ui->line_NOju->text()).toInt());

    double time_cin2 = (double)clock();
    ui->textEdit->append(">>>>>>导入点云耗时: "+QString::number((time_cin2-time_cin1)/1000)+"s");
}

void MainWindow::baowei(int a)//test主页求解包围盒
{
    //QString no_julei="RegionGrowing_cluster_"+ui->line_NOju->text()+".pcd";
    QString no_julei="RegionGrowing_cluster_"+QString::number(a)+".pcd";
    qDebug()<<"读取聚类文件的名称："<<no_julei;

    //pcl::io::loadPCDFile(no_julei.toStdString(), *cloud_in);

    pcl::PointCloud<pcl::PointXYZ>::Ptr cloud_in(new pcl::PointCloud<pcl::PointXYZ>);
    pcl::PointCloud<pcl::PointXYZ>::Ptr cloud_in2(new pcl::PointCloud<pcl::PointXYZ>);
    pcl::io::loadPCDFile(no_julei.toStdString(), *cloud_in2);

    if(cloud_in2->size() > 1800)
    {
        ui->textEdit->append("-------------------------------------------------------多个料包聚类到一起！！！");
        on_stop_clicked();
    }

    int maxz_points = INT_MIN;//找出Z最大值

    //qDebug()<<"QString::number(cloud_in2->size())"<<QString::number(cloud_in2->size());

    for (size_t j = 0;j < cloud_in2->size();j++) {
        if(maxz_points < cloud_in2->points[j].z)
        {
            maxz_points = cloud_in2->points[j].z;
            //qDebug()<<maxz_points<<cloud_in2->points[j].z;
        }
    }
    qDebug()<<"聚类Z最大值"<<maxz_points;
    pcl::PointIndices pi1;	///点云索引index数组
    for (size_t j = 0;j < cloud_in2->size();j++) {
        if(cloud_in2->points[j].z >= maxz_points-120)
        {
            pi1.indices.push_back(j);
        }
    }
    pcl::copyPointCloud(*cloud_in2,pi1.indices,*cloud_in);
    pcl::io::savePCDFile(no_julei.toLocal8Bit().constData(), *cloud_in);


    // -------------------------------包围盒----------------------------------------
    //------------创建pcl::MomentOfInertiaEstimation 类-------------
    pcl::MomentOfInertiaEstimation <pcl::PointXYZ> feature_extractor;
    feature_extractor.setInputCloud(cloud_in);
    feature_extractor.compute();

    vector <float> moment_of_inertia;//存储惯性矩的特征向量
    vector <float> eccentricity;//存储偏心率的特征向量
    //声明存储描述符和边框所需的所有必要变量。
    pcl::PointXYZ min_point_AABB;
    pcl::PointXYZ max_point_AABB;
    pcl::PointXYZ min_point_OBB;
    pcl::PointXYZ max_point_OBB;
    pcl::PointXYZ position_OBB;
    Eigen::Matrix3f rotational_matrix_OBB;
    float major_value, middle_value, minor_value;
    Eigen::Vector3f major_vector, middle_vector, minor_vector;
    Eigen::Vector3f mass_center;

    feature_extractor.getMomentOfInertia(moment_of_inertia);//惯性矩特征
    feature_extractor.getEccentricity(eccentricity);//偏心率特征
    feature_extractor.getAABB(min_point_AABB, max_point_AABB);//AABB
    feature_extractor.getOBB(min_point_OBB, max_point_OBB, position_OBB, rotational_matrix_OBB);//OBB对应的相关参数
    feature_extractor.getEigenValues(major_value, middle_value, minor_value);//三个特征值
    feature_extractor.getEigenVectors(major_vector, middle_vector, minor_vector);//三个特征向量
    feature_extractor.getMassCenter(mass_center);//点云中心坐标
    //------------------可视化---------------------------------
    //最小包围盒
    //    viewer->addPointCloud<pcl::PointXYZ>(cloud_in, pcl::visualization::PointCloudColorHandlerCustom<pcl::PointXYZ>(cloud_in, 0, 255, 0), "sample cloud");
    //    viewer->setPointCloudRenderingProperties(pcl::visualization::PCL_VISUALIZER_POINT_SIZE, 2, "sample cloud");
    //AABB
    //    viewer->addCube(min_point_AABB.x, max_point_AABB.x, min_point_AABB.y, max_point_AABB.y, min_point_AABB.z, max_point_AABB.z, 1.0, 0.0, 0.0, "AABB");
    //    viewer->setShapeRenderingProperties(pcl::visualization::PCL_VISUALIZER_OPACITY, 0.5, "AABB");
    //    viewer->setShapeRenderingProperties(pcl::visualization::PCL_VISUALIZER_LINE_WIDTH, 3, "AABB");

    Eigen::Vector3f position(position_OBB.x, position_OBB.y, position_OBB.z);
    //cout << "\nposition_OBB: " << position_OBB << endl;
    //cout << "mass_center: " << mass_center << endl;
    Eigen::Quaternionf quat(rotational_matrix_OBB);
    viewer->addCube(position, quat, max_point_OBB.x - min_point_OBB.x, max_point_OBB.y - min_point_OBB.y, max_point_OBB.z - min_point_OBB.z, "OBB");
    viewer->setShapeRenderingProperties(pcl::visualization::PCL_VISUALIZER_COLOR, 0, 0, 1, "OBB");
    viewer->setShapeRenderingProperties(pcl::visualization::PCL_VISUALIZER_OPACITY, 0.5, "OBB");
    viewer->setShapeRenderingProperties(pcl::visualization::PCL_VISUALIZER_LINE_WIDTH, 3, "OBB");
    viewer->setRepresentationToWireframeForAllActors();
    pcl::PointXYZ center(mass_center(0), mass_center(1), mass_center(2));
    pcl::PointXYZ x_axis(major_vector(0) + mass_center(0), major_vector(1) + mass_center(1), major_vector(2) + mass_center(2));
    pcl::PointXYZ y_axis(middle_vector(0) + mass_center(0), middle_vector(1) + mass_center(1), middle_vector(2) + mass_center(2));
    pcl::PointXYZ z_axis(minor_vector(0) + mass_center(0), minor_vector(1) + mass_center(1), minor_vector(2) + mass_center(2));
    viewer->addLine(center, x_axis, 1.0f, 0.0f, 0.0f, "major eigen vector");
    viewer->addLine(center, y_axis, 0.0f, 1.0f, 0.0f, "middle eigen vector");
    viewer->addLine(center, z_axis, 0.0f, 0.0f, 1.0f, "minor eigen vector");

    //ui->textEdit->append("蓝色的包围盒为OBB,红色的包围盒为AABB");
    viewer->getRenderWindow()->Render();//点云更新
    //角度变换
    Eigen::Vector3f eulerAngle = rotational_matrix_OBB.eulerAngles(0, 1, 2);
    float xAngle = pcl::rad2deg(eulerAngle(0));//弧度转角度
    float yAngle = pcl::rad2deg(eulerAngle(1));//弧度转角度
    float zAngle = pcl::rad2deg(eulerAngle(2));//弧度转角度

    if(xAngle>-90 && xAngle<0)
    {
        xAngle=xAngle+180;
    }
    if(xAngle>0 && xAngle<90)
    {
        xAngle=xAngle-180;
    }

    if(yAngle>90 && yAngle<180)
    {
        yAngle=yAngle-180;
    }
    if(yAngle>-180 && yAngle<-90)
    {
        yAngle=yAngle+180;
    }

    if(zAngle>90 && zAngle<180)
    {
        zAngle=zAngle-180;
    }
    if(zAngle>-180 && zAngle<-90)
    {
        zAngle=zAngle+180;
    }

    zAngle=zAngle+62;//Z方向补偿量

    if(zAngle>90 && zAngle<180)
    {
        zAngle=zAngle-180;
    }
    if(zAngle>-180 && zAngle<-90)
    {
        zAngle=zAngle+180;
    }

    //紧凑包围盒
    //旋转
    Eigen::Matrix4f OBB_Matrix4f = Eigen::Matrix4f::Identity();//初始化变换矩阵为单位矩阵
    Eigen::Matrix4f OBB_Matrix4f2 = Eigen::Matrix4f::Identity();//初始化变换矩阵为单位矩阵
    for (int i = 0; i < 3; i++)
    {
        for (int j = 0; j < 3; j++)
        {
            OBB_Matrix4f(i, j) = rotational_matrix_OBB(i, j);
        }
    }
    OBB_Matrix4f(0, 3) = position_OBB.x;
    OBB_Matrix4f(1, 3) = position_OBB.y;
    OBB_Matrix4f(2, 3) = position_OBB.z;
    OBB_Matrix4f2 = OBB_Matrix4f.inverse();//求逆矩阵

    pcl::PointCloud<pcl::PointXYZ>::Ptr cloud_box1(new pcl::PointCloud<pcl::PointXYZ>);
    pcl::transformPointCloud(*cloud_in, *cloud_box1, OBB_Matrix4f2);


    float min_S_box = 0;
    float min_theta_z = 0;
    for (float theta_z = -M_PI / 9; theta_z <= M_PI / 9; theta_z += M_PI / 36)
    {
        calculationAABB(cloud_box1, theta_z, S_box);
        //min_S_box = S_box;
        //min_theta_z = pcl::rad2deg(theta_z);

        if (min_S_box == 0)
        {
            min_S_box = S_box;
            min_theta_z = pcl::rad2deg(theta_z);
        }
        else
        {
            if (min_S_box > S_box)
            {
                min_S_box = S_box;
                min_theta_z = pcl::rad2deg(theta_z);
            }
        }
    }
    qDebug()<< "\n角度" << min_theta_z << "下最小面积:" << min_S_box ;
    qDebug()<< "zAngle" <<zAngle;
    zAngle -= min_theta_z;//紧凑包围盒补偿

    //旋转补偿角度
    float theta_zzz = pcl::deg2rad(min_theta_z);
    Eigen::Matrix4f transform_z = Eigen::Matrix4f::Identity();//初始化变换矩阵为单位矩阵
    transform_z(0, 0) = cos(theta_zzz);
    transform_z(0, 1) = -sin(theta_zzz);
    transform_z(1, 0) = sin(theta_zzz);
    transform_z(1, 1) = cos(theta_zzz);
    pcl::transformPointCloud(*cloud_box1, *cloud_box1, transform_z);
    pcl::transformPointCloud(*cloud_box1, *cloud_box1, OBB_Matrix4f);
    viewer->addPointCloud<pcl::PointXYZ>(cloud_box1, pcl::visualization::PointCloudColorHandlerCustom<pcl::PointXYZ>(cloud_box1, 255, 0, 0), "cloud_boxZ");
    viewer->setPointCloudRenderingProperties(pcl::visualization::PCL_VISUALIZER_POINT_SIZE, 2, "cloud_boxZ");

    b0=position_OBB.x;
    b1=position_OBB.y;
    b2=position_OBB.z;
    b3=zAngle;  //0
    b4=yAngle;  //180
    b5=xAngle;  //0
}

void MainWindow::calculationAABB(pcl::PointCloud<pcl::PointXYZ>::Ptr& cloud_in,float theta_in, float S_out)
{
    pcl::PointCloud<pcl::PointXYZ>::Ptr cloud_in1(new pcl::PointCloud<pcl::PointXYZ>);

    Eigen::Matrix4f transform_z = Eigen::Matrix4f::Identity();//初始化变换矩阵为单位矩阵
    transform_z(0, 0) = cos(theta_in);
    transform_z(0, 1) = -sin(theta_in);
    transform_z(1, 0) = sin(theta_in);
    transform_z(1, 1) = cos(theta_in);
    pcl::transformPointCloud(*cloud_in, *cloud_in1, transform_z);

    pcl::MomentOfInertiaEstimation <pcl::PointXYZ> feature_extractor;
    feature_extractor.setInputCloud(cloud_in1);
    feature_extractor.compute();

    vector <float> moment_of_inertia;//存储惯性矩的特征向量
    vector <float> eccentricity;//存储偏心率的特征向量
    //声明存储描述符和边框所需的所有必要变量。
    pcl::PointXYZ min_point_AABB;
    pcl::PointXYZ max_point_AABB;
    pcl::PointXYZ min_point_OBB;
    pcl::PointXYZ max_point_OBB;
    pcl::PointXYZ position_OBB;
    Eigen::Matrix3f rotational_matrix_OBB;
    float major_value, middle_value, minor_value;
    Eigen::Vector3f major_vector, middle_vector, minor_vector;
    Eigen::Vector3f mass_center;

    feature_extractor.getMomentOfInertia(moment_of_inertia);//惯性矩特征
    feature_extractor.getEccentricity(eccentricity);//偏心率特征
    feature_extractor.getAABB(min_point_AABB, max_point_AABB);//AABB对应的左下角和右上角坐标
    feature_extractor.getOBB(min_point_OBB, max_point_OBB, position_OBB, rotational_matrix_OBB);//OBB对应的相关参数
    feature_extractor.getEigenValues(major_value, middle_value, minor_value);//三个特征值
    feature_extractor.getEigenVectors(major_vector, middle_vector, minor_vector);//三个特征向量
    feature_extractor.getMassCenter(mass_center);//点云中心坐标
    //	//------------------计算结果---------------------------------

    S_out = (max_point_AABB.x - min_point_AABB.x)* (max_point_AABB.y - min_point_AABB.y);
    S_box = S_out/1000000;

    //cout << "旋转角度："<< pcl::rad2deg(theta_in) << "	AABB包围盒面积:	" << S_out /1000000 << endl;
}

void MainWindow::on_pushButton_6_clicked()//test包围盒传点
{
    baowei((ui->line_NOju->text()).toInt());

    ui->textEdit->append("-------------------------------------------------------");
    ui->textEdit->append(">>>>>>OBB包围盒中心点坐标X: "+QString::number(b0)+" Y: "+QString::number(b1)+" Z: "+QString::number(b2));
    ui->textEdit->append(">>>>>>OBB包围盒中心点角度值zAngle: "+QString::number(b3)+" yAngle: "+QString::number(b4)+" xAngle: "+QString::number(b5));
    ui->textEdit->append("-------------------------------------------------------");

    if (modbusDevice->state() != QModbusDevice::ConnectedState)
    {
        ui->textEdit->append("无法写入数据，未连接到机器人!!!");
        return;
    }
    //xieru_robot();
    write_dianwei(304,b0);
    write_dianwei(305,b1);
    write_dianwei(306,b2);
    write_dianwei(307,b3);
    write_dianwei(308,0);
    write_dianwei(309,180);
    write_dianwei(310,1);

    ui->textEdit->append(QString::number(b0)+" "+QString::number(b1)+" "+QString::number(b2)+" "+
                         QString::number(b3)+" "+QString::number(0)+" "+QString::number(180));
    ui->textEdit->append("完成写入数据到机器人!!!");
}

void MainWindow::on_stop_clicked()
{
    ui->textEdit->append("-------------------------------------------------------停止运行程序！！！");
    stop_bool=0;
    robot_bool2 = 0;
    //my_timer->stop();
    write_dianwei(310,0);
    write_dianwei(311,0);
    setLED(19,1);
    setLED2(ui->LED_4,1,16);
}

void MainWindow::get_string(QString a)//打印提示信息
{
    ui->textEdit->append(a);
}

void MainWindow::read_bool()//读取机器人状态
{
    //从服务器读数据的读按钮槽方法

    //如果设备没有被创建就返回
    if (!modbusDevice)
    {
        return;
    }
    if (modbusDevice->state() != QModbusDevice::ConnectedState)
    {
        //ui->textEdit->append("无法读取机器人状态，未连接到机器人!!!");
        return;
    }
    QModbusDataUnit readUnit(QModbusDataUnit::HoldingRegisters,310,1);
    if (auto *reply = modbusDevice->sendReadRequest(readUnit,1)) {
        if (!reply->isFinished())
            connect(reply, &QModbusReply::finished, this, &MainWindow::readReady);
        else
            delete reply; // broadcast replies return immediately
        //        qDebug()<<"采集8个数";
    }
}

void MainWindow::readReady()//在这里读的数据
{
    //QModbusReply这个类存储了来自client的数据,sender()返回发送信号的对象的指针
    auto reply = qobject_cast<QModbusReply *>(sender());
    if (!reply)
    {
        return;
    }
    //数据从QModbusReply这个类的result方法中获取,也就是本程序中的reply->result()
    //判断是否出错
    const QModbusDataUnit unit = reply->result();
    robot_bool = unit.value(0);
    if (robot_bool==1){
        setLED(19,0);
    }
    else if(robot_bool==0 && robot_bool2==1)
    {
        qDebug()<<"读取stop_bool"<<stop_bool;
        if(stop_bool == 1)
        {
            xieru_robot();
        }

        setLED(19,1);
    }
    robot_bool2 = robot_bool;
    //qDebug()<<"robot_bool:"<<robot_bool;
}

void MainWindow::write_dianwei(int dizhi,int value)//单片机写入
{
    if (modbusDevice->state() != QModbusDevice::ConnectedState)
    {
        ui->textEdit->append("无法写入数据，未连接到机器人!!!");
        return;
    }

    QModbusDataUnit writeData(QModbusDataUnit::HoldingRegisters, dizhi, 1);

    writeData.setValue(0,value);
    qDebug() << "发送的数据为： " << writeData.values();
    QModbusReply* reply = modbusDevice->sendWriteRequest(writeData, 1);
    if (reply)
    {
        if (!reply->isFinished())
        {
            //接收响应信息
            connect(reply, &QModbusReply::finished, this, [this, reply](){
                if (reply->error() == QModbusDevice::ProtocolError)
                {
                    //接收到的响应信息是协议错误
                    qDebug() << "写入数据错误：ww" << reply->errorString();
                }
                else if (reply->error() != QModbusDevice::NoError)
                {
                    //接收到的响应消息是其它错误
                    qDebug() << "写入数据错误：qq " << reply->errorString();
                }
                else
                {
                    //接收到的消息没有错误 一般没有必要解析响应消息
                    const QModbusDataUnit data = reply->result();

                    qDebug() << "消息数据个数：" << data.valueCount() << " :" << data.values();

                }

                reply->deleteLater();
            });
        }
        else
        {
            //发送没有响应数据
            //broadcast replies return immediately
            reply->deleteLater();
        }
    }
    else
    {
        qDebug() << "sendWriteRequest Error: " << reply->errorString();
    }
}

//开始工作
void MainWindow::on_start_clicked()
{
    stop_bool=0;
    robot_bool2 = 0;
    write_dianwei(310,0);

    setLED(19,0);
    setLED2(ui->LED_4,2,16);

    stop_bool=1;
    setLED(22,0);

    double time_scan1 = (double)clock();

    getPointCloud();//扫描点云
    savePointCloud();//保存点云

    double time_scan2 = (double)clock();
    ui->textEdit->append(">>>>>>扫描点云耗时: "+QString::number((time_scan2-time_scan1)/1000)+"s");

    qDebug() << "------------子线程开始处理点云";
    ui->textEdit->append("---开始处理点云");
    emit send_cloud();//处理点云

    setLED(22,1);
}

void MainWindow::xieru_robot()//写入机器人
{
    QDateTime current_date_time = QDateTime::currentDateTime();
    QString current_date = current_date_time.toString("yyyy-MM-dd hh:mm::ss");
    ui->textEdit->append("-------------------------------------------------------"+current_date);
    qDebug()<<"stop_bool"<<stop_bool;

    QList<QString> list;
    QFile file("points_4.txt");
    QTextStream textStream(&file);
    if(!file.open(QIODevice::ReadOnly))
    {
        qDebug()<<file.errorString();
    }
    textStream.setDevice(&file);

    while(!textStream.atEnd())
    {
        list.append(textStream.readLine());
    }
    file.close();

    if (modbusDevice->state() != QModbusDevice::ConnectedState)
    {
        ui->textEdit->append("无法写入数据，未连接到机器人!!!");
        return;
    }
    if(list.size()!=0)
    {
        b0=list[0].toFloat();
        b1=list[1].toFloat();
        b2=list[2].toFloat();
        b3=list[3].toFloat();
        b4=0/*list[4].toFloat()*/;
        b5=180/*list[5].toFloat()*/;

        write_dianwei(304,b0);
        write_dianwei(305,b1);
        write_dianwei(306,b2);
        write_dianwei(307,b3);
        write_dianwei(308,b4);
        write_dianwei(309,b5);
        write_dianwei(310,1);

        ui->line_X->setText(QString::number(b0));
        ui->line_Y->setText(QString::number(b1));
        ui->line_Z->setText(QString::number(b2));
        ui->line_ZAngle->setText(QString::number(b3));
        ui->line_YAngle->setText(QString::number(b4));
        ui->line_XAngle->setText(QString::number(b5));

        ui->textEdit->append(QString::number(b0)+" "+QString::number(b1)+" "+QString::number(b2)+" "+
                             QString::number(b3)+" "+QString::number(b4)+" "+QString::number(b5));
        ui->textEdit->append("完成写入数据到机器人!!!");
        for(int i=0;i<6;i++){
            list.removeAt(0);
        }
        file.open(QIODevice::WriteOnly|QFile::Truncate);
        file.close();
        if(!file.open(QIODevice::WriteOnly|QFile::Append| QIODevice::Text))
        {
            qDebug()<<file.errorString();
        }
        QTextStream textStream2(&file);
        for(int i=0;i<list.size();i++){
            textStream2<<list.at(i)<<"\n";
        }
        textStream2.flush();
        file.close();
    }
    else
    {
        on_start_clicked();//启动一次 扫描
        ui->textEdit->append("-------------------------------------------------------");
        ui->textEdit->append("启动一次扫描");
        my_timer->stop();
        Sleep(25000);
        my_timer->start();
    }
    qDebug()<<"主页list"<<list;

}

void MainWindow::set_bool2()//接收子线程完成信号，开始写入
{
    robot_bool2 = 1;
    qDebug()<<"robot_bool2"<<robot_bool2;
}

void MainWindow::on_cam_search_clicked()
{
    //Wait for the PhoXiControl
    while (!Factory.isPhoXiControlRunning())
    {
        LOCAL_CROSS_SLEEP(100);
    }
    //qDebug()<<"PhoXi Control Version: " << QString::fromStdString(Factory.GetPhoXiControlVersion()) ;
    //std::cout << "PhoXi Control Version: " << Factory.GetPhoXiControlVersion() << std::endl;
    ui->textEdit->append( "PhoXi Control Version: "+QString::fromStdString(Factory.GetPhoXiControlVersion()));
    //qDebug()<<QString::fromStdString(Factory.GetPhoXiControlVersion());
    std::cout << "PhoXi API Version: " << Factory.GetAPIVersion() << std::endl;

    DeviceList = Factory.GetDeviceList();
    std::cout << "PhoXi Factory found " << DeviceList.size() << " devices." << std::endl << std::endl;
    if (DeviceList.size()==1)
    {
        ui->textEdit->append("没有查找到设备");
        setLED2(ui->LED_1,1,16);
    }
    else
    {
        ui->textEdit->append("设备已查找到");
        setLED2(ui->LED_1,2,16);
    }
    pho::api::PhoXiDeviceInformation *DeviceInfo;
    for (std::size_t i = 0; i < DeviceList.size(); ++i)
    {
        DeviceInfo = &DeviceList[i];
        std::cout << "Device: " << i << std::endl;
        std::cout << "  Name:                    " << DeviceInfo->Name << std::endl;
        std::cout << "  Hardware Identification: " << DeviceInfo->HWIdentification << std::endl;
        std::cout << "  Type:                    " << std::string(DeviceInfo->Type) << std::endl;
        std::cout << "  Firmware version:        " << DeviceInfo->FirmwareVersion << std::endl;
        std::cout << "  Variant:                 " << DeviceInfo->Variant << std::endl;
        std::cout << "  IsFileCamera:            " << (DeviceInfo->IsFileCamera ? "Yes" : "No") << std::endl;
        std::cout << "  Status:                  "
                  << (DeviceInfo->Status.Attached ? "Attached to PhoXi Control. " : "Not Attached to PhoXi Control. ")
                  << (DeviceInfo->Status.Ready    ? "Ready to connect"            : "Occupied")
                  << std::endl << std::endl;
    }
}

void MainWindow::on_cam_connect_clicked()
{
    if (DeviceList.size()==1)
    {
        ui->textEdit->append("没有查找到设备,无法连接到相机");
        setLED2(ui->LED_2,1,16);
    }
    else
    {
        std::size_t Index=0;
        if (Index >= DeviceList.size())
        {
            std::cout << "Bad Index, or not number!" << std::endl;
            return;
        }
        PhoXiDevice = Factory.Create(DeviceList[Index]);
        if (!PhoXiDevice)
        {
            std::cout << "Device " << DeviceList[Index].HWIdentification << " was not created" << std::endl;
            return;
        }

        if (PhoXiDevice->Connect())
        {
            std::cout << "Connection to the device " << DeviceList[Index].HWIdentification << " was Successful!" << std::endl;
            ui->textEdit->append("连接相机成功");
            setLED2(ui->LED_2,2,16);
        }
        else
        {
            std::cout << "Connection to the device " << DeviceList[Index].HWIdentification << " was Unsuccessful!" << std::endl;
            ui->textEdit->append("连接相机失败");
            setLED2(ui->LED_2,1,16);
        }
    }
}

void MainWindow::on_rob_connect_clicked()
{
    if (!modbusDevice){
        ui->textEdit->append("未搜索到机器人");
        setLED2(ui->LED_3,1,16);
        return;
    }
    if (modbusDevice->state() != QModbusDevice::ConnectedState) {
        //处于非连接状态，进行连接
        //TCP连接,端口502，地址127.0.0.1   192.168.2.100
        modbusDevice->setConnectionParameter(QModbusDevice::NetworkPortParameter, 502);
        modbusDevice->setConnectionParameter(QModbusDevice::NetworkAddressParameter, "192.168.1.12");
        //连接超时设置，2000ms
        modbusDevice->setTimeout(2000);
        //连接失败重试连接，3次
        modbusDevice->setNumberOfRetries(3);
        //调试窗口显示连接状态
        modbusDevice->connectDevice();
        qDebug()<< modbusDevice->state();
    }
    //Sleep(4000);
    qDebug()<< modbusDevice->state();
    if (modbusDevice->state() == QModbusDevice::ConnectedState)
    {
        ui->textEdit->append("机器人连接成功");
        setLED2(ui->LED_3,2,16);
        my_timer->start(3000); // 单位为毫秒。此处是500毫秒。
    }
    else
    {
        ui->textEdit->append("机器人连接失败");
        setLED2(ui->LED_3,1,16);
    }
}

void MainWindow::on_pushButton_9_clicked()//刷新显示点云图
{
    ui->dockWidget->show();
}

void MainWindow::setLED(int label,int color)
{
    switch (label){
    case 19:
        // 将label中的文字清空
        ui->label_19->setText("");
        switch (color) {
        case 0:
            // 红色
            ui->label_19->setStyleSheet("background-color:rgb(255,0,0);"
                                        "border-radius: 8px;");
            break;
        case 1:
            // 绿色
            ui->label_19->setStyleSheet("background-color:rgb(0,255,0);"
                                        "border-radius: 8px;");
            break;
        default:
            break;
        }
    case 22:
        // 将label中的文字清空
        ui->label_22->setText("");
        switch (color) {
        case 0:
            // 红色
            ui->label_22->setStyleSheet("background-color:rgb(255,0,0);"
                                        "border-radius: 8px;");
            break;
        case 1:
            // 绿色
            ui->label_22->setStyleSheet("background-color:rgb(0,255,0);"
                                        "border-radius: 8px;");
            break;
        default:
            break;
        }
    default:
        break;
    }
}

void MainWindow::setLED2(QLabel* label, int color, int size)
{
    // 将label中的文字清空
    label->setText("");
    // 先设置矩形大小
    // 如果ui界面设置的label大小比最小宽度和高度小，矩形将被设置为最小宽度和最小高度；
    // 如果ui界面设置的label大小比最小宽度和高度大，矩形将被设置为最大宽度和最大高度；
    QString min_width = QString("min-width: %1px;").arg(size);              // 最小宽度：size
    QString min_height = QString("min-height: %1px;").arg(size);            // 最小高度：size
    QString max_width = QString("max-width: %1px;").arg(size);              // 最小宽度：size
    QString max_height = QString("max-height: %1px;").arg(size);            // 最小高度：size
    // 再设置边界形状及边框
    QString border_radius = QString("border-radius: %1px;").arg(size/2);    // 边框是圆角，半径为size/2
    QString border = QString("border:1px solid black;");                    // 边框为1px黑色
    // 最后设置背景颜色
    QString background = "background-color:";
    switch (color) {
    case 0:
        // 灰色
        background += "rgb(190,190,190)";
        break;
    case 1:
        // 红色
        background += "rgb(255,0,0)";
        break;
    case 2:
        // 绿色
        background += "rgb(0,255,0)";
        break;
    case 3:
        // 黄色
        background += "rgb(255,255,0)";
        break;
    default:
        break;
    }

    const QString SheetStyle = min_width + min_height + max_width + max_height + border_radius + border + background;
    label->setStyleSheet(SheetStyle);
}

void MainWindow::on_cam_cut_clicked()
{
    PhoXiDevice->StopAcquisition();
    std::cout << "Do you want to logout the device? (0 if NO / 1 if YES) ";
    bool Entry=true;
    PhoXiDevice->Disconnect(Entry);
    ui->textEdit->append("退出相机成功");
}

void MainWindow::on_rob_cut_clicked()
{
    modbusDevice->disconnectDevice();
    ui->textEdit->append("退出机器人成功");
}

//相机
void MainWindow::PrintFrameInfo(const pho::api::PFrame &Frame)
{
    const pho::api::FrameInfo &FrameInfo = Frame->Info;
    std::cout << "  Frame params: " << std::endl;
    std::cout << "    Frame Index: " << FrameInfo.FrameIndex << std::endl;
    std::cout << "    Frame Timestamp: " << FrameInfo.FrameTimestamp << " s" << std::endl;
    std::cout << "    Frame Acquisition duration: " << FrameInfo.FrameDuration << " ms" << std::endl;
    std::cout << "    Frame Computation duration: " << FrameInfo.FrameComputationDuration << " ms" << std::endl;
    std::cout << "    Frame Transfer duration: " << FrameInfo.FrameTransferDuration << " ms" << std::endl;
    std::cout << "    Sensor Position: ["
              << FrameInfo.SensorPosition.x << "; "
              << FrameInfo.SensorPosition.y << "; "
              << FrameInfo.SensorPosition.z << "]"
              << std::endl;
    std::cout << "    Total scan count: " << FrameInfo.TotalScanCount << std::endl;
}

void MainWindow::PrintFrameData(const pho::api::PFrame &Frame)
{
    if (Frame->Empty())
    {
        std::cout << "Frame is empty.";
        return;
    }
    std::cout << "  Frame data: " << std::endl;
    if (!Frame->PointCloud.Empty())
    {
        std::cout << "    PointCloud:    ("
                  << Frame->PointCloud.Size.Width << " x "
                  << Frame->PointCloud.Size.Height << ") Type: "
                  << Frame->PointCloud.GetElementName()
                  << std::endl;
    }
    if (!Frame->NormalMap.Empty())
    {
        std::cout << "    NormalMap:     ("
                  << Frame->NormalMap.Size.Width << " x "
                  << Frame->NormalMap.Size.Height << ") Type: "
                  << Frame->NormalMap.GetElementName()
                  << std::endl;
    }
    if (!Frame->DepthMap.Empty())
    {
        std::cout << "    DepthMap:      ("
                  << Frame->DepthMap.Size.Width << " x "
                  << Frame->DepthMap.Size.Height << ") Type: "
                  << Frame->DepthMap.GetElementName()
                  << std::endl;
    }
    if (!Frame->ConfidenceMap.Empty())
    {
        std::cout << "    ConfidenceMap: ("
                  << Frame->ConfidenceMap.Size.Width << " x "
                  << Frame->ConfidenceMap.Size.Height << ") Type: "
                  << Frame->ConfidenceMap.GetElementName()
                  << std::endl;
    }
    if (!Frame->Texture.Empty())
    {
        std::cout << "    Texture:       ("
                  << Frame->Texture.Size.Width << " x "
                  << Frame->Texture.Size.Height << ") Type: "
                  << Frame->Texture.GetElementName()
                  << std::endl;
    }
}

void MainWindow::PrintCapturingSettings(const pho::api::PhoXiCapturingSettings &CapturingSettings)
{
    std::cout << "  CapturingSettings: "         << std::endl;
    std::cout << "    ShutterMultiplier: "       << CapturingSettings.ShutterMultiplier << std::endl;
    std::cout << "    ScanMultiplier: "          << CapturingSettings.ScanMultiplier << std::endl;
    std::cout << "    CameraOnlyMode: "          << CapturingSettings.CameraOnlyMode << std::endl;
    std::cout << "    AmbientLightSuppression: " << CapturingSettings.AmbientLightSuppression << std::endl;
    std::cout << "    MaximumFPS: "              << CapturingSettings.MaximumFPS << std::endl;
    std::cout << "    SinglePatternExposure: "   << CapturingSettings.SinglePatternExposure << std::endl;
    std::cout << "    CodingStrategy: "          << std::string(CapturingSettings.CodingStrategy) << std::endl;
    std::cout << "    CodingQuality: "           << std::string(CapturingSettings.CodingQuality) << std::endl;
    std::cout << "    TextureSource: "           << std::string(CapturingSettings.TextureSource) << std::endl;
}

void MainWindow::PrintProcessingSettings(const pho::api::PhoXiProcessingSettings &ProcessingSettings)
{
    std::cout << "  ProcessingSettings: " << std::endl;
    std::cout << "    Confidence (MaxInaccuracy): " << ProcessingSettings.Confidence << std::endl;
    std::cout << "    CalibrationVolumeOnly: " << ProcessingSettings.CalibrationVolumeOnly;
    PrintVector("MinCameraSpace(in DataCutting)", ProcessingSettings.ROI3D.CameraSpace.min);
    PrintVector("MaxCameraSpace(in DataCutting)", ProcessingSettings.ROI3D.CameraSpace.max);
    PrintVector("MinPointCloudSpace (in DataCutting)", ProcessingSettings.ROI3D.PointCloudSpace.min);
    PrintVector("MaxPointCloudSpace (in DataCutting)", ProcessingSettings.ROI3D.PointCloudSpace.max);
    std::cout << "    MaxCameraAngle: "          << ProcessingSettings.NormalAngle.MaxCameraAngle << std::endl;
    std::cout << "    MaxProjectionAngle: "      << ProcessingSettings.NormalAngle.MaxProjectorAngle << std::endl;
    std::cout << "    MinHalfwayAngle: "         << ProcessingSettings.NormalAngle.MinHalfwayAngle << std::endl;
    std::cout << "    MaxHalfwayAngle: "         << ProcessingSettings.NormalAngle.MaxHalfwayAngle << std::endl;
    std::cout << "    SurfaceSmoothness: "       << std::string(ProcessingSettings.SurfaceSmoothness) << std::endl;
    std::cout << "    NormalsEstimationRadius: " << ProcessingSettings.NormalsEstimationRadius << std::endl;
}

void MainWindow::PrintCoordinatesSettings(const pho::api::PhoXiCoordinatesSettings &CoordinatesSettings)
{
    std::cout << "  CoordinatesSettings: " << std::endl;
    PrintMatrix("CustomRotationMatrix", CoordinatesSettings.CustomTransformation.Rotation);
    PrintVector("CustomTranslationVector", CoordinatesSettings.CustomTransformation.Translation);
    PrintMatrix("RobotRotationMatrix", CoordinatesSettings.CustomTransformation.Rotation);
    PrintVector("RobotTranslationVector", CoordinatesSettings.RobotTransformation.Translation);
    std::cout << "    CoordinateSpace: " << std::string(CoordinatesSettings.CoordinateSpace) << std::endl;
    std::cout << "    RecognizeMarkers: " << CoordinatesSettings.RecognizeMarkers << std::endl;
    std::cout << "    MarkerScale: "
              << CoordinatesSettings.MarkersSettings.MarkerScale.Width << " x "
              << CoordinatesSettings.MarkersSettings.MarkerScale.Height
              << std::endl;
}

void MainWindow::PrintCalibrationSettings(const pho::api::PhoXiCalibrationSettings &CalibrationSettings)
{
    std::cout << "  CalibrationSettings: " << std::endl;
    std::cout << "    FocusLength: " << CalibrationSettings.FocusLength << std::endl;
    std::cout << "    PixelSize: "
              << CalibrationSettings.PixelSize.Width << " x "
              << CalibrationSettings.PixelSize.Height
              << std::endl;
    PrintMatrix("CameraMatrix", CalibrationSettings.CameraMatrix);
    std::cout << "    DistortionCoefficients: " << std::endl;
    std::cout << "      Format is the following: " << std::endl;
    std::cout << "      (k1, k2, p1, p2[, k3[, k4, k5, k6[, s1, s2, s3, s4[, tx, ty]]]])" << std::endl;

    std::vector<double> distCoeffs = CalibrationSettings.DistortionCoefficients;
    std::stringstream currentDistCoeffsSS;
    int brackets = 0;
    currentDistCoeffsSS << "(";
    currentDistCoeffsSS << distCoeffs[0];
    for (int i = 1; i < distCoeffs.size(); ++i)
    {
        if (i == 4 || i == 5 || i == 8 || i == 12 || i == 14)
        {
            currentDistCoeffsSS << "[";
            ++brackets;
        }
        currentDistCoeffsSS << ", " << distCoeffs[i];
    }
    for (int j = 0; j < brackets; ++j)
    {
        currentDistCoeffsSS << "]";
    }
    currentDistCoeffsSS << ")";
    std::cout << "      " << currentDistCoeffsSS.str() << std::endl;
}

void MainWindow::PrintMatrix(const std::string &name, const pho::api::CameraMatrix64f &matrix)
{
    std::cout << "    " << name << ": "
              << std::endl << "      ["
              << matrix[0][0] << ", "
                              << matrix[0][1] << ", "
                                              << matrix[0][2] << "]"

                                                              << std::endl << "      ["
                                                              << matrix[1][0] << ", "
                                                                              << matrix[1][1] << ", "
                                                                                              << matrix[1][2] << "]"

                                                                                                              << std::endl << "      ["
                                                                                                              << matrix[2][0] << ", "
                                                                                                                              << matrix[2][1] << ", "
                                                                                                                                              << matrix[2][2] << "]"
                                                                                                                                                              << std::endl;
}

void MainWindow::PrintVector(const std::string &name, const pho::api::Point3_64f &vector)
{
    std::cout << "    " << name << ": ["
              << vector.x << "; "
              << vector.y << "; "
              << vector.z << "]"
              << std::endl;
}

void MainWindow::getPointCloud()
{
    //Check if the device is connected
    if (!PhoXiDevice || !PhoXiDevice->isConnected())
    {
        std::cout << "Device is not created, or not connected!" << std::endl;
        return;
    }
    //If it is not in Software trigger mode, we need to switch the modes
    if (PhoXiDevice->TriggerMode != pho::api::PhoXiTriggerMode::Software)
    {
        std::cout << "Device is not in Software trigger mode" << std::endl;
        if (PhoXiDevice->isAcquiring())
        {
            std::cout << "Stopping acquisition" << std::endl;
            //If the device is in Acquisition mode, we need to stop the acquisition
            if (!PhoXiDevice->StopAcquisition())
            {
                throw std::runtime_error("Error in StopAcquistion");
            }
        }
        std::cout << "Switching to Software trigger mode " << std::endl;
        //Switching the mode is as easy as assigning of a value, it will call the appropriate calls in the background
        PhoXiDevice->TriggerMode = pho::api::PhoXiTriggerMode::Software;
        //Just check if did everything run smoothly
        if (!PhoXiDevice->TriggerMode.isLastOperationSuccessful())
        {
            throw std::runtime_error(PhoXiDevice->TriggerMode.GetLastErrorMessage().c_str());
        }
    }

    //Start the device acquisition, if necessary
    if (!PhoXiDevice->isAcquiring())
    {
        if (!PhoXiDevice->StartAcquisition())
        {
            throw std::runtime_error("Error in StartAcquisition");
        }
    }
    //We can clear the current Acquisition buffer -- This will not clear Frames that arrives to the PC after the Clear command is performed
    int ClearedFrames = PhoXiDevice->ClearBuffer();
    std::cout << ClearedFrames << " frames were cleared from the cyclic buffer" << std::endl;

    //While we checked the state of the StartAcquisition call, this check is not necessary, but it is a good practice
    if (!PhoXiDevice->isAcquiring())
    {
        std::cout << "Device is not acquiring" << std::endl;
        return;
    }
    for (std::size_t i = 0; i < 1; ++i)
    {
        std::cout << "Triggering the " << i << "-th frame" << std::endl;
        int FrameID = PhoXiDevice->TriggerFrame(/*If false is passed here, the device will reject the frame if it is not ready to be triggered, if true us supplied, it will wait for the trigger*/);
        if (FrameID < 0)
        {
            //If negative number is returned trigger was unsuccessful
            std::cout << "Trigger was unsuccessful!" << std::endl;
            continue;
        }
        else
        {
            std::cout << "Frame was triggered, Frame Id: " << FrameID << std::endl;
        }

        std::cout << "Waiting for frame " << i << std::endl;
        //Wait for a frame with specific FrameID. There is a possibility, that frame triggered before the trigger will arrive after the trigger call, and will be retrieved before requested frame
        //  Because of this, the TriggerFrame call returns the requested frame ID, so it can than be retrieved from the Frame structure. This call is doing that internally in background
        pho::api::PFrame Frame = PhoXiDevice->GetSpecificFrame(FrameID/*, You can specify Timeout here - default is the Timeout stored in Timeout Feature -> Infinity by default*/);
        if (Frame)
        {
            PrintFrameInfo(Frame);
            PrintFrameData(Frame);
            SampleFrame = Frame;
        }
        else
        {
            std::cout << "Failed to retrieve the frame!";
        }
    }
}

void MainWindow::savePointCloud()
{
    //Check if we have SampleFrame Data
    if (!SampleFrame || SampleFrame->Empty())
    {
        std::cout << "Frame does not exist, or has no content!" << std::endl;
        return;
    }

    //We will count the number of measured points
    if (!SampleFrame->PointCloud.Empty())
    {
        int MeasuredPoints = 0;
        pho::api::Point3_32f ZeroPoint(0.0f, 0.0f, 0.0f);
        for (int y = 0; y < SampleFrame->PointCloud.Size.Height; ++y)
        {
            for (int x = 0; x < SampleFrame->PointCloud.Size.Width; ++x)
            {
                if (SampleFrame->PointCloud[y][x] != ZeroPoint)
                {
                    MeasuredPoints++;
                }
            }
        }
        std::cout << "Your sample PointCloud has " << MeasuredPoints << " measured points." << std::endl;

        float *MyLocalCopy = new float[SampleFrame->PointCloud.GetElementsCount() * 3];

        pho::api::Point3_32f *RawPointer = SampleFrame->PointCloud.GetDataPtr();
        memcpy(MyLocalCopy, RawPointer, SampleFrame->PointCloud.GetDataSize());
        //Data are organized as a matrix of X, Y, Z floats, see the documentation for all other types

        delete[] MyLocalCopy;
        //Data from SampleFrame, or all other frames that are returned by the device are copied from the Cyclic buffer and will remain in the memory until the Frame will go out of scope
        //You can specifically call SampleFrame->PointCloud.Clear() to release some of the data
    }

    //You can store the Frame as a ply structure
    //If you don't specify Output folder the PLY file will be saved where FullAPIExample_CSharp.exe is
    const auto outputFolder = OutputFolder.empty() ? std::string() : OutputFolder + DELIMITER;
    const auto sampleFramePly = outputFolder + "SampleFrame.ply";
    std::cout << "Saving frame as 'SampleFrame.ply'" << std::endl;
    if (SampleFrame->SaveAsPly(sampleFramePly, true, true))
    {
        std::cout << "Saved sample frame as PLY to: " << sampleFramePly << std::endl;
    }
    else
    {
        std::cout << "Could not save sample frame as PLY to " << sampleFramePly << " !" << std::endl;
    }
    //You can save scans to any format, you only need to specify path + file name
    //API will look at extension and save the scan in the correct format
    //You can define which options to save (PointCloud, DepthMap, ...) in PhoXi Control application -> Saving options
    //This method has a an optional 2nd parameter: FrameId
    //Use this option to save other scans than the last one
    //Absolute path is prefered
    //If you don't specify Output folder the file will be saved to %APPDATA%\PhotoneoPhoXiControl\ folder on Windows or ~/.PhotoneoPhoXiControl/ on Linux
    const auto sampleFrameAnyFormat = outputFolder + "OtherSampleFrame.tif";
    if (PhoXiDevice->SaveLastOutput(sampleFrameAnyFormat))
    {
        std::cout << "Saved sample frame to3: " << sampleFrameAnyFormat << std::endl;
    }
    else
    {
        std::cout << "Could not save sample frame to4: " << sampleFrameAnyFormat << " !" << std::endl;
    }
    ui->textEdit->append("---扫描仪保存完成");
    //If you want OpenCV support, you need to link appropriate libraries and add OpenCV include directory
    //To add the support, add #define PHOXI_OPENCV_SUPPORT before include of PhoXi include files
#ifdef PHOXI_OPENCV_SUPPORT
    if (!SampleFrame->PointCloud.Empty())
    {
        cv::Mat PointCloudMat;
        if (SampleFrame->PointCloud.ConvertTo(PointCloudMat))
        {
            cv::Point3f MiddlePoint = PointCloudMat.at<cv::Point3f>(PointCloudMat.rows/2, PointCloudMat.cols/2);
            std::cout << "Middle point: " << MiddlePoint.x << "; " << MiddlePoint.y << "; " << MiddlePoint.z;
        }
    }
#endif
    //If you want PCL support, you need to link appropriate libraries and add PCL include directory
    //To add the support, add #define PHOXI_PCL_SUPPORT before include of PhoXi include files
#ifdef PHOXI_PCL_SUPPORT
    //The PCL convert will convert the appropriate data into the pcl PointCloud based on the Point Cloud type
    pcl::PointCloud<pcl::PointXYZRGBNormal> MyPCLCloud;
    SampleFrame->ConvertTo(MyPCLCloud);
#endif
}
