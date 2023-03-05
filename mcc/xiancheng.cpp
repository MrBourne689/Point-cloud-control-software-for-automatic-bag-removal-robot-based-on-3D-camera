#include "xiancheng.h"

int rob2::number_clusters;

cam1::cam1(QObject *parent) : QObject(parent)
{
    //qRegisterMetaType<__int64>("__int64");
}

void cam1::get_cloud()
{
    //qDebug() << "子线程1------------接收到";
}

float rob2::S_box2;

rob2::rob2(QObject *parent) : QObject(parent)
{
    //qRegisterMetaType<__int64>("__int64");
    initialVtkWidget();
}

void rob2::initialVtkWidget()
{
    //点云初始化
    cloud_Original0.reset(new PointCloudT);
    cloud_Original.reset(new PointCloudT);
    cloud_tisu.reset(new PointCloudT);
    cloud_zhitong.reset(new PointCloudT);
    cloud_banjing.reset(new PointCloudT);
    cloud_quyu.reset(new PointCloudT);
    cloud_baoweihe.reset(new PointCloudT);
    cloud_filtered1.reset(new PointCloudT);
    cloud_filtered2.reset(new PointCloudT);
    colored_cloud.reset(new pcl::PointCloud <pcl::PointXYZRGB>);

    //pcl::PLYReader reader;
    //reader.read<pcl::PointXYZ>("bunny.ply", *cloud_Original);
    //qDebug() << "------------导入点云数量：" << cloud_Original->points.size() ;
    //    emit send_string("------------导入点云数量："+QString::number(cloud_Original->points.size()));
    //showcloud();
}

void rob2::readply()
{
    //读取文本型和二进制型点云数据
    //只能打开PCD文件
    //    QString fileName = QFileDialog::getOpenFileName(this,
    //                                                    tr("Open PointCloud"), ".",
    //                                                    tr("Open PLY files(*.ply)"));
    QString fileName ="SampleFrame.ply";
    if (!fileName.isEmpty())
    {
        std::string file_name = fileName.toStdString();

        pcl::PLYReader reader;
        reader.read<pcl::PointXYZ>(file_name, *cloud_Original0);

        if (cloud_Original0->size() == 0)
        {
            emit send_string("读取的点云为空");
            return;
        }

        QList<QString> list_values2;
        QFile file2("values.txt");
        QTextStream textStream2(&file2);
        //textStream2.setDevice(&file2);
        if(!file2.open(QIODevice::ReadOnly))
        {
            qDebug()<<file2.errorString();
            return;
        }
        while(!textStream2.atEnd())
        {
            QString oneline;
            oneline = textStream2.readLine();
            list_values2.append(oneline);
        }
        textStream2.flush();
        file2.close();

        qDebug()<<"list_values2"<<list_values2;

        biaoding(cloud_Original0,cloud_Original);
        qDebug() << "------------导入点云数量：" << cloud_Original->points.size() ;
        emit send_string("------------导入点云数量："+QString::number(cloud_Original->points.size()));
        pcl::io::savePCDFile("cloud_biaoding.pcd", *cloud_Original);
        //showcloud();

        tisu(cloud_Original,cloud_tisu,list_values2[0].toFloat());
        if (cloud_tisu->size() == 0)
        {
            emit send_string("体素滤波后点云为空");
            emit stop_signal();
            return;
        }
        qDebug() << "------------体素滤波后点云数量：" << cloud_tisu->points.size() ;
        emit send_string("------------体素滤波后点云数量："+QString::number(cloud_tisu->points.size()));
        pcl::io::savePCDFile("cloud_tisu.pcd", *cloud_tisu);

        zhitonglvbo(cloud_tisu,cloud_zhitong,list_values2[1].toFloat(),list_values2[2].toFloat()
                ,list_values2[3].toFloat(),list_values2[4].toFloat(),list_values2[5].toFloat(),list_values2[6].toFloat());
        if (cloud_zhitong->size() == 0)
        {
            emit send_string("直通滤波后点云为空");
            emit stop_signal();
            return;
        }
        qDebug() << "------------直通滤波后点云数量：" << cloud_zhitong->points.size() ;
        emit send_string("------------直通滤波后点云数量："+QString::number(cloud_zhitong->points.size()));
        pcl::io::savePCDFile("cloud_zhitong.pcd", *cloud_zhitong);

        banjinglvbo(cloud_zhitong,cloud_banjing,list_values2[7].toFloat(),list_values2[8].toFloat());
        if (cloud_banjing->size() == 0)
        {
            emit send_string("半径滤波后点云为空");
            emit stop_signal();
            return;
        }
        qDebug() << "------------半径滤波后点云数量：" << cloud_banjing->points.size() ;
        emit send_string("------------半径滤波后点云数量："+QString::number(cloud_banjing->points.size()));
        pcl::io::savePCDFile("cloud_banjing.pcd", *cloud_banjing);

        quyushengzhang(cloud_banjing,cloud_quyu,list_values2[9].toFloat(),list_values2[10].toFloat(),list_values2[11].toFloat(),list_values2[12].toFloat());
        //qDebug() << "------------区域生长滤波后点云数量：" << cloud_quyu->points.size() ;
        // emit send_string("------------区域生长后点云数量："+QString::number(cloud_quyu->points.size()));
    }
}

void rob2::doing()
{
    emit send_LED(22,0);
    qDebug()<<"子线程开始处理点云！";

    double time_start = (double)clock();

    readply();//读取点云

    if(number_clusters==0)
    {
        qDebug()<<"子线程停止处理点云！";
        return;
    }

    double time_mid = (double)clock();
    qDebug()<<">>>>>>子线程点云处理耗时"<<(time_mid-time_start)/1000<<"s";
    emit send_string(">>>>>>子线程点云处理耗时: "+QString::number((time_mid-time_start)/1000)+"s");

    QList<QString> list;
    QFile file("points.txt");
    QTextStream textStream(&file);
    if(!file.open(QIODevice::ReadOnly))
    {
        qDebug()<<file.errorString();
    }
    textStream.setDevice(&file);

    while(!textStream.atEnd())
    {
        for(int i=1;i<=number_clusters;i++) {
            QString OBBx,OBBy,OBBz,zAngle,yAngle,xAngle;
            OBBx = textStream.readLine();
            OBBy = textStream.readLine();
            OBBz = textStream.readLine();
            zAngle = textStream.readLine();
            yAngle = textStream.readLine();
            xAngle = textStream.readLine();
            list.append(OBBx);
            list.append(OBBy);
            list.append(OBBz);
            list.append(zAngle);
            list.append(yAngle);
            list.append(xAngle);
        }
    }
    file.close();

    int maxz_points = INT_MIN;//找出Z最大值
    for (int j=2;j<list.size();j=j+6) {
        if(maxz_points < list[j].toFloat())
        {
            maxz_points = list[j].toFloat();
        }
    }
    qDebug()<<"Z最大值"<<maxz_points;

    QList<QString> list2;
    for (int j=0;j<list.size();j=j+6) {
        if(list[j+2].toFloat() >= (maxz_points-100))//比Z最大值小10cm
        {
            list2.append(list[j]);
            list2.append(list[j+1]);
            list2.append(list[j+2]);
            list2.append(list[j+3]);
            list2.append(list[j+4]);
            list2.append(list[j+5]);
        }
    }
    qDebug()<<"list2"<<list2;

    QList<QString> list3;
    QList<QString> list4;

    while(list2.size()!=0){

        int minx_points = INT_MAX;//找出X最小点
        for (int j=0;j<list2.size();j=j+6) {
            if(minx_points >= list2[j].toFloat())
            {
                minx_points=list2[j].toFloat();
            }
        }
        qDebug()<<"X最小值"<<minx_points;
        //qDebug()<<"list2.size()"<<list2.size();


        for (int j=0;j<list2.size();j=j+6) {
            //qDebug()<<"list2[j].toFloat()"<<list2[j].toFloat();

            if(list2[j].toFloat() >= minx_points &&
                    list2[j].toFloat() <= (minx_points+300))
            {
                list3.append(list2[j]);
                list3.append(list2[j+1]);
                list3.append(list2[j+2]);
                list3.append(list2[j+3]);
                list3.append(list2[j+4]);
                list3.append(list2[j+5]);

                list2.removeAt(j);
                list2.removeAt(j);
                list2.removeAt(j);
                list2.removeAt(j);
                list2.removeAt(j);
                list2.removeAt(j);
                j-=6;//平移取数
            }
        }
        qDebug()<<"子线程list2"<<list2;
        qDebug()<<"子线程list3"<<list3;

        while(list3.size()!=0){
            int min_y=0;
            for (int j=0;j<list3.size();j=j+6)
            {
                if(list3[min_y+1].toFloat() > list3[j+1].toFloat())//找出Y最小点
                {
                    min_y = j;
                }
            }
            //qDebug()<<"Y最小值"<<list3[min_y+1].toFloat();
            list4.append(list3[min_y]);
            list4.append(list3[min_y+1]);
            list4.append(list3[min_y+2]);
            list4.append(list3[min_y+3]);
            list4.append(list3[min_y+4]);
            list4.append(list3[min_y+5]);

            list3.removeAt(min_y);
            list3.removeAt(min_y);
            list3.removeAt(min_y);
            list3.removeAt(min_y);
            list3.removeAt(min_y);
            list3.removeAt(min_y);
        }

    }
    qDebug()<<"子线程list4"<<list4;

    QFile file4("points_4.txt");
    file4.open(QIODevice::WriteOnly|QFile::Truncate);
    file4.close();
    if(!file4.open(QIODevice::WriteOnly|QFile::Append| QIODevice::Text))
    {
        qDebug()<<file4.errorString();
    }
    QTextStream textStream4(&file4);
    for(int i=0;i<list4.size();i++){
        textStream4<<list4.at(i)<<"\n";
    }
    textStream4.flush();
    file4.close();
    emit send_LED(22,1);
    emit finished();
    qDebug()<<"子线程完成";

    double time_end = (double)clock();
    qDebug()<<">>>>>>子线程排序耗时"<<(time_end-time_mid)/1000<<"s";
    emit send_string(">>>>>>子线程点云排序耗时: "+QString::number((time_end-time_mid)/1000)+"s");
    emit send_string("---点云处理完成，开始写入");
}


void rob2::biaoding(PointCloudT::Ptr& cloud_in,PointCloudT::Ptr& cloud_out)
{
    Eigen::Matrix4f transformation = Eigen::Matrix4f::Identity();//初始化变换矩阵为单位矩阵
    transformation(0, 0)=-0.0251833;  transformation(0, 1)=0.997084;  transformation(0, 2)=0.0720375	; transformation(0, 3)=2248.44;
    transformation(1, 0)=0.98973;   transformation(1, 1)=0.0350107; transformation(1, 2)=-0.138594; transformation(1, 3)=371.097;
    transformation(2, 0)=-0.140712; transformation(2, 1)=-0.0678075; transformation(2, 2)=-0.987726;   transformation(2, 3)=3170.56;
    transformation(3, 0)=0; transformation(3, 1)=0; transformation(3, 2)=0; transformation(3, 3)=1;
    // -----------------------------------输出变换矩阵信息------------------------------
    //    std::cout << "The Estimated Rotation and translation matrices (using getTransformation function) are : \n";
    //    std::printf("\n");
    //    std::printf("    | %6.3f %6.3f %6.3f | \n", transformation(0, 0), transformation(0, 1), transformation(0, 2));
    //    std::printf("R = | %6.3f %6.3f %6.3f | \n", transformation(1, 0), transformation(1, 1), transformation(1, 2));
    //    std::printf("    | %6.3f %6.3f %6.3f | \n", transformation(2, 0), transformation(2, 1), transformation(2, 2));
    //    std::printf("\n");
    //    std::printf("t = < %0.3f, %0.3f, %0.3f >\n", transformation(0, 3), transformation(1, 3), transformation(2, 3));

    pcl::transformPointCloud(*cloud_in,*cloud_out,transformation);
}
void rob2::tisu(PointCloudT::Ptr& cloud_in,PointCloudT::Ptr& cloud_out,float line_value)
{
    // --------------------------------VoxelGrid----------------------------------
    //pcl::PointCloud<pcl::PointXYZ>::Ptr cloud_filtered(new pcl::PointCloud<pcl::PointXYZ>);
    pcl::VoxelGrid<pcl::PointXYZ> vg;
    vg.setInputCloud(cloud_in);             // 输入点云
    vg.setLeafSize(line_value, line_value, line_value); // 设置最小体素边长
    vg.filter(*cloud_out);          // 进行滤波
    //qDebug() << "PointCloud after filtering: " << cloud_out->points.size() ;
    //emit send_string("PointCloud after filtering: "+QString::number(cloud_out->points.size()));
}
void rob2::zhitonglvbo(PointCloudT::Ptr& cloud_in,PointCloudT::Ptr& cloud_out,float value_x1,float value_x2,float value_y1,float value_y2,float value_z1,float value_z2)
{

    qDebug()<<value_x1<<value_x2<<value_y1<<value_y2<<value_z1<<value_z2;
    // --------------------------------直通滤波----------------------------------
    pcl::PassThrough<pcl::PointXYZ> pass_Z;
    pass_Z.setInputCloud(cloud_in);
    pass_Z.setFilterFieldName("z"); //滤波字段名被设置为Z轴方向
    pass_Z.setFilterLimits(value_z1, value_z2); //设置在过滤方向上的过滤范围
    // pass_Z.setKeepOrganized(true); // 保持有序点云结构，该功能用于有序点云才有意义。
    pass_Z.setNegative(false); //设置保留范围内的点还是过滤掉范围内的点，标志为false时保留范围内的点
    pass_Z.filter(*cloud_filtered1);

    pcl::PassThrough<pcl::PointXYZ> pass_X;
    pass_X.setInputCloud(cloud_filtered1);
    pass_X.setFilterFieldName("x"); //滤波字段名被设置为X轴方向
    pass_X.setFilterLimits(value_x1, value_x2); //设置在过滤方向上的过滤范围
    pass_X.setNegative(false); //设置保留范围内的点还是过滤掉范围内的点，标志为false时保留范围内的点
    pass_X.filter(*cloud_filtered2);

    pcl::PassThrough<pcl::PointXYZ> pass_Y;
    pass_X.setInputCloud(cloud_filtered2);
    pass_X.setFilterFieldName("y"); //滤波字段名被设置为X轴方向
    pass_X.setFilterLimits(value_y1, value_y2); //设置在过滤方向上的过滤范围
    pass_X.setNegative(false); //设置保留范围内的点还是过滤掉范围内的点，标志为false时保留范围内的点
    pass_X.filter(*cloud_out);

}
void rob2::banjinglvbo(PointCloudT::Ptr& cloud_in,PointCloudT::Ptr& cloud_out,float value_0,float value_1)
{
    //pcl::io::savePCDFile("cloud_filtered.pcd", *cloud_filtered3);
    // -------------------------------半径滤波----------------------------------------
    //pcl::PointCloud<pcl::PointXYZ>::Ptr cloud_radius(new pcl::PointCloud<pcl::PointXYZ>);
    pcl::RadiusOutlierRemoval<pcl::PointXYZ> ror;
    ror.setInputCloud(cloud_in);     // 输入点云
    ror.setRadiusSearch(value_0);        // 设置半径为0.1m范围内找临近点
    ror.setMinNeighborsInRadius(value_1); // 设置查询点的邻域点集数小于10删除
    ror.filter(*cloud_out);       // 执行滤波
    //pcl::io::savePCDFileASCII("cloud_radius.pcd", *cloud_out);
    //qDebug() << "滤波前有: " << cloud_filtered1->size() << " 个点 " ;
    //qDebug()<<"滤波前有: " << cloud_filtered1->size() << " 个点 ";
    //qDebug() << "滤波后有: " << cloud_radius->size() << " 个点 " ;
}
void rob2::quyushengzhang(PointCloudT::Ptr& cloud_in,PointCloudT::Ptr& cloud_out,float value_0,float value_1,float value_2,float value_3)
{
    // -------------------------------区域生长----------------------------------------
    //---------------------------------------法线和表面曲率估计---------------------------------------
    pcl::search::Search<pcl::PointXYZ>::Ptr tree(new pcl::search::KdTree<pcl::PointXYZ>);
    pcl::PointCloud <pcl::Normal>::Ptr normals(new pcl::PointCloud <pcl::Normal>);
    pcl::NormalEstimationOMP<pcl::PointXYZ, pcl::Normal> n;
    n.setInputCloud(cloud_in);   // 设置法线估计对象输入点集
    n.setSearchMethod(tree);  // 设置搜索方法
    n.setNumberOfThreads(6);  // 设置openMP的线程数
    n.setKSearch(20);         // 设置用于法向量估计的k近邻数目
    n.compute(*normals);      // 计算并输出法向量

    //--------------------------------------------区域生长-------------------------------------------
    pcl::RegionGrowing<pcl::PointXYZ, pcl::Normal> reg;
    reg.setMinClusterSize(value_0);                         // 一个聚类需要的最小点数
    reg.setMaxClusterSize(value_1);                    // 一个聚类需要的最大点数
    reg.setSearchMethod(tree);                         // 搜索方法
    reg.setNumberOfNeighbours(value_2);                     // 搜索的邻域点的个数
    reg.setInputCloud(cloud_in);                          // 输入点云
    reg.setInputNormals(normals);                      // 输入的法线
    reg.setSmoothnessThreshold(pcl::deg2rad(value_3));     // 设置平滑阈值，即法向量夹角的阈值(角度制)
    reg.setCurvatureThreshold(1.0);                    // 设置曲率的阈值

    std::vector <pcl::PointIndices> clusters;
    reg.extract(clusters);                             // 获取聚类的结果，分割结果保存在点云索引的向量中。

    qDebug() << "输出聚类的数量clusters.size():" << clusters.size() ;        // 输出聚类的数量
    number_clusters=clusters.size();

    if(clusters.size()==0)
    {
        qDebug()<<"聚类数量为0，程序停止运行!";
        emit send_string("聚类数量为0，程序停止运行!");
        emit stop_signal();
        return;
    }

    //qDebug() << "clusters[0].indices.size():" << clusters[0].indices.size() ; // 输出第一个聚类的数量
    emit send_string("------输出聚类的数量clusters.size():" + QString::number(clusters.size()));
    //emit send_string("聚类clusters[0].indices.size():"+QString::number(clusters[0].indices.size()));



    //----------------------------------------保存聚类的点云----------------------------------------
    int begin = 0;
    for (std::vector<pcl::PointIndices>::const_iterator it = clusters.begin(); it != clusters.end(); ++it)
    {
        emit send_string("-------------------------------------------------------");
        pcl::PointCloud<pcl::PointXYZ>::Ptr cloud_cluster(new pcl::PointCloud<pcl::PointXYZ>);

        for (std::vector<int>::const_iterator pit = it->indices.begin(); pit != it->indices.end(); pit++)

            cloud_cluster->points.push_back(cloud_in->points[*pit]);
        cloud_cluster->width = cloud_cluster->points.size();
        cloud_cluster->height = 1;
        cloud_cluster->is_dense = true;

        qDebug() << "PointCloud representing the Cluster: " << cloud_cluster->points.size() << " data points.";
        emit send_string("PointCloud representing the Cluster:" + QString::number(cloud_cluster->points.size())+" data points.");

        std::stringstream ss;
        ss << "RegionGrowing_cluster_" << begin + 1 << ".pcd";
        pcl::io::savePCDFileBinary(ss.str(), *cloud_cluster);
        qDebug() << "RegionGrowing_cluster_" << begin + 1 << ".pcd" << "Saved" ;
        emit send_string("RegionGrowing_cluster_" + QString::number(begin + 1)+".pcd Saved");

        begin++;
        emit send_string("-------------------------------------------------------");
    }

    //pcl::PointCloud <pcl::PointXYZRGB>::Ptr colored_cloud = reg.getColoredCloud();
    colored_cloud = reg.getColoredCloud();
    // 保存附加颜色的点云，这里保存的结果仍然是一整块点云
    pcl::io::savePCDFileBinary("colored_pointCloud.pcd", *colored_cloud);
    if(clusters.size() != 0){
        QFile file("points.txt");
        file.open(QIODevice::WriteOnly|QFile::Truncate);
        file.close();
        for(size_t i=1;i <= clusters.size();i++){
            baowei2(i);
        }
    }

}
void rob2::baowei2(int a)//包围盒
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
        emit send_string("-------------------------------------------------------多个料包聚类到一起！！！");
        emit stop_signal();
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

    Eigen::Vector3f position(position_OBB.x, position_OBB.y, position_OBB.z);
    Eigen::Quaternionf quat(rotational_matrix_OBB);
    pcl::PointXYZ center(mass_center(0), mass_center(1), mass_center(2));
    pcl::PointXYZ x_axis(major_vector(0) + mass_center(0), major_vector(1) + mass_center(1), major_vector(2) + mass_center(2));
    pcl::PointXYZ y_axis(middle_vector(0) + mass_center(0), middle_vector(1) + mass_center(1), middle_vector(2) + mass_center(2));
    pcl::PointXYZ z_axis(minor_vector(0) + mass_center(0), minor_vector(1) + mass_center(1), minor_vector(2) + mass_center(2));
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

    //cout << "\nOBB_Matrix4f\n" << OBB_Matrix4f << endl;
    //cout << "\nOBB_Matrix4f逆矩阵\n" << OBB_Matrix4f2 << endl;

    pcl::PointCloud<pcl::PointXYZ>::Ptr cloud_box1(new pcl::PointCloud<pcl::PointXYZ>);
    pcl::transformPointCloud(*cloud_in, *cloud_box1, OBB_Matrix4f2);


    float min_S_box = 0;
    float min_theta_z = 0;
    for (float theta_z = -M_PI / 9; theta_z <= M_PI / 9; theta_z += M_PI / 36)
    {
        calculationAABB2(cloud_box1, theta_z, S_box2);
        //min_S_box = S_box;
        //min_theta_z = pcl::rad2deg(theta_z);

        if (min_S_box == 0)
        {
            min_S_box = S_box2;
            min_theta_z = pcl::rad2deg(theta_z);
        }
        else
        {
            if (min_S_box > S_box2)
            {
                min_S_box = S_box2;
                min_theta_z = pcl::rad2deg(theta_z);
            }
        }
    }
    qDebug()<< "\n角度" << min_theta_z << "下最小面积:" << min_S_box ;
    qDebug()<< "zAngle" <<zAngle;
    zAngle -= min_theta_z;//紧凑包围盒补偿

    emit send_string("-------------------------------------------------------");
    qDebug()<<QString::number(a)+"------OBB包围盒中心点坐标:"<<position_OBB.x<<" "<<position_OBB.y<<" "<<position_OBB.z;
    emit send_string(QString::number(a)+"------OBB包围盒中心点坐标:"+QString::number(position_OBB.x)+" "+QString::number(position_OBB.y)+" "+QString::number(position_OBB.z));
    qDebug()<<QString::number(a)+"------xAngle:"<<xAngle<<"  yAngle:"<< yAngle <<" zAngle-"+QString::number(min_theta_z)+":"<< zAngle;
    emit send_string(QString::number(a)+"------包围盒偏转角度：Z(-"+QString::number(min_theta_z)+"):"+QString::number(zAngle)+" Y:"+QString::number(yAngle)+" X:"+QString::number(xAngle));
    qDebug() <<QString::number(a)+"------size of cloud :" << cloud_in->points.size() ;
    emit send_string(QString::number(a)+"------size of cloud :"+QString::number(cloud_in->points.size()));

    //旋转补偿角度
    float theta_zzz = pcl::deg2rad(min_theta_z);
    Eigen::Matrix4f transform_z = Eigen::Matrix4f::Identity();//初始化变换矩阵为单位矩阵
    transform_z(0, 0) = cos(theta_zzz);
    transform_z(0, 1) = -sin(theta_zzz);
    transform_z(1, 0) = sin(theta_zzz);
    transform_z(1, 1) = cos(theta_zzz);
    pcl::transformPointCloud(*cloud_box1, *cloud_box1, transform_z);
    pcl::transformPointCloud(*cloud_box1, *cloud_box1, OBB_Matrix4f);

    QFile file("points.txt");
    if(!file.open(QIODevice::WriteOnly|QFile::Append| QIODevice::Text))
    {
        qDebug()<<file.errorString();
    }
    QTextStream textStream(&file);
    textStream<< position_OBB.x<<"\n"<< position_OBB.y<<"\n"<< position_OBB.z<<"\n"
              << zAngle<<"\n" << yAngle<<"\n" << xAngle<<"\n";
    textStream.flush();

    file.close();
}

void rob2::calculationAABB2(pcl::PointCloud<pcl::PointXYZ>::Ptr& cloud_in,float theta_in, float S_out)
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
    S_box2 = S_out/1000000;

    //cout << "旋转角度："<< pcl::rad2deg(theta_in) << "	AABB包围盒面积:	" << S_out /1000000 << endl;
}

