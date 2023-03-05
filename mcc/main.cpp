#pragma execution_character_set("utf-8")
#include "mainwindow.h"
#include <QApplication>

int main(int argc, char *argv[])
{
    QApplication a(argc, argv);
    MainWindow w;
    w.setWindowTitle("自动化点云处理软件");
    w.show();
    return a.exec();
}
