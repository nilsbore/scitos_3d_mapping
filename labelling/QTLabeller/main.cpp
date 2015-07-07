#include "mainwindow.h"
#include <QApplication>
#include <iostream>

using namespace std;

int main(int argc, char *argv[])
{
   string path="";
   string waypId = "";
   string labels = "";
   string display_labels="";
   if (argc == 5)
   {
       path=argv[1];
      waypId = argv[2];
      labels = argv[3];
      display_labels=argv[4];
   } else {
      std::cout<<"Please provide required argument"<<std::endl;
      return -1;
   }

   QApplication a(argc, argv);
   MainWindow w;
   w.show();

   w.parseInputPath(path, waypId);
   w.readLabels(QString(labels.c_str()),QString(display_labels.c_str()));

   return a.exec();
}
