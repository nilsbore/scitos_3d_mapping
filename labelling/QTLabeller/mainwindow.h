#ifndef MAINWINDOW_H
#define MAINWINDOW_H

#include <QMainWindow>
#include <QDebug>

namespace Ui {
   class MainWindow;
}

class MainWindow : public QMainWindow
{
   Q_OBJECT

public:
   explicit MainWindow(QWidget *parent = 0);
   ~MainWindow();
   void parseInputPath(std::string path, std::string waypoint);
   std::vector<QString> listXmlInFolder(QString qrootFolder, int depth);

public slots:
   void addLabelButtonSlot();
   void showNextLabelSlot();
   void showPreviousLabelSlot();
   void readLabels(QString labels, QString display_labels);
   void addNewLabel();
   void updateLabelLists(QStringList displayLabels);
   void saveLabel();

private:

   void displayImage(int image_number, std::vector<QString> allImageFiles, std::vector<QString> allObjectFiles);

   Ui::MainWindow *ui;
   int m_maxFolderDepth;
   int m_currentImage;
   std::vector<QString> m_allMasks;
   std::vector<QString> m_allObjectFiles;
   QString m_labelFile, m_displayLabelFile;
   QStringList m_allLabels;
   QStringList m_displayLabels;

};

#endif // MAINWINDOW_H
