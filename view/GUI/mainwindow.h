#ifndef MAINWINDOW_H
#define MAINWINDOW_H

#include <QMainWindow>
#include <QtGui>
#include <QtCore>
#include <QDebug>
#include <QVBoxLayout>
#include <QRadioButton>
#include <memory>
#include <QFileDialog>
#include "ProgramOptions.h"
#include "processmanager.h"

QT_BEGIN_NAMESPACE
namespace Ui { class MainWindow; }
QT_END_NAMESPACE

class MainWindow : public QMainWindow
{
    Q_OBJECT

public:
    MainWindow(QWidget *parent = nullptr);
    ~MainWindow();

private slots:
    void on_addButton_clicked();

    void on_insertButton_clicked();

    void on_deleteButton_clicked();

    void on_dockButton_clicked();

    void on_undockButton_clicked();

    void on_startButton_clicked();

    int getControllerIndex() const;

    QString getRobotName() const;

    void selectionChanged();

    void on_sendGoalButton_clicked();

    void on_actionwaypoints_triggered();

    void on_actionmap_triggered();

private:
    Ui::MainWindow *ui;
    QStringListModel *model;
    ProgramOptions opt;
    QVector<QRadioButton*>controllerList;
    QStringList controllerCmds;
    bool isStarted;
    QSettings *settings;
    QThread *thread;
    ProcessManager *proc;


private:

    void startProc(const QStringList& cmd, const QString& name, const PMODE& mode);
    void stopProc();





};
#endif // MAINWINDOW_H
