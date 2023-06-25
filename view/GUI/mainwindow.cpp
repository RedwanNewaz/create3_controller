#include "mainwindow.h"
#include "./ui_mainwindow.h"

MainWindow::MainWindow(QWidget *parent)
    : QMainWindow(parent)
    , ui(new Ui::MainWindow)
{
    ui->setupUi(this);
    model = new QStringListModel(this);

    // utilize qsetting to populate listView
    settings = new QSettings("create3_controller", "gui");

    QStringList robotNames = settings->value("robotNames", QStringList()).toStringList();

    model->setStringList(robotNames);

    ui->listView->setModel(model);
    ui->comboBox->setModel(model);

    ui->listView->setEditTriggers(QAbstractItemView::AnyKeyPressed | QAbstractItemView::DoubleClicked);

    QVBoxLayout *layout = new QVBoxLayout(ui->widget);
    for(auto controller: opt.getControllerNames())
    {
        QRadioButton *radio = new QRadioButton(controller, this);
        layout->addWidget(radio);
        controllerList.push_back(radio);
    }

    // make a default selection
    controllerList.back()->setChecked(true);
    controllerCmds = opt.getControllerCmds();

    // change window name
    setWindowTitle("create3_controller_gui");
    setWindowFlags(windowFlags() | Qt::WindowStaysOnTopHint);

    // set process flag false
    isStarted = false;
}

MainWindow::~MainWindow()
{
    if(thread->isRunning())
    {
        qDebug() << "[+] quitting thread";
        thread->quit();
    }
    // get all the items from ui's listView
    QStringList allElements;
    for (int i = 0; i < model->rowCount(); ++i) {
        QModelIndex index = model->index(i, 0);
        QString element = model->data(index, Qt::DisplayRole).toString();
        allElements << element;
    }
    qDebug() << allElements;
    settings->setValue("robotNames", allElements);

    delete settings;
    delete model;
    delete ui;

}


void MainWindow::on_addButton_clicked()
{
    // find the position where we want to edit in
    int row = model->rowCount();
    model->insertRows(row, 1);
    //set up to edit mode
    QModelIndex index = model->index(row);
    ui->listView->edit(index);
}

void MainWindow::on_insertButton_clicked()
{
    // find the position where we want to edit in
    int row = ui->listView->currentIndex().row();
    model->insertRows(row, 1);
    //set up to edit mode
    QModelIndex index = model->index(row);
    ui->listView->edit(index);
}

void MainWindow::on_deleteButton_clicked()
{
    model->removeRows(ui->listView->currentIndex().row(), 1);
}

void MainWindow::on_dockButton_clicked()
{
    auto robotName = getRobotName();
    qDebug() << "[+] docking := " << robotName;

    auto cmds = (robotName.isEmpty())? opt.getSysCmds("dock") :opt.getSysCmds("dock", getRobotName());

//    qDebug() << cmds;
    startProc(cmds, "DOCK_BUTTON");
}

void MainWindow::on_undockButton_clicked()
{
    auto robotName = getRobotName();
    qDebug() << "[+] undocking := " << getRobotName();
//    auto cmds = opt.getSysCmds("undock", getRobotName());
    auto cmds = (robotName.isEmpty())? opt.getSysCmds("undock") :opt.getSysCmds("undock", getRobotName());

//    qDebug() << cmds;
    startProc(cmds, "UNDOCK_BUTTON");
}

void MainWindow::on_startButton_clicked()
{
    auto robotName = getRobotName();
    auto cmd = controllerCmds.at(getControllerIndex());
    if (!robotName.isEmpty())
        cmd += " namespace:=" + robotName;
    qDebug() << "[+] start := " << cmd;
    // start process
    isStarted = !isStarted;
    //disable other buttons
    if(isStarted)
    {
        //start process
        startProc(cmd, "START_BUTTON");
        // change options
        ui->dockButton->setDisabled(true);
        ui->undockButton->setDisabled(true);
        ui->widget->setDisabled(true);
        ui->startButton->setText("STOP");
    }
    else
    {
        if(thread->isRunning())
        {
            qDebug() << "[+] quitting start button thread";
            thread->quit();
        }
        //change option
        ui->dockButton->setDisabled(false);
        ui->undockButton->setDisabled(false);
        ui->widget->setDisabled(false);
        ui->startButton->setText("Start");

    }

}

int MainWindow::getControllerIndex() const
{
    int result = -1;
    for(int i = 0; i < controllerList.size(); ++i)
        if(controllerList[i]->isChecked())
            result = i;
    return result;

}

QString MainWindow::getRobotName() const
{
    return ui->comboBox->currentText();
}


void MainWindow::on_sendGoalButton_clicked()
{
    auto send = ui->goalCoordText->toPlainText();
    auto goal = send.split(",");
    // removing white space
    for(auto &g: goal)
        g = g.trimmed();


    QString cmd = "ros2 topic pub --once FIRST/goal_pose nav_msgs/msg/Odometry '{pose:{pose:{position:{x: SECOND,y: THIRD,z: 0.0}}}}'";

    if(goal.size() < 2)
        return;
    auto robotName = getRobotName();
    QString name = (robotName.isEmpty()) ? "/goal_pose" : "/" + robotName + "/goal_pose";


    QStringList cmds;
    cmds << "ros2" << "topic" << "pub" << "--once";
    cmds <<name << "geometry_msgs/msg/PoseStamped";
    QString payload = "{pose:{position:{x: SECOND, y: THIRD, z: 0.0}}}";

//    cmd.replace("FIRST", name);
    payload.replace("SECOND", goal[0]);
    payload.replace("THIRD", goal[1]);
    cmds << payload;
//    qDebug() << "[+] sending " << cmd;
    startProc(cmds, "SEND_GOAL");


}

void MainWindow::on_actionwaypoints_triggered()
{

    QString csvDir = settings->value("csvDir").toString();

    QString filename = QFileDialog::getOpenFileName(this, tr("Open File"),
                                                            csvDir,
                                                            tr("CSV (*.csv)"));

    QDir directory(filename);
    csvDir = filename.replace(directory.dirName(), "");
    qDebug() << "csv_dir " << csvDir;
    settings->setValue("csvDir", csvDir);
    //ros2 action send_goal /waypoints action_waypoints_interfaces/action/Waypoints  "{csv_path: /home/redwan/colcon_ws/src/create3_controller/test/wp_test1.csv}"

    auto robotName = getRobotName();
    QString name = (robotName.isEmpty()) ? "/waypoints" : "/" + robotName + "/waypoints";

    QStringList cmds;
    cmds << "ros2" << "action" << "send_goal";
    cmds <<name << "action_waypoints_interfaces/action/Waypoints";

    QString payload = "{csv_path: FIRST}";
    payload.replace("FIRST", filename + directory.dirName());
    cmds << payload;
    startProc(cmds, "SEND_WAYPOINTS");

}
