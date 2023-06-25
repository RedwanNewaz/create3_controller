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

    QStringList args;
    auto robotName = getRobotName();
    args << robotName;
    auto gen_cmds = opt.getCLIcmd("dock");

    startProc(gen_cmds(args), "DOCK_BUTTON");
}

void MainWindow::on_undockButton_clicked()
{
    QStringList args;
    auto robotName = getRobotName();
    args << robotName;
    auto gen_cmds = opt.getCLIcmd("undock");

    startProc(gen_cmds(args), "UNDOCK_BUTTON");
}

void MainWindow::on_startButton_clicked()
{
    auto robotName = getRobotName().replace("/", "");
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
    auto robotName = ui->comboBox->currentText();
    if(!robotName.isEmpty())
        robotName = "/" + robotName;
    return robotName;
}


void MainWindow::on_sendGoalButton_clicked()
{
    auto send = ui->goalCoordText->toPlainText();
    auto goal = send.split(",");

    QStringList args;

    auto robotName = getRobotName();
    args << robotName;
    // removing white space
    for(auto &g: goal)
    {
        g = g.trimmed();
        args << g;
    }

    auto gen_cmds = opt.getCLIcmd("send_goal");

    startProc(gen_cmds(args), "SEND_GOAL");


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


    QStringList args;
    auto robotName = getRobotName();
    args << robotName;
    args << filename + directory.dirName();

    auto gen_cmds = opt.getCLIcmd("send_waypoints");
    startProc( gen_cmds(args), "SEND_WAYPOINTS");

}
