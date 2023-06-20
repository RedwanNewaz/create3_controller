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

    qDebug() << cmds;
    startProc(cmds);
}

void MainWindow::on_undockButton_clicked()
{
    auto robotName = getRobotName();
    qDebug() << "[+] undocking := " << getRobotName();
//    auto cmds = opt.getSysCmds("undock", getRobotName());
    auto cmds = (robotName.isEmpty())? opt.getSysCmds("undock") :opt.getSysCmds("undock", getRobotName());

    qDebug() << cmds;
    startProc(cmds);
}

void MainWindow::on_startButton_clicked()
{
    auto robotName = getRobotName();
    auto cmd = controllerCmds.at(getControllerIndex());
    if (!robotName.isEmpty())
        cmd += " namespace:=" + robotName;
    auto cmds = cmd.split(" ");
    qDebug() << "[+] start := " << cmds;
    // start process
    isStarted = !isStarted;
    //disable other buttons
    if(isStarted)
    {
        //start process
        startProc(cmds);

        // change options
        ui->dockButton->setDisabled(true);
        ui->undockButton->setDisabled(true);
        ui->widget->setDisabled(true);
        ui->startButton->setText("STOP");
    }
    else
    {

        do{
            proc->terminate();
            proc->waitForFinished(3000);
            qDebug() << "process state " << proc->state();

        }while(proc->state() == QProcess::Running);

        delete proc;
        
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

void MainWindow::on_process_output()
{
    QString output = QString::fromLocal8Bit(proc->readAllStandardOutput());
    QStringList lines = output.split("\\r\\n");
    QStringListIterator it(lines);
    while(it.hasNext()){
       qDebug() << qUtf8Printable(it.next())<< endl;
    }
}

void MainWindow::startProc(QStringList &cmds)
{
    proc = new QProcess(this);
    QString proc_base = cmds.front();
    cmds.pop_front();
    connect(proc, SIGNAL(readyReadStandardOutput()), this, SLOT(on_process_output()));
    proc->start(proc_base, cmds);
    qDebug() << "[+] started new process";
}
