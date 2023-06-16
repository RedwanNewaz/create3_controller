#include "mainwindow.h"
#include "./ui_mainwindow.h"

MainWindow::MainWindow(QWidget *parent)
    : QMainWindow(parent)
    , ui(new Ui::MainWindow)
{
    ui->setupUi(this);
    model = new QStringListModel(this);

    QStringList list;
    list << "ac31" << "ac32";
    model->setStringList(list);

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




}

MainWindow::~MainWindow()
{
    delete ui;
    delete model;
//    delete config;
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
    qDebug() << "[+] docking := " << getRobotName();
    auto cmds = opt.getSysCmds("dock", getRobotName());

    qDebug() << cmds;
}

void MainWindow::on_undockButton_clicked()
{
    qDebug() << "[+] undocking := " << getRobotName();
    auto cmds = opt.getSysCmds("undock", getRobotName());
    qDebug() << cmds;
}

void MainWindow::on_startButton_clicked()
{
    auto cmd = controllerCmds.at(getControllerIndex()) + getRobotName();
    qDebug() << "[+] start := " << cmd;
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
