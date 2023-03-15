#include "mainwindow.h"
#include "./ui_mainwindow.h"
#include <QDebug>
#include <QFileInfo>

MainWindow::MainWindow(QWidget *parent)
    : QMainWindow(parent)
    , ui(new Ui::MainWindow)
{
    ui->setupUi(this);

    setWindowTitle("create3_controller_gui");
    setWindowFlags(windowFlags() | Qt::WindowStaysOnTopHint);

    m_dfa = new DFA();
    // resize internal state
    m_state.resize(m_dfa->size());
    fill(m_state.begin(), m_state.end(), false);


    // set ui items for comparison
    set_ui_items();
    set_states();

    // start process manager
    m_manager_ = new ProcessManager();
    connect(ui->actionset_bag_file, SIGNAL(triggered()), m_manager_, SLOT(on_bag_file_triggered()));
    connect(ui->actionset_rviz_file, SIGNAL(triggered()), m_manager_, SLOT(on_rviz_file_triggered()));

    // set cpu usage
    previousSnap = std::make_unique<CPUSnapshot>();
    // start timer
    m_timer = new QTimer();
    connect(m_timer, SIGNAL(timeout(void)), this, SLOT(timer_callback(void)));
    m_timer->start(300);

    // button state
    m_button_status = false;

}

MainWindow::~MainWindow()
{
    delete ui;
    delete m_dfa;
    delete m_timer;
}

void MainWindow::set_ui_items()
{

    m_ui_items[EXP]  = ui->op_type_exp;
    m_ui_items[SIM] = ui->op_type_sim;
    m_ui_items[BAG] = ui->sim_type_bag;
    m_ui_items[GAZEBO] = ui->sim_type_dummy;
    m_ui_items[STATE_EST] = ui->node_type_state_estimator;
    m_ui_items[CONTROLLER] = ui->node_type_controller;
    m_ui_items[JOY] = ui->node_type_joy;
    m_ui_items[MAP] = ui->node_type_map;
    m_ui_items[RVIZ] = ui->node_type_rviz;
    m_ui_items[APRILTAG] = ui->filter_type_apriltag;
    m_ui_items[ODOM] = ui->filter_type_odom;
    m_ui_items[FUSION] = ui->filter_type_fusion;
    qDebug() << "ui item set";
}


void MainWindow::on_btn_start_clicked()
{

    // toggle button color red / green
    QPalette pal = ui->btn_start->palette();
    if(!m_button_status)
    {
        qDebug() << "[MainWindow]: start button clicked";
        std::set<int>pids;
        for(int i=0; i<m_state.size(); ++i)
            if(m_state[i])
                pids.insert(i);

        pids.insert(START);
        m_manager_->start(pids);


        m_button_status = true;
        ui->btn_start->setText("Stop");
        ui->btn_start->setProperty("color", "red");
        pal.setColor(QPalette::Button, QColor(Qt::red));
        ui->btn_start->setAutoFillBackground(true);
        ui->btn_start->setPalette(pal);
        ui->btn_start->update();

    }
    else
    {

        m_manager_->stop();
        m_button_status = false;
        ui->btn_start->setText("Start");
        qDebug() << "[MainWindow]: stop button clicked";
        pal.setColor(QPalette::Button, QColor(Qt::gray));
        ui->btn_start->setAutoFillBackground(true);
        ui->btn_start->setPalette(pal);
        ui->btn_start->update();
    }

}

void MainWindow::timer_callback()
{
    // get ui status
    QVector<bool>temp(m_ui_items.size());
    for (int i = 0; i < m_ui_items.size(); ++i)
    {
        try // is it a radio button?
        {
            auto ptr = std::any_cast<QRadioButton*>(m_ui_items[i]);
            temp[i] = ptr->isChecked();
        }
        catch (const std::bad_any_cast& e)
        {
            try // is it a check box?
            {
                auto ptr = std::any_cast<QCheckBox*>(m_ui_items[i]);
                temp[i] = ptr->isChecked();
            }
            catch (const std::bad_any_cast& e)
            {
                std::cout << i << " " << e.what() << '\n';
            }
        }
    }




    list<int>q;
    for(int i=0; i<m_ui_items.size(); ++i)
        if(temp[i])
            q.push_back(i);
    q.push_back(START);
    if(m_dfa->isValid(q))
    {
        std::copy(temp.begin(), temp.end(), m_state.begin());
        set_states();
        ui->btn_start->setEnabled(true);
    }
    else
    {
        //qDebug()<< "[MainWindow] invalid choice";
        ui->btn_start->setEnabled(false);
    }

    // compute cpu usage
    currentSnap = std::make_unique<CPUSnapshot>();

    const float ACTIVE_TIME = currentSnap->GetActiveTimeTotal() - previousSnap->GetActiveTimeTotal();
    const float IDLE_TIME   = currentSnap->GetIdleTimeTotal() - previousSnap->GetIdleTimeTotal();
    const float TOTAL_TIME  = ACTIVE_TIME + IDLE_TIME;
    int usage = 100.f * ACTIVE_TIME / TOTAL_TIME;
    //    std::cout << "total cpu usage: " << usage << " %" << std::endl;
    previousSnap = std::make_unique<CPUSnapshot>();
    ui->cpu_usage_bar->setValue(usage);


}


void MainWindow::set_states()
{
    // update the ui status based on user input(s)
    for (int i = 0; i < m_ui_items.size(); ++i)
    {
        try // is it a radio button?
        {
            auto ptr = std::any_cast<QRadioButton*>(m_ui_items[i]);
            if(m_state[i] != ptr->isChecked())
                ptr->setChecked(m_state[i]);

        }
        catch (const std::bad_any_cast& e)
        {
            try // is it a check box?
            {
                auto ptr = std::any_cast<QCheckBox*>(m_ui_items[i]);
                if(m_state[i] != ptr->isChecked())
                    ptr->setChecked(m_state[i]);
            }
            catch (const std::bad_any_cast& e)
            {
                std::cout << i << " " << e.what() << '\n';
            }
        }
    }

    // special cases
    if(m_state[EXP])
    {
        ui->grp_sim->setDisabled(true);
        m_state[BAG] = m_state[GAZEBO] = false;
    }
    else
        ui->grp_sim->setDisabled(false);

    // gazebo simulation uses only odometer for the state estimation
    if(m_state[GAZEBO])
    {
        m_state[ODOM] = true;
        m_state[FUSION] = m_state[APRILTAG] = false;
        ui->filter_type_odom->setChecked(m_state[ODOM]);
    }
}



void MainWindow::on_op_type_sim_toggled(bool checked)
{
   if(checked)
   {
       fill(m_state.begin(), m_state.end(), false);
       for(auto v: m_dfa->defaultSimPath())
       {
           m_state[v] = true;
       }
       set_states();
   }

}

void MainWindow::on_op_type_exp_toggled(bool checked)
{
    if(checked)
    {
        fill(m_state.begin(), m_state.end(), false);
        for(auto v: m_dfa->defaultExpPath())
        {
            m_state[v] = true;
        }

        set_states();
    }
}


