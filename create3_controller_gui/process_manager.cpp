#include "process_manager.h"
#include <QDebug>

ProcessManager::ProcessManager(QWidget *parent) : QWidget(parent)
{
    // roslaunch
    m_programs[GAZEBO] = "ros2 launch create3_controller create3_gazebo.launch.py";

//    controller param /home/airlab/colcon_ws/src/create3_controller/config/dwa_param.yaml
    m_programs[CONTROLLER] = "ros2 run create3_controller create3_controller_node";


    // roslaunch with args
    m_programs[STATE_EST] = "ros2 run create3_controller create3_state_estimator";

    // rosrun
    m_programs[JOY] = "ros2 launch create3_controller create3_joystick.py";

    // rosrun with args
    // /home/airlab/colcon_ws/src/create3_controller/config/create3_state.rviz
    m_programs[RVIZ] = "rviz2 -d ";
    // ros2 bag play /home/airlab/colcon_ws/bags/sev3/ --qos-profile-overrides-path /home/airlab/colcon_ws/bags/sev3/reliability_override.yaml
    m_programs[BAG] = "ros2 bag play ";

    // settings
    m_settings = new QSettings("gui.conf");




}

ProcessManager::~ProcessManager()
{
    delete m_settings;
    if(!m_processes.empty())
        stop();
}

void ProcessManager::start(const std::set<int> &pids)
{
    // stop processes
    if(!m_processes.empty())
        stop();

    for(const auto& pid: pids)
    {
        if(m_programs.find(pid) != m_programs.end())
        {

            std::string templateProg = "{0}";
            if (pid == RVIZ)
            {
                auto filename = m_settings->value("rviz_file").toString();
                if(!file_exist(filename))
                {
                    qDebug() << "[ProcessManager] RVIZ file not found!";
                    continue;
                }
                templateProg += " {1}";
                templateProg = fmt::format(templateProg, m_programs[pid], filename.toStdString());

            }
            else if(pid == BAG)
            {
                auto path = m_settings->value("bag_file").toString();
                //TODO check dir exist
                if(!QDir(path).exists())
                {
                    qDebug() << "[ProcessManager] BAG file not found!";
                    continue;
                }
                templateProg += " {1} --qos-profile-overrides-path {1}/reliability_override.yaml";
                templateProg = fmt::format(templateProg, m_programs[pid], path.toStdString());
            }
            else if(pid == STATE_EST)
            {
                //https://www.theconstructsim.com/how-to-pass-ros2-arguments-to-a-ros2-node-via-the-command-line/
                templateProg += " --ros-args -p sensor:={1}";
                std::string arg;
                if(pids.count(APRILTAG))
                {
                    arg = "apriltag";
                }
                else if (pids.count(ODOM))
                {
                    arg = "odom";
                }
                else if(pids.count(FUSION))
                {
                    arg = "fusion";
                }
                templateProg = fmt::format(templateProg, m_programs[pid], arg);
            }
            else
            {
                templateProg = fmt::format(templateProg, m_programs[pid]);
            }

            QString program = QString::fromStdString(templateProg);
            qDebug() << program;

            // create a qt process
            QProcess *proc = new QProcess(this);
            QStringList proc_args = program.split(" ");
            QString proc_base = proc_args.front();
            proc_args.pop_front();
            proc_args.removeAll(QString(""));
            qDebug() <<proc_args;
            proc->start(proc_base, proc_args);
            m_processes.push_back(proc);
        }



    }
}

void ProcessManager::stop()
{
    for(auto& proc: m_processes)
    {
        proc->terminate();
    }
    m_processes.clear();
}

void ProcessManager::on_bag_file_triggered()
{

    QString bagFile = QFileDialog::getExistingDirectory(this, tr("Open Directory"),
                                                        "/home",
                                                        QFileDialog::ShowDirsOnly
                                                        | QFileDialog::DontResolveSymlinks);
    if(!bagFile.isEmpty())
        m_settings->setValue("bag_file", bagFile);
}

void ProcessManager::on_rviz_file_triggered()
{
    QString rvizFile = QFileDialog::getOpenFileName(this, tr("Open File"),
                                                    "/home",
                                                    tr("RVIZ (*.rviz)"));
    if(!rvizFile.isEmpty())
        m_settings->setValue("rviz_file", rvizFile);
}

bool ProcessManager::file_exist(const QString &path)
{
    QFileInfo check_file(path);
    // check if file exists and if yes: Is it really a file and no directory?
    if (check_file.exists() && check_file.isFile()) {
        return true;
    } else {
        return false;
    }
}
