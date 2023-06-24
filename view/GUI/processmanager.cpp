#include "processmanager.h"

ProcessManager::ProcessManager(const QString &cmds, QObject *parent) : QObject(parent)
{
    m_cmds = cmds.split(" ");
}

ProcessManager::ProcessManager(const QStringList &cmds, QObject *parent) : QObject(parent), m_cmds(cmds)
{
}

ProcessManager::~ProcessManager()
{
    qDebug() << "[+] process has been terminated " << QThread::currentThread();
    if(m_proc->state() == QProcess::Running)
        terminate();
    this->deleteLater();
}

void ProcessManager::terminate()
{
    do{
        m_proc->terminate();
        m_proc->waitForFinished(3000);
        qDebug() << "process state " << m_proc->state();

    }while(m_proc->state() == QProcess::Running);

}

void ProcessManager::run()
{

    qDebug() << "[+] started new process " << QThread::currentThread();
    qDebug() << m_cmds;
    m_proc = std::make_unique<QProcess>(this);
    QString proc_base = m_cmds.front();
    m_cmds.pop_front();
    this->connect(m_proc.get(), SIGNAL(readyReadStandardOutput()), this, SLOT(on_process_output()));
    m_proc->execute(proc_base, m_cmds);

}

void ProcessManager::on_process_output()
{
    QString output = QString::fromLocal8Bit(m_proc->readAllStandardOutput());
    QStringList lines = output.split("\\r\\n");
    lines.removeAll(QString(""));
    QStringListIterator it(lines);
    while(it.hasNext()){
       qDebug() << qUtf8Printable(it.next())<< endl;
    }
}
