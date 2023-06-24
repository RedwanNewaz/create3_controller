#ifndef PROCESSMANAGER_H
#define PROCESSMANAGER_H

#include <QObject>
#include <QThread>
#include <QProcess>
#include <memory>
#include <QDebug>
class ProcessManager : public QObject
{
    Q_OBJECT
public:
    explicit ProcessManager(const QString& cmd, QObject *parent = nullptr);
    ProcessManager(const QStringList& cmds, QObject *parent = nullptr);
    virtual ~ProcessManager();
signals:

public:
    void terminate();
    void run();

protected slots:

    void on_process_output();

private:
    std::unique_ptr<QProcess> m_proc;
    QStringList m_cmds;


};

#endif // PROCESSMANAGER_H
