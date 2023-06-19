#ifndef MAINWINDOW_H
#define MAINWINDOW_H

#include <QMainWindow>
#include <QtGui>
#include <QtCore>
#include <QDebug>
#include <QVBoxLayout>
#include <QRadioButton>
#include <QProcess>

QT_BEGIN_NAMESPACE
namespace Ui { class MainWindow; }
QT_END_NAMESPACE

class ProgramOptions
{
public:
    ProgramOptions()
    {
        QString filePath = ":/config.json";
        qDebug() << "[+] parsing json file = " << parseJsonFile(filePath);

    }

    int parseJsonFile(const QString& filePath)
    {
        // Open the JSON file
          QFile jsonFile(filePath);
          if (!jsonFile.open(QIODevice::ReadOnly | QIODevice::Text)) {
              qDebug() << "Failed to open JSON file:" << jsonFile.errorString();
              return 1;
          }

          // Read the contents of the JSON file
          QByteArray jsonData = jsonFile.readAll();
          jsonFile.close();

          // Parse the JSON data
          QJsonParseError parseError;
          QJsonDocument jsonDoc = QJsonDocument::fromJson(jsonData, &parseError);
          if (parseError.error != QJsonParseError::NoError) {
              qDebug() << "Failed to parse JSON file:" << parseError.errorString();
              return 1;
          }

          // Make sure the JSON data is an object
          if (!jsonDoc.isObject()) {
              qDebug() << "JSON data is not an object";
              return 1;
          }
          // create json object
          jsonObject = jsonDoc.object();
          return 0;
    }

    QStringList getSysCmds(const QString& arg, const QString& name)
    {
        auto cand = getItem("system");
        auto dock = cand.value(arg).toVariant().toMap();
        auto cmd = dock["cmd"].toString();
        cmd = cmd.replace( "\"", "" );
        int insertIndex = dock["argIndex"].toInt();
        QStringList cmds = cmd.split(" ");
        cmds[insertIndex] = "/" + name + cmds[insertIndex];
        return cmds;
    }

    QStringList getSysCmds(const QString& arg)
    {
        auto cand = getItem("system");
        auto dock = cand.value(arg).toVariant().toMap();
        auto cmd = dock["cmd"].toString();
        cmd = cmd.replace( "\"", "" );
        return cmd.split(" ");
    }

    QStringList getControllerNames()
    {
        auto cand = getItem("controller").toVariantMap().toStdMap();

        QStringList result;
        for (const auto name: cand)
        {
           result << name.first;
        }
        return result;
    }

    QStringList getControllerCmds()
    {
        QStringList results;
        auto cand = getItem("controller");
        for(auto item: getControllerNames())
        {
            auto obj = cand.value(item).toVariant().toMap();
            results << obj["cmd"].toString();
        }
        return results;
    }



private:
    QJsonObject jsonObject;
    QJsonObject getItem(const QString& field) const
    {
        return jsonObject.value(field).toObject();
    }
};

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

    void on_process_output();

private:
    Ui::MainWindow *ui;
    QStringListModel *model;
    ProgramOptions opt;
    QVector<QRadioButton*>controllerList;
    QStringList controllerCmds;
    bool isStarted;
    QProcess *proc;
    QSettings *settings;

protected:
    void startProc(QStringList& cmds);

};
#endif // MAINWINDOW_H
