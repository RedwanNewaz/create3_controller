#ifndef MAINWINDOW_H
#define MAINWINDOW_H

#include <QMainWindow>
#include <QTimer>
#include <QVector>
#include <memory>
#include <QString>
#include <QProcess>
#include "CPUSnapshot.h"
#include "dfa.h"
#include "process_manager.h"
#include <any>

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
    void on_btn_start_clicked();
    void timer_callback();
    void on_op_type_sim_toggled(bool checked);
    void on_op_type_exp_toggled(bool checked);
    void set_ui_items();





protected:
    void set_states();


private:
    Ui::MainWindow *ui;
    DFA *m_dfa;
    QTimer *m_timer;
    QVector<bool> m_state;
    // make sure start is the last item and we need to avoid start
    std::array<std::any, START>m_ui_items;

    ProcessManager *m_manager_;

    std::unique_ptr<CPUSnapshot> currentSnap, previousSnap;
    bool m_button_status;
    QProcess *m_proc;
};
#endif // MAINWINDOW_H
