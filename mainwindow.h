#ifndef MAINWINDOW_H
#define MAINWINDOW_H

#include <QMainWindow>

namespace Ui {
class MainWindow;
}

class MainWindow : public QMainWindow
{
    Q_OBJECT

public:
    explicit MainWindow(QWidget *parent = 0);
    ~MainWindow();

private slots:
    void on_btnIniciar_clicked();

    void on_btnParar_clicked();

    void on_btIV_1_clicked();

    void on_btnAcharTab_clicked();

    void on_btnSisLi_clicked();

    void on_btnOpenGL_clicked();

    void on_btnFlange_clicked();

private:
    Ui::MainWindow *ui;
};

#endif // MAINWINDOW_H
