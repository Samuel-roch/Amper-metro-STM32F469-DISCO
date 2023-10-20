#ifndef MAINWINDOW_H
#define MAINWINDOW_H

#include <QMainWindow>
#include <QElapsedTimer>
#include <lvgl.h>

namespace Ui {
class MainWindow;
}

class MainWindow : public QMainWindow
{
    Q_OBJECT

public:
    explicit MainWindow(QWidget *parent = 0);
    ~MainWindow();

private:
    Ui::MainWindow *ui;

public:
    QImage m_screenBuffer;
    QElapsedTimer m_elapsedTimer;

    lv_disp_drv_t lv_display_driver;

    lv_color_t framebuffer_1[800*480];

    int16_t m_last_x = 0;
    int16_t m_last_y = 0;
    bool    m_left_button_down = false;
    void update_frame(lv_color_t *color_p);

protected:
    virtual void mousePressEvent(QMouseEvent *event) override;
    virtual void mouseReleaseEvent(QMouseEvent *event) override;
    virtual void mouseMoveEvent(QMouseEvent *event) override;
    virtual void timerEvent(QTimerEvent *event) override;
};

#endif // MAINWINDOW_H
