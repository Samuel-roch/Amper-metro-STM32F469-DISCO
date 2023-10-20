#include "mainwindow.h"
#include "ui_mainwindow.h"
#include <QMouseEvent>
#include <QDebug>

MainWindow * _main_window;

extern "C"
{

void stm32_flush_cb(lv_disp_drv_t *disp_drv, const lv_area_t *area, lv_color_t *color_p)
{
    Q_UNUSED(area);

    _main_window->update_frame(color_p);
    lv_disp_flush_ready(disp_drv);
}

void stm32_touchpad_read_cb(lv_indev_drv_t *indev_drv, lv_indev_data_t *indev_data)
{
    Q_UNUSED(indev_drv);

    static int16_t last_x = 0;
    static int16_t last_y = 0;

    if(_main_window->m_left_button_down)
    {
        indev_data->point.x = _main_window->m_last_x;
        indev_data->point.y = _main_window->m_last_y;
        indev_data->state = LV_INDEV_STATE_PRESSED;
        last_x = indev_data->point.x;
        last_y = indev_data->point.y;
    }
    else
    {
        indev_data->point.x = last_x;
        indev_data->point.y = last_y;
        indev_data->state = LV_INDEV_STATE_RELEASED;
    }
}

static lv_obj_t * chart;
static lv_chart_series_t * ser_rpm;
static lv_chart_series_t * ser_setPoint;
static lv_obj_t * kb;
static lv_obj_t * ta_setPoint;
static lv_obj_t * ta_kP;
static lv_obj_t * ta_kI;
static lv_obj_t * ta_kD;

static lv_obj_t * l_error;
static lv_obj_t * l_duty;
static lv_obj_t * l_P;
static lv_obj_t * l_I;
static lv_obj_t * l_D;
static lv_obj_t * l_RPM;

const lv_coord_t ecg_sample[] =
{
    100,106,113,119,125,131,137,143,148,154,159,164,168,173,177,181,
    184,188,190,193,195,197,198,199,200,200,200,199,198,197,195,193,
    190,188,184,181,177,173,168,164,159,154,148,143,137,131,125,119,
    113,106,100,94,87,81,75,69,63,57,52,46,41,36,32,27,
    23,19,16,12,10,7,5,3,2,1,0,0,0,1,2,3,
    5,7,10,12,16,19,23,27,32,36,41,46,52,57,63,69,
    75,81,87,94,100,
};

void chartUpdate(lv_timer_t * tmr)
{
    static int p = 0;
    lv_chart_set_next_value(chart, ser_rpm, ecg_sample[(p++) % 99]);
    lv_chart_set_next_value(chart, ser_setPoint, 200);

}

static void event_handler(lv_event_t * e)
{
    lv_event_code_t code = lv_event_get_code(e);

    if(code == LV_EVENT_CLICKED)
    {
    }
    else if(code == LV_EVENT_VALUE_CHANGED)
    {
    }
}

static void ta_event_cb(lv_event_t * e)
{
    lv_obj_t * ta = lv_event_get_target(e);

    if(e->code == LV_EVENT_FOCUSED)
    {
        if(lv_obj_has_flag(kb, LV_OBJ_FLAG_HIDDEN))
        {
            lv_obj_clear_flag(kb, LV_OBJ_FLAG_HIDDEN);
        }
        lv_keyboard_set_textarea(kb, ta);
    }
    else if(e->code == LV_EVENT_DEFOCUSED)
    {
        lv_obj_add_flag(kb, LV_OBJ_FLAG_HIDDEN);
    }
}

void create_gui(void)
{
    lv_obj_t * label;
    lv_obj_t * btn1 = lv_btn_create(lv_scr_act());
    lv_obj_add_event_cb(btn1, event_handler, LV_EVENT_ALL, NULL);
    lv_obj_set_pos(btn1, 636, 5);

    label = lv_label_create(btn1);
    lv_label_set_text(label, "Start");
    lv_obj_center(label);

    btn1 = lv_btn_create(lv_scr_act());
    lv_obj_add_event_cb(btn1, event_handler, LV_EVENT_ALL, NULL);
    lv_obj_set_pos(btn1, 636, 145);

    label = lv_label_create(btn1);
    lv_label_set_text(label, "Reset");
    lv_obj_center(label);

    /*Create a chart*/
    chart = lv_chart_create(lv_scr_act());
    lv_obj_set_size(chart, 720, 250);
    lv_obj_align(chart, LV_ALIGN_BOTTOM_MID, 20, -30);

    lv_chart_set_point_count(chart, 200);
    lv_chart_set_range(chart, LV_CHART_AXIS_PRIMARY_Y, 0, 300);
    lv_chart_set_range(chart, LV_CHART_AXIS_SECONDARY_Y, 0, 200);

    lv_chart_set_axis_tick(chart, LV_CHART_AXIS_PRIMARY_Y, 10, 5, 6, 5, true, 40);
    lv_chart_set_axis_tick(chart, LV_CHART_AXIS_PRIMARY_X, 10, 5, 20, 5, true, 40);

    lv_chart_set_update_mode(chart, LV_CHART_UPDATE_MODE_CIRCULAR);

    //lv_obj_add_event_cb(chart, event_cb, LV_EVENT_ALL, NULL);
    lv_obj_refresh_ext_draw_size(chart);

    //cursor = lv_chart_add_cursor(chart, lv_palette_main(LV_PALETTE_BLUE), LV_DIR_LEFT | LV_DIR_BOTTOM);

    ser_rpm = lv_chart_add_series(chart, lv_palette_main(LV_PALETTE_RED), LV_CHART_AXIS_PRIMARY_X);
    ser_setPoint = lv_chart_add_series(chart, lv_palette_main(LV_PALETTE_BLUE), LV_CHART_AXIS_PRIMARY_X);

    lv_chart_set_zoom_x(chart, 250);

    lv_timer_create(chartUpdate, 100, NULL);

    lv_obj_t * l = lv_label_create(lv_scr_act());
    lv_label_set_text(l, "SetPoint:");
    lv_obj_set_pos(l, 8, 18);

    ta_setPoint = lv_textarea_create(lv_scr_act());
    lv_textarea_set_text(ta_setPoint, "0.0");
    lv_obj_set_pos(ta_setPoint, 80, 5);
    lv_obj_set_size(ta_setPoint, 120, 42);
    lv_obj_add_event_cb(ta_setPoint, ta_event_cb, LV_EVENT_ALL, NULL);

    l = lv_label_create(lv_scr_act());
    lv_label_set_text(l, "kP:");
    lv_obj_set_pos(l, 8, 63 );

    ta_kP = lv_textarea_create(lv_scr_act());
    lv_textarea_set_text(ta_kP, "1.0");
    lv_obj_set_pos(ta_kP, 80, 52);
    lv_obj_set_size(ta_kP, 120, 42);
    lv_obj_add_event_cb(ta_kP, ta_event_cb, LV_EVENT_ALL, NULL);

    l = lv_label_create(lv_scr_act());
    lv_label_set_text(l, "kI:");
    lv_obj_set_pos(l, 8, 110 );

    ta_kI = lv_textarea_create(lv_scr_act());
    lv_textarea_set_text(ta_kI, "0.0");

    lv_obj_set_pos(ta_kI, 80, 99);
    lv_obj_set_size(ta_kI, 120, 42);
    lv_obj_add_event_cb(ta_kI, ta_event_cb, LV_EVENT_ALL, NULL);

    l = lv_label_create(lv_scr_act());
    lv_label_set_text(l, "kD:");
    lv_obj_set_pos(l, 8, 152 );

    ta_kD = lv_textarea_create(lv_scr_act());
    lv_textarea_set_text(ta_kD, "0.0");
    lv_obj_set_pos(ta_kD, 80, 146);
    lv_obj_set_size(ta_kD, 120, 42);
    lv_obj_add_event_cb(ta_kD, ta_event_cb, LV_EVENT_ALL, NULL);

    /*Create a keyboard*/
    kb = lv_keyboard_create(lv_scr_act());
    lv_obj_set_size(kb,  LV_HOR_RES, LV_VER_RES / 2);
    lv_keyboard_set_mode(kb, LV_KEYBOARD_MODE_NUMBER);

    lv_obj_add_flag(kb, LV_OBJ_FLAG_HIDDEN);

    l = lv_label_create(lv_scr_act());
    lv_label_set_text(l, "RPM:");
    lv_obj_set_pos(l, 360, 20 );

    l = lv_label_create(lv_scr_act());
    lv_label_set_text(l, "Duty:");
    lv_obj_set_pos(l, 360, 51 );

    l = lv_label_create(lv_scr_act());
    lv_label_set_text(l, "Erro:");
    lv_obj_set_pos(l, 360, 81 );

    l = lv_label_create(lv_scr_act());
    lv_label_set_text(l, "P:");
    lv_obj_set_pos(l, 360, 112 );

    l = lv_label_create(lv_scr_act());
    lv_label_set_text(l, "I:");
    lv_obj_set_pos(l, 360, 142 );

    l = lv_label_create(lv_scr_act());
    lv_label_set_text(l, "D:");
    lv_obj_set_pos(l, 360, 172 );

    l_RPM = lv_label_create(lv_scr_act());
    lv_label_set_text(l_RPM, "0");
    lv_obj_set_pos(l_RPM, 403, 20 );

    l_error = lv_label_create(lv_scr_act());
    lv_label_set_text(l_error, "0");
    lv_obj_set_pos(l_error, 403, 51 );

    l_duty = lv_label_create(lv_scr_act());
    lv_label_set_text(l_duty, "0");
    lv_obj_set_pos(l_duty, 403, 112 );

    l_P = lv_label_create(lv_scr_act());
    lv_label_set_text(l_P, "0");
    lv_obj_set_pos(l_P, 403, 81 );

    l_I = lv_label_create(lv_scr_act());
    lv_label_set_text(l_I, "0");
    lv_obj_set_pos(l_I, 403, 142 );

    l_D = lv_label_create(lv_scr_act());
    lv_label_set_text(l_D, "0");
    lv_obj_set_pos(l_D, 403, 172 );
}



}

MainWindow::MainWindow(QWidget *parent) :
    QMainWindow(parent),
    m_screenBuffer(800, 480, QImage::Format_RGB32),
    ui(new Ui::MainWindow)
{
    ui->setupUi(this);

    _main_window = this;
    setFixedSize(QSize(800, 480));
    //Tracking mouse while moving
    setMouseTracking(true);

    lv_init();

    static lv_disp_draw_buf_t draw_buf;
    lv_disp_draw_buf_init(&draw_buf, framebuffer_1, framebuffer_1, (800*480));
    lv_disp_drv_init(&lv_display_driver);
    lv_display_driver.direct_mode = true;
    lv_display_driver.hor_res = 800;
    lv_display_driver.ver_res = 480;
    lv_display_driver.flush_cb = stm32_flush_cb;
    lv_display_driver.draw_buf = &draw_buf;

    lv_disp_drv_register(&lv_display_driver);

    static lv_indev_drv_t indev_drv;
    lv_indev_drv_init(&indev_drv);

    indev_drv.type = LV_INDEV_TYPE_POINTER;
    indev_drv.read_cb = stm32_touchpad_read_cb;

    lv_indev_drv_register(&indev_drv);

    create_gui();

    startTimer(1);
    m_elapsedTimer.start();
}

MainWindow::~MainWindow()
{
    delete ui;
}

void MainWindow::update_frame(lv_color_t *color_p)
{
    for(int32_t y = 0; y <= 479; y++)
        for(int32_t x = 0; x <= 799; x++)
            m_screenBuffer.setPixel(x , y, lv_color_to32(color_p[(y * 800) + x]));

    ui->frame->setPixmap(QPixmap::fromImage(m_screenBuffer));
}


void MainWindow::mousePressEvent(QMouseEvent *event)
{
    m_last_x = event->pos().x();
    m_last_y = event->pos().y();
    m_left_button_down = true;
    qDebug() << m_last_x;
    qDebug() << m_last_y;
}

void MainWindow::mouseReleaseEvent(QMouseEvent *event)
{
    m_last_x = event->pos().x();
    m_last_y = event->pos().y();
    m_left_button_down = false;
}

void MainWindow::mouseMoveEvent(QMouseEvent *event)
{
    m_last_x = event->pos().x();
    m_last_y = event->pos().y();
}

void MainWindow::timerEvent(QTimerEvent *event)
{
    Q_UNUSED(event)
    lv_tick_inc(1);
    lv_timer_handler();
}
