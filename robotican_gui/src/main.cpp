#include "gui_manager.h"
#include <QTimer>

int main(int argc, char *argv[])
{
    ros::init(argc, argv, "RoboticanGuiNode");
    ros::AsyncSpinner spinner(1);
    spinner.start();

    QApplication app(argc, argv);
    QMainWindow widget;
    Ui::MainWindow win;
    win.setupUi(&widget);

    GUImanager gui(widget, win, app);
    gui.startGUI();

    widget.show();
    app.exec();
    return 0;
}