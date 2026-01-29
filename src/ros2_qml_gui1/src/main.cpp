#include <QGuiApplication>
#include <QQmlApplicationEngine>
#include <QQmlContext>
#include "ros2_qml_gui/cam_node.hpp"
#include "ros2_qml_gui/robot_controller.hpp"
#include <thread>

int main(int argc, char *argv[])
{
    QCoreApplication::setAttribute(Qt::AA_EnableHighDpiScaling);
    QCoreApplication::setAttribute(Qt::AA_UseSoftwareOpenGL);

    rclcpp::init(argc, argv);
    QGuiApplication app(argc, argv);
    QQmlApplicationEngine engine;

    auto camNode = std::make_shared<CamNode>(engine);

    // Load saved topic selections (or auto-discover if no config)
    camNode->loadTopicSelections();
    
    engine.rootContext()->setContextProperty("camNode", camNode.get());

    // Initialize Robot Controller
    auto robotController = new RobotController(camNode);
    engine.rootContext()->setContextProperty("robotController", robotController);

    engine.load(QUrl(QStringLiteral("qrc:/qml/Main.qml")));
    if (engine.rootObjects().isEmpty())
        return -1;

    std::thread rosThread([=]() { 
        rclcpp::spin(camNode); 
    });
    rosThread.detach();

    return app.exec();
}
