#include <QGuiApplication>
#include <QQmlApplicationEngine>
#include <QQmlContext>
#include "unified_control_gui/cam_node.hpp"
#include "unified_control_gui/robot_controller.hpp"
#include "unified_control_gui/cartridge_controller.hpp"
#include <thread>

int main(int argc, char *argv[])
{
    QCoreApplication::setAttribute(Qt::AA_EnableHighDpiScaling);
    QCoreApplication::setAttribute(Qt::AA_UseSoftwareOpenGL);

    rclcpp::init(argc, argv);
    QGuiApplication app(argc, argv);
    QQmlApplicationEngine engine;

    auto camNode = std::make_shared<CamNode>(engine);
    camNode->loadTopicSelections();
    
    engine.rootContext()->setContextProperty("camNode", camNode.get());

    auto robotController = new RobotController(camNode);
    engine.rootContext()->setContextProperty("robotController", robotController);

    auto cartridgeController = new CartridgeController(camNode);
    engine.rootContext()->setContextProperty("cartridgeController", cartridgeController);

    engine.load(QUrl(QStringLiteral("qrc:/qml/Main.qml")));
    if (engine.rootObjects().isEmpty())
        return -1;

    std::thread rosThread([=]() { 
        rclcpp::spin(camNode); 
    });
    rosThread.detach();

    return app.exec();
}
