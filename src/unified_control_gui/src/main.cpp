#include <QGuiApplication>
#include <QQmlApplicationEngine>
#include <QQmlContext>
#include <QDir>
#include <QFileInfo>
#include "unified_control_gui/cam_node.hpp"
#include "unified_control_gui/robot_controller.hpp"
#include "unified_control_gui/cartridge_controller.hpp"
#include "unified_control_gui/scale_controller.hpp"
#include <thread>

int main(int argc, char *argv[])
{
    QCoreApplication::setAttribute(Qt::AA_EnableHighDpiScaling);
    QCoreApplication::setAttribute(Qt::AA_UseSoftwareOpenGL);

    rclcpp::init(argc, argv);
    QGuiApplication app(argc, argv);
    QQmlApplicationEngine engine;

    // Add qrc:/icons path for icons
    engine.addImportPath("qrc:/");

    auto camNode = std::make_shared<CamNode>(engine);
    camNode->loadTopicSelections();
    
    engine.rootContext()->setContextProperty("camNode", camNode.get());

    auto robotController = new RobotController(camNode);
    engine.rootContext()->setContextProperty("robotController", robotController);

    auto cartridgeController = new CartridgeController(camNode);
    engine.rootContext()->setContextProperty("cartridgeController", cartridgeController);

    auto scaleController = new ScaleController(camNode);
    engine.rootContext()->setContextProperty("scaleController", scaleController);

    // Load QML from filesystem (fast iteration) → fallback to qrc
    QString qmlPath = "/home/pi/ros2_ws/src/unified_control_gui/qml/Main.qml";
    if (QFileInfo::exists(qmlPath)) {
        qDebug() << "Loading QML from filesystem:" << qmlPath;
        engine.load(QUrl::fromLocalFile(qmlPath));
    } else {
        qDebug() << "Loading QML from qrc (fallback)";
        engine.load(QUrl(QStringLiteral("qrc:/qml/Main.qml")));
    }

    if (engine.rootObjects().isEmpty())
        return -1;

    std::thread rosThread([=]() { 
        rclcpp::spin(camNode); 
    });
    rosThread.detach();

    return app.exec();
}
