#include "unified_control_gui/cam_node.hpp"
// #include <cv_bridge/cv_bridge.h>
#include <opencv2/imgproc.hpp>
#include <cv_bridge/cv_bridge.hpp>
#include <sensor_msgs/msg/image.hpp>
#include <sensor_msgs/image_encodings.hpp>
#include <fstream>
#include <sys/stat.h>

CamNode::CamNode(QQmlApplicationEngine &engine)
    : QObject(), rclcpp::Node("qml_cam_node_hp"), engine_(&engine) {
    // Set config file path
    const char* home = std::getenv("HOME");
    if (home) {
        std::string config_dir = std::string(home) + "/.config/ros2_gui";
        // Create directory if it doesn't exist
        mkdir(config_dir.c_str(), 0755);
        configFilePath_ = config_dir + "/camera_topics.conf";
        RCLCPP_INFO(this->get_logger(), "Config file: %s", configFilePath_.c_str());
    }
}

void CamNode::setup(const std::vector<std::string> &topics)
{
    subs_.clear();

    std::vector<std::string> limitedTopics = topics;
    if (limitedTopics.size() > maxCameras_)
        limitedTopics.resize(maxCameras_);

    while (providers_.size() > limitedTopics.size())
    {
        delete providers_.back();
        providers_.pop_back();
    }

    while (providers_.size() < limitedTopics.size())
    {
        QString providerId = QString("cam_%1").arg(providers_.size());
        auto *provider = new CamProvider();
        engine_->addImageProvider(providerId, provider);
        providers_.push_back(provider);
    }

    cameraList_.clear();

    for (size_t i = 0; i < limitedTopics.size(); ++i)
    {
        QString providerId = QString("cam_%1").arg(i);
        CamProvider *provider = providers_[i];

        auto sub = this->create_subscription<sensor_msgs::msg::Image>(
            limitedTopics[i], 10,
            [this, i](const std::shared_ptr<const sensor_msgs::msg::Image> &msg)
            {
                try
                {
                    auto cvimg = cv_bridge::toCvCopy(msg, "bgr8")->image;
                    // auto cv_ptr = cv_bridge::toCvShare(msg, sensor_msgs::image_encodings::BGR8);
                    // cv::Mat cvimg = cv_ptr->image;
                    QImage qimg(cvimg.data, cvimg.cols, cvimg.rows, cvimg.step, QImage::Format_BGR888);
                    providers_[i]->setImage(qimg.copy());
                }
                catch (const std::exception &e)
                {
                    RCLCPP_ERROR(this->get_logger(), "Image error for cam %lu: %s", i, e.what());
                }
            });

        subs_.push_back(sub);

        QVariantMap cam;
        cam["name"] = QString("Camera %1").arg(i + 1);
        cam["topic"] = QString::fromStdString(limitedTopics[i]);
        cam["providerId"] = providerId;
        cameraList_.append(cam);
    }

    emit cameraListChanged();
}

QStringList CamNode::getAvailableImageTopics()
{
    QStringList result;
    auto topics_and_types = this->get_topic_names_and_types();
    for (const auto &pair : topics_and_types)
    {
        const auto &topic_name = pair.first;
        const auto &types = pair.second;
        for (const auto &type : types)
        {
            if (type == "sensor_msgs/msg/Image")
            {
                result << QString::fromStdString(topic_name);
            }
        }
    }
    return result;
}

void CamNode::fetchAvailableTopicsAsync()
{
    // Prevent concurrent discovery
    bool expected = false;
    if (!fetchingTopics_.compare_exchange_strong(expected, true))
        return;

    // Run discovery in background thread to avoid blocking UI
    std::thread([this]() {
        QStringList result;
        try {
            auto topics_and_types = this->get_topic_names_and_types();
            for (const auto &pair : topics_and_types) {
                for (const auto &type : pair.second) {
                    if (type == "sensor_msgs/msg/Image")
                        result << QString::fromStdString(pair.first);
                }
            }
        } catch (...) {}

        fetchingTopics_ = false;
        // Emit via queued connection — safe to cross thread boundary
        emit availableTopicsChanged(result);
    }).detach();
}


void CamNode::updateCameraTopic(int index, const QString &newTopic)
{
    if (index < 0 || index >= static_cast<int>(subs_.size()))
        return;

    // Guard: skip empty or invalid topic names
    if (newTopic.isEmpty()) {
        RCLCPP_WARN(this->get_logger(), "updateCameraTopic: empty topic for index %d, skipped", index);
        return;
    }

    subs_[index].reset();

    auto sub = this->create_subscription<sensor_msgs::msg::Image>(
        newTopic.toStdString(), 10,
        [this, index](const std::shared_ptr<const sensor_msgs::msg::Image> &msg)
        {
            try
            {
                auto cvimg = cv_bridge::toCvCopy(msg, "bgr8")->image;
                QImage qimg(cvimg.data, cvimg.cols, cvimg.rows, cvimg.step, QImage::Format_BGR888);
                providers_[index]->setImage(qimg.copy());
            }
            catch (const std::exception &e)
            {
                RCLCPP_ERROR(this->get_logger(), "Image error: %s", e.what());
            }
        });

    subs_[index] = sub;

    QVariantMap cam = cameraList_[index].toMap();
    cam["topic"] = newTopic;
    cameraList_[index] = cam;

    emit cameraListChanged();
    
    // Auto-save topic selections when changed
    saveTopicsToFile();
}

void CamNode::refreshTopics()
{
    QStringList qTopics = getAvailableImageTopics();

    if (qTopics.size() > maxCameras_)
        qTopics = qTopics.mid(0, maxCameras_);

    std::vector<std::string> topics;
    for (const QString &qstr : qTopics)
        topics.push_back(qstr.toStdString());

    setup(topics);
}

QVariantList CamNode::cameraList() const
{
    return cameraList_;
}

// Save current topic selections to file
void CamNode::saveTopicsToFile()
{
    if (configFilePath_.empty()) {
        RCLCPP_WARN(this->get_logger(), "Config file path not set, cannot save");
        return;
    }

    std::ofstream file(configFilePath_);
    if (!file.is_open()) {
        RCLCPP_ERROR(this->get_logger(), "Failed to open config file for writing: %s", configFilePath_.c_str());
        return;
    }

    // Write each camera's topic
    for (const auto &cam : cameraList_) {
        QVariantMap camMap = cam.toMap();
        QString topic = camMap["topic"].toString();
        file << topic.toStdString() << std::endl;
    }

    file.close();
    RCLCPP_INFO(this->get_logger(), "Saved %zu camera topics to %s", cameraList_.size(), configFilePath_.c_str());
}

// Load topic selections from file
std::vector<std::string> CamNode::loadTopicsFromFile()
{
    std::vector<std::string> topics;

    if (configFilePath_.empty()) {
        RCLCPP_WARN(this->get_logger(), "Config file path not set, cannot load");
        return topics;
    }

    std::ifstream file(configFilePath_);
    if (!file.is_open()) {
        RCLCPP_INFO(this->get_logger(), "Config file not found (first run?), will use defaults");
        return topics;
    }

    std::string line;
    while (std::getline(file, line)) {
        if (!line.empty()) {
            topics.push_back(line);
        }
    }

    file.close();
    RCLCPP_INFO(this->get_logger(), "Loaded %zu camera topics from %s", topics.size(), configFilePath_.c_str());
    return topics;
}

// Public method callable from QML
void CamNode::saveTopicSelections()
{
    saveTopicsToFile();
}

// Public method callable from QML - loads and applies saved topics
void CamNode::loadTopicSelections()
{
    auto topics = loadTopicsFromFile();
    if (!topics.empty()) {
        setup(topics);
        RCLCPP_INFO(this->get_logger(), "Restored camera topics from config");
    } else {
        // If no saved config, auto-discover topics
        refreshTopics();
    }
}

