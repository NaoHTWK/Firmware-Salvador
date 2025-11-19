#include <fcntl.h>
#include <semaphore.h>
#include <sys/mman.h>
#include <sys/stat.h>
#include <unistd.h>

#include <cerrno>
#include <cstring>
#include <iostream>
#include <memory>
#include <rclcpp/rclcpp.hpp>
#include <sensor_msgs/msg/image.hpp>
#include <stdexcept>
#include <string>
#include <vector>

#include <thread>

#include "drobotics_shm_publisher/shm_util.h"

class ShmWriter {
public:
    ShmWriter(const std::string& shm_name, const std::string& sem_name, size_t size)
        : shm_name_(shm_name),
          sem_name_(sem_name),
          size_(size),
          shm_fd_(-1),
          ptr_(nullptr),
          sem_(SEM_FAILED) {

        shm_fd_ = shm_open(shm_name_.c_str(), O_CREAT | O_RDWR, 0666);
        if (shm_fd_ == -1) {
            throw std::runtime_error("shm_open failed: " + std::string(strerror(errno)));
        }

        if (ftruncate(shm_fd_, size_) == -1) {
            close(shm_fd_);
            shm_unlink(shm_name_.c_str());
            throw std::runtime_error("ftruncate failed: " + std::string(strerror(errno)));
        }

        ptr_ = mmap(0, size_, PROT_WRITE, MAP_SHARED, shm_fd_, 0);
        if (ptr_ == MAP_FAILED) {
            close(shm_fd_);
            shm_unlink(shm_name_.c_str());
            throw std::runtime_error("mmap failed: " + std::string(strerror(errno)));
        }

        sem_ = sem_open(sem_name_.c_str(), O_CREAT, 0666, 0);
        if (sem_ == SEM_FAILED) {
            munmap(ptr_, size_);
            close(shm_fd_);
            shm_unlink(shm_name_.c_str());
            throw std::runtime_error("sem_open failed: " + std::string(strerror(errno)));
        }
    }

    ~ShmWriter() {
        if (ptr_ != MAP_FAILED)
            munmap(ptr_, size_);
        if (shm_fd_ != -1)
            close(shm_fd_);
        shm_unlink(shm_name_.c_str());
        if (sem_ != SEM_FAILED) {
            sem_close(sem_);
            sem_unlink(sem_name_.c_str());
        }
    }

    void write_image(const sensor_msgs::msg::Image::SharedPtr msg) {
        if (!ptr_)
            return;

        RCLCPP_INFO(rclcpp::get_logger("shm_publisher"),
                    "Image received: %dx%d, encoding: %s",
                    msg->width, msg->height, msg->encoding.c_str());

        size_t encoding_len = msg->encoding.length() + 1;
        size_t header_size = sizeof(drobotics::ShmHeader) + encoding_len;
        if (header_size + msg->data.size() > size_) {
            RCLCPP_ERROR(rclcpp::get_logger("shm_publisher"),
                         "Image data exceeds shared memory size!");
            return;
        }

        drobotics::ShmHeader* header = static_cast<drobotics::ShmHeader*>(ptr_);
        header->timestamp_sec = msg->header.stamp.sec;
        header->timestamp_nanosec = msg->header.stamp.nanosec;
        header->width = msg->width;
        header->height = msg->height;
        header->step = msg->step;
        header->encoding_len = encoding_len;

        char* encoding_ptr = header->encoding;
        memcpy(encoding_ptr, msg->encoding.c_str(), encoding_len);

        uint8_t* data_ptr = reinterpret_cast<uint8_t*>(encoding_ptr + encoding_len);
        memcpy(data_ptr, msg->data.data(), msg->data.size());

        sem_post(sem_);
    }

private:
    std::string shm_name_;
    std::string sem_name_;
    size_t size_;
    int shm_fd_;
    void* ptr_;
    sem_t* sem_;
};

class ImageTopicPublisher : public rclcpp::Node {
public:
    ImageTopicPublisher(const std::string& node_name, const std::string& topic_name,
                        const std::string& shm_name, const std::string& sem_name,
                        const std::string& encoding, size_t image_data_size)
        : Node(node_name) {
        size_t encoding_len = encoding.length() + 1;
        size_t header_size = sizeof(drobotics::ShmHeader) + encoding_len;
        writer_ = std::make_unique<ShmWriter>(shm_name, sem_name, header_size + image_data_size);

        subscription_ = this->create_subscription<sensor_msgs::msg::Image>(
                topic_name, rclcpp::QoS(rclcpp::KeepLast(1)),
                [this](const sensor_msgs::msg::Image::SharedPtr msg) {
                    writer_->write_image(msg);
                });
        RCLCPP_INFO(this->get_logger(), "%s SHM writer created for topic %s.", node_name.c_str(),
                    topic_name.c_str());
    }

private:
    std::unique_ptr<ShmWriter> writer_;
    rclcpp::Subscription<sensor_msgs::msg::Image>::SharedPtr subscription_;
};

int main(int argc, char** argv) {
    rclcpp::init(argc, argv);

    auto depth_publisher = std::make_shared<ImageTopicPublisher>(
            "depth_image_shm_publisher",
            "/booster_camera_bridge/StereoNetNode/stereonet_depth", drobotics::DEPTH_SHM_NAME,
            drobotics::DEPTH_SEM_NAME, "mono16", 448 * 1088);

    auto rgb_publisher = std::make_shared<ImageTopicPublisher>(
            "rgb_image_shm_publisher", "/booster_camera_bridge/StereoNetNode/rectified_image",
            drobotics::RGB_SHM_NAME, drobotics::RGB_SEM_NAME, "nv12", 448 * 544 * 1.5);

    rclcpp::executors::MultiThreadedExecutor executor;
    executor.add_node(depth_publisher);
    executor.add_node(rgb_publisher);

    executor.spin();

    rclcpp::shutdown();
    return 0;
}
