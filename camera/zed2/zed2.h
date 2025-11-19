#include <thread>

#include "../camera_connector.h"

namespace sl {
class Camera;
class Resolution;
}  // namespace sl

class Zed2 : public htwk::CameraConnector {
public:
    Zed2();
    ~Zed2();

    void start() override;
    void stop() override;

private:
    std::shared_ptr<Image> get_image() override;
    bool running = false;

    std::shared_ptr<sl::Camera> zed;
    std::thread cam_thread;

    std::shared_ptr<sl::Resolution> default_image_size;
    float fx = 0;
    float fy = 0;
    float cx = 0;
    float cy = 0;
};
