#pragma once
#include <memory>
class Image;

namespace htwk{
    class Camera;
    
    class CameraConnector {
    public:
        
        virtual void start() = 0;
        virtual void stop() = 0;
        virtual ~CameraConnector() = default;
        
    protected:
        friend class Camera;
        virtual std::shared_ptr<Image> get_image() = 0;
    };
    
}
