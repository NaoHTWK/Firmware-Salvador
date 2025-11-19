#pragma once

#include <color.h>
#include <pub_sub/object_hypothesis.h>
#include <range_check.h>
#include <arm_neon.h>

#include <chrono>
#include <cmath>
#include <cstdint>
#include <utility>
#include <image.h>

namespace htwk {

class Timer {
public:
    Timer(std::string name, long timeout_ms) : name(std::move(name)), timeout_ms(timeout_ms) {}
    ~Timer() {
        std::chrono::time_point<std::chrono::system_clock> now = std::chrono::system_clock::now();
        if (now > start + std::chrono::milliseconds(timeout_ms))
            printf("%s took %ldms!\n", name.c_str(),
                   std::chrono::duration_cast<std::chrono::milliseconds>(now - start).count());
    }

private:
    std::string name;
    long timeout_ms;
    std::chrono::time_point<std::chrono::system_clock> start = std::chrono::system_clock::now();
};

class BaseDetector {
protected:
    static constexpr int width = Image::WIDTH;
    static constexpr int height = Image::HEIGHT;

public:
static inline void addYCbCr(const uint8_t* const p, int& Y, int& Cb, int& Cr) {
        const uint8_t y = p[0];
        const uint8_t u = p[1];
        const uint8_t v = p[2];
        Y += y;
        Cb += u;
        Cr += v;
    }

    // just use getYCbCr()
    inline color getColor(const uint8_t* const img, int x, int y) const
    __attribute__((nonnull)) __attribute__((pure)) {

        const uint8_t* const p = &img[(y * width + x) * 3];
        const uint8_t yc = p[0];
        const uint8_t u = p[1];
        const uint8_t v = p[2];

        return color{yc,u,v};
    }

    // converts BGRA bytes to YCbCr and sets ref Y Cb Cr
    static inline void getYCbCr(const uint8_t* const p, int& Y, int& Cb, int& Cr) {
        Y = p[0];
        Cb = p[1];
        Cr = p[2];
    }

    inline uint8_t getY(Image* img, int x, int y) const
    __attribute__((nonnull)) {
       	return img->data_yuv[(y * width + x) * 3 + 0];
    }

    inline uint8_t getCb(Image* img, int x, int y) const
    __attribute__((nonnull)) {
        return img->data_yuv[(y * width + x) * 3 + 1];
    }

    inline uint8_t getCr(Image* img, int x, int y) const
    __attribute__((nonnull)) {
		return img->data_yuv[(y * width + x) * 3 + 2];
    }


    BaseDetector() {}
};

}  // namespace htwk
