#pragma once

#include <cstdint>
#include <memory>
#include <vector>

#include "base_detector.h"
#include "field_color_detector.h"
#include "linesegment.h"
#include "point_2d.h"

namespace htwk {

#define maxEdgesPerScanline 24  // to reduce memory amount and cpu time

struct Scanline {
    int vx, vy;
    int edgeCnt;
    int edgesX[maxEdgesPerScanline];
    int edgesY[maxEdgesPerScanline];
    int edgesIntensity[maxEdgesPerScanline];
    int regionsCy[maxEdgesPerScanline];
    int regionsCb[maxEdgesPerScanline];
    int regionsCr[maxEdgesPerScanline];
    bool regionsIsGreen[maxEdgesPerScanline];
    bool regionsIsWhite[maxEdgesPerScanline];
    int link[maxEdgesPerScanline];
};

class RegionClassifier : protected BaseDetector {
private:
    void classifyGreenRegions(Scanline *sl, FieldColorDetector *field) __attribute__((nonnull));
    void classifyWhiteRegions(Scanline *sl) __attribute__((nonnull));
    bool addEdge(Image* img, Scanline *scanline, int xPeak, int yPeak, int edgeIntensity,
                 bool optimize) const __attribute__((nonnull));

    // TODO: We need a nicer architecture for this so we can unify the 3 scan methods.
    void scan(Image* img, int xPos, int yPos, FieldColorDetector *field, Scanline *scanline) const
            __attribute__((nonnull));
    void scan_avg_x(Image* img, int xPos, int yPos, FieldColorDetector *field,
                    Scanline *scanline) const __attribute__((nonnull));
    void scan_avg_y(Image* img, int xPos, int yPos, FieldColorDetector *field,
                    Scanline *scanline) const __attribute__((nonnull));
    point_2d getGradientVector(int x, int y, int lineWidth, Image* img) __attribute__((nonnull));
    void getColorsFromRegions(Image* img, Scanline *sl, int dirX, int dirY) const
            __attribute__((nonnull));
    void addSegments(const std::vector<Scanline>* scanlines, int scanlineCnt, Image* img) __attribute__((nonnull));

    std::vector<Scanline> scanVertical;
    std::vector<Scanline> scanHorizontal;

// between 8 for Tape-Lines and up to 32 for artificial grass
#ifdef WEBOTS
    int tEdge = 32;  // minimal edge-intensity
#else
    int tEdge = 20;  // minimal edge-intensity
#endif
    int maxEdgesInLine = 2;  // because of multiple edges in one lineregion
    int greenRegionColorDist = 674;
    int maxLineBorder = 6;  // maximal distance (px) between
    int lineRegionsCnt;
    static const int matchRadius = 2;
    int pattern[matchRadius * 2 + 1];

public:
    const int lineSpacing;
    static constexpr int searchRadius = 2;
    static constexpr int searchLen = 8;

    std::vector<LineSegment *> lineSegments;

    RegionClassifier(const RegionClassifier &cpy) = delete;
    explicit RegionClassifier();
    RegionClassifier(RegionClassifier &) = delete;
    RegionClassifier(RegionClassifier &&) = delete;
    RegionClassifier &operator=(const RegionClassifier &cpy) = delete;
    RegionClassifier &operator=(RegionClassifier &&cpy) = delete;
    ~RegionClassifier();

    void proceed(Image* img, FieldColorDetector *field) __attribute__((nonnull));
    int getScanVerticalSize() {
        return width / lineSpacing;
    }
    int getScanHorizontalSize() {
        return height / lineSpacing;
    }
    std::vector<LineSegment *> getLineSegments(const std::vector<int> &fieldborder);
};

}  // namespace htwk
