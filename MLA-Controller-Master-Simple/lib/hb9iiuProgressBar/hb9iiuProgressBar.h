#ifndef HB9IIUPROGRESSBAR_H
#define HB9IIUPROGRESSBAR_H

#include <Arduino_GFX_Library.h>

class HB9IIUProgressBar {
public:
    HB9IIUProgressBar(Arduino_GFX *gfx, int width = 300, int height = 30);
    void showProgress(int totalTime, int y);
    
private:
    void drawProgressBarOutline(int x, int y);
    void fillProgressBar(int x, int y, int progress);
    uint16_t interpolateColor(int progress);

    Arduino_GFX *gfx;  // Pointer to the graphics object
    const int numRectangles;
    const int barHeight;
    int rectWidth;
};

#endif
