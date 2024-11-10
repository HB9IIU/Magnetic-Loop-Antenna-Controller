#include "hb9iiuProgressBar.h"

// Constructor
HB9IIUProgressBar::HB9IIUProgressBar(Arduino_GFX *gfx, int width, int height)
    : gfx(gfx), numRectangles(60), barHeight(height) {
    rectWidth = width / numRectangles; // Calculate the rectangle width based on the total width of the progress bar
}

// Function to draw the outline of the progress bar in grey
void HB9IIUProgressBar::drawProgressBarOutline(int x, int y) {
    uint16_t greyColor = gfx->color565(192, 192, 192); // Define a lighter grey color
    gfx->drawRect(x, y, 300, barHeight, greyColor); // Draw the outline in grey
}

// Function to interpolate colors between red, yellow, and green
uint16_t HB9IIUProgressBar::interpolateColor(int progress) {
    int red, green, blue = 0; // Initialize color components

    if (progress <= 80) {
        // Red to Yellow transition (0% to 80%)
        red = 255; // Full red
        green = (progress * 255) / 80; // Gradually increase green to 255 at 80%
    } else {
        // Yellow to Green transition (80% to 100%)
        red = 255 - ((progress - 80) * 255) / 20; // Gradually decrease red to 0 at 100%
        green = 255; // Full green
    }

    return gfx->color565(red, green, blue); // Return the RGB color in 565 format
}

// Function to fill the progress bar using small rectangles with gradient colors
void HB9IIUProgressBar::fillProgressBar(int x, int y, int progress) {
    int filledRectCount = (progress * numRectangles) / 100; // Calculate how many rectangles to fill

    // Draw filled rectangles
    for (int i = 0; i < filledRectCount; i++) {
        uint16_t color = interpolateColor((i * 100) / numRectangles); // Get color for current rectangle
        gfx->fillRect(x + i * rectWidth + 1, y + 1, rectWidth - 2, barHeight - 2, color); // Adjust for border
    }
}

// Function to show progress over a specified y position
void HB9IIUProgressBar::showProgress(int totalTime, int y) {
    int x = 10; // X position

    // Clear the display area for the progress bar only at the beginning
    gfx->fillRect(x, y, 300, barHeight, BLACK); // Clear previous progress area
    drawProgressBarOutline(x, y); // Draw the outline in grey

    unsigned long startTime = millis(); // Record the start time
    unsigned long currentTime;

    // Fill the progress bar until the total time is reached
    do {
        currentTime = millis(); // Get the current time
        int elapsedTime = currentTime - startTime; // Calculate elapsed time
        int progress = map(elapsedTime, 0, totalTime, 0, 100); // Map elapsed time to progress (0-100)

        // Ensure progress does not exceed 100%
        if (progress > 100) {
            progress = 100;
        }

        // Fill the progress bar
        fillProgressBar(x, y, progress); 

        delay(100); // Update every 100 ms
    } while (currentTime - startTime < totalTime); // Loop until total time is reached

    // Wait for an additional 500 ms after reaching 100%
    delay(500); 

    // Clear the progress bar area after waiting
    gfx->fillRect(x, y, 300, barHeight, BLACK);
}
