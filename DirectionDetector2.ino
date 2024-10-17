/**
   ESP32-CAM Directional Motion Detection Modified for Centroid Testing
   Licensed under the Attribution-ShareAlike 4.0 International (CC BY-SA 4.0)
   http://creativecommons.org/licenses/by-sa/4.0/
*/

#include <vector>
#include <queue>
#include <cmath>

#define CAMERA_MODEL_AI_THINKER // Select the camera model
#include "camera_pins.h"
#include "esp_camera.h"

#define FRAME_SIZE FRAMESIZE_QVGA // Framebuffer size
#define WIDTH 320                 // Resolution Width
#define HEIGHT 240                // Resolution height
#define BLOCK_SIZE 4              // Size of each sensor block on the display
#define W (WIDTH / BLOCK_SIZE)
#define H (HEIGHT / BLOCK_SIZE)
#define BLOCK_DIFF_THRESHOLD 1.5
#define IMAGE_DIFF_THRESHOLD 0.1
#define DEBUG 0    // Debugging flag
#define INFO 1     // Information flag
#define FLASH_PIN 4 // Pin of ESP32 Flash (LED)
#define FLASH_MODE 0 // 0 = off, 1 = flash, 2 = steady

uint16_t prev_frame[H][W] = {0};
uint16_t current_frame[H][W] = {0};

#define LINE_POSITION (H / 2)    // Middle of the frame
#define DISTANCE_THRESHOLD 5.0   // Threshold for matching objects
#define MAX_MISSING_FRAMES 5     // Frames before an object is considered lost
#define MAX_OBJECTS 10           // Maximum number of objects to track

struct TrackedObject {
    int id;
    float x;
    float y;
    float prev_x;
    float prev_y;
    bool counted;         // Whether the object has been counted crossing the line
    int missingFrames;    // Number of consecutive frames the object was not detected
};

TrackedObject objects[MAX_OBJECTS];
int objectCount = 0;
int nextObjectID = 1;
// int peopleCount = 0; // Counter for people entering/exiting (commented out)

void setup_camera();
bool capture_still();
bool motion_detect();
void update_frame();
void print_frame(uint16_t frame[H][W]);
std::vector<std::vector<std::pair<int, int>>> getBlobs(
    std::vector<std::pair<int, int>> &movingBlocks);

void setup() {
    Serial.begin(115200);
    Serial.println("Begin Setup...");

    pinMode(FLASH_PIN, OUTPUT);

    setup_camera();

    Serial.println("End Setup...");
}

void loop() {
    if (FLASH_MODE == 2)
        digitalWrite(FLASH_PIN, HIGH);

    if (!capture_still()) {
    #if INFO
        Serial.println("Failed capture");
    #endif
        return;
    }

    if (motion_detect()) {
        // People count is commented out for testing centroid locations
        // Serial.print("People Count: ");
        // Serial.println(peopleCount);
    }

    update_frame();
}

bool capture_still() {
    if (FLASH_MODE == 1)
        digitalWrite(FLASH_PIN, HIGH);

    camera_fb_t *frame_buffer = esp_camera_fb_get();

    if (FLASH_MODE == 1)
        digitalWrite(FLASH_PIN, LOW);

    if (!frame_buffer)
        return false;

    // Clear current_frame
    memset(current_frame, 0, sizeof(current_frame));

    // Downsample image into blocks
    for (uint32_t i = 0; i < WIDTH * HEIGHT; i++) {
        const uint16_t x = i % WIDTH;
        const uint16_t y = i / WIDTH;
        const uint8_t block_x = x / BLOCK_SIZE;
        const uint8_t block_y = y / BLOCK_SIZE;
        const uint8_t pixel = frame_buffer->buf[i];
        current_frame[block_y][block_x] += pixel;
    }

    // Average pixels in block
    for (int y = 0; y < H; y++)
        for (int x = 0; x < W; x++)
            current_frame[y][x] /= (BLOCK_SIZE * BLOCK_SIZE);

    #if DEBUG
        Serial.println("Current frame:");
        print_frame(current_frame);
        Serial.println("---------------");
    #endif

    esp_camera_fb_return(frame_buffer); // Free the camera buffer

    return true;
}

bool motion_detect() {
    std::vector<std::pair<int, int>> movingBlocks;

    // Detect moving blocks
    for (uint16_t y = 0; y < H; y++) {
        for (uint16_t x = 0; x < W; x++) {
            float current = current_frame[y][x];
            float prev = prev_frame[y][x];
            float delta = prev > 0 ? abs(current - prev) / prev : 0;

            if (delta >= BLOCK_DIFF_THRESHOLD) {
                movingBlocks.push_back(std::make_pair(x, y));
            }
        }
    }

    if (movingBlocks.empty())
        return false;

    // Get blobs from moving blocks
    std::vector<std::vector<std::pair<int, int>>> blobs = getBlobs(movingBlocks);

    // Calculate centroids
    std::vector<std::pair<float, float>> centroids;

    for (auto &blob : blobs) {
        float sumX = 0;
        float sumY = 0;
        for (auto &point : blob) {
            sumX += point.first;
            sumY += point.second;
        }
        float centroidX = sumX / blob.size();
        float centroidY = sumY / blob.size();
        centroids.push_back(std::make_pair(centroidX, centroidY));
    }

    // Print centroid locations
    for (size_t i = 0; i < centroids.size(); i++) {
        Serial.print("Centroid ");
        Serial.print(i);
        Serial.print(": (");
        Serial.print(centroids[i].first);
        Serial.print(", ");
        Serial.print(centroids[i].second);
        Serial.println(")");
    }

    // Match centroids to existing objects
    bool objectMatched[MAX_OBJECTS] = {false};

    for (auto &centroid : centroids) {
        float minDistance = 10000;
        int matchedObjectIndex = -1;

        for (int i = 0; i < objectCount; i++) {
            float distance = sqrt(pow(objects[i].x - centroid.first, 2) +
                                  pow(objects[i].y - centroid.second, 2));

            if (distance < minDistance) {
                minDistance = distance;
                matchedObjectIndex = i;
            }
        }

        if (minDistance < DISTANCE_THRESHOLD && matchedObjectIndex != -1) {
            // Update existing object
            objects[matchedObjectIndex].prev_x = objects[matchedObjectIndex].x;
            objects[matchedObjectIndex].prev_y = objects[matchedObjectIndex].y;
            objects[matchedObjectIndex].x = centroid.first;
            objects[matchedObjectIndex].y = centroid.second;
            objects[matchedObjectIndex].missingFrames = 0;
            objectMatched[matchedObjectIndex] = true;
        } else {
            // Create new object
            if (objectCount < MAX_OBJECTS) {
                objects[objectCount].id = nextObjectID++;
                objects[objectCount].x = centroid.first;
                objects[objectCount].y = centroid.second;
                objects[objectCount].prev_x = centroid.first;
                objects[objectCount].prev_y = centroid.second;
                objects[objectCount].counted = false;
                objects[objectCount].missingFrames = 0;
                objectMatched[objectCount] = true;
                objectCount++;
            }
        }
    }

    // Update missing frames and remove lost objects
    for (int i = 0; i < objectCount; i++) {
        if (!objectMatched[i]) {
            objects[i].missingFrames++;
        } else {
            objects[i].missingFrames = 0;
        }

        if (objects[i].missingFrames > MAX_MISSING_FRAMES) {
            // Remove object
            for (int j = i; j < objectCount - 1; j++) {
                objects[j] = objects[j + 1];
            }
            objectCount--;
            i--;
        }
    }

    // Commented out the people counting logic for testing
    /*
    // Determine movement direction
    for (int i = 0; i < objectCount; i++) {
        if (!objects[i].counted) {
            if (objects[i].prev_y < LINE_POSITION && objects[i].y >= LINE_POSITION) {
                // Moved from top to bottom
                peopleCount++;
                objects[i].counted = true;
                Serial.print("Person entered. Count: ");
                Serial.println(peopleCount);
            } else if (objects[i].prev_y > LINE_POSITION && objects[i].y <= LINE_POSITION) {
                // Moved from bottom to top
                peopleCount--;
                objects[i].counted = true;
                Serial.print("Person exited. Count: ");
                Serial.println(peopleCount);
            }
        }
    }
    */

    return true;
}

void update_frame() {
    memcpy(prev_frame, current_frame, sizeof(prev_frame));
}

void print_frame(uint16_t frame[H][W]) {
    for (int y = 0; y < H; y++) {
        for (int x = 0; x < W; x++) {
            Serial.print(frame[y][x]);
            Serial.print('\t');
        }
        Serial.println();
    }
}

void setup_camera() {
    camera_config_t config;

    config.ledc_channel = LEDC_CHANNEL_0;
    config.ledc_timer   = LEDC_TIMER_0;
    config.pin_d0       = Y2_GPIO_NUM;
    config.pin_d1       = Y3_GPIO_NUM;
    config.pin_d2       = Y4_GPIO_NUM;
    config.pin_d3       = Y5_GPIO_NUM;
    config.pin_d4       = Y6_GPIO_NUM;
    config.pin_d5       = Y7_GPIO_NUM;
    config.pin_d6       = Y8_GPIO_NUM;
    config.pin_d7       = Y9_GPIO_NUM;
    config.pin_xclk     = XCLK_GPIO_NUM;
    config.pin_pclk     = PCLK_GPIO_NUM;
    config.pin_vsync    = VSYNC_GPIO_NUM;
    config.pin_href     = HREF_GPIO_NUM;
    config.pin_sscb_sda = SIOD_GPIO_NUM;
    config.pin_sscb_scl = SIOC_GPIO_NUM;
    config.pin_pwdn     = PWDN_GPIO_NUM;
    config.pin_reset    = RESET_GPIO_NUM;
    config.xclk_freq_hz = 20000000;
    config.pixel_format = PIXFORMAT_GRAYSCALE;
    config.frame_size   = FRAME_SIZE;
    config.jpeg_quality = 12;
    config.fb_count     = 1;

    // Initialize the camera
    esp_err_t err = esp_camera_init(&config);
    if (err != ESP_OK) {
        Serial.printf("Camera init failed with error 0x%x", err);
        return;
    }

    sensor_t *sensor = esp_camera_sensor_get();
    sensor->set_framesize(sensor, FRAME_SIZE);
}

std::vector<std::vector<std::pair<int, int>>> getBlobs(
    std::vector<std::pair<int, int>> &movingBlocks) {
    std::vector<std::vector<std::pair<int, int>>> blobs;
    bool visited[H][W] = {false};

    for (auto &block : movingBlocks) {
        int x = block.first;
        int y = block.second;

        if (!visited[y][x]) {
            std::vector<std::pair<int, int>> blob;
            std::queue<std::pair<int, int>> q;
            q.push(block);
            visited[y][x] = true;

            while (!q.empty()) {
                auto current = q.front();
                q.pop();
                blob.push_back(current);

                // Check neighbors
                for (int dx = -1; dx <= 1; dx++) {
                    for (int dy = -1; dy <= 1; dy++) {
                        int nx = current.first + dx;
                        int ny = current.second + dy;

                        if (nx >= 0 && nx < W && ny >= 0 && ny < H &&
                            !visited[ny][nx]) {
                            if (std::find(movingBlocks.begin(), movingBlocks.end(),
                                          std::make_pair(nx, ny)) != movingBlocks.end()) {
                                q.push(std::make_pair(nx, ny));
                                visited[ny][nx] = true;
                            }
                        }
                    }
                }
            }

            blobs.push_back(blob);
        }
    }

    return blobs;
}
