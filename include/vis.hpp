#pragma once
#include "raylib.h"
#include "rlgl.h"
#include <cmath>
#include <common/mavlink.h>
#include <sstream>
#include <vector>

struct AttitudeFrame {
  uint32_t time_boot_ms;
  float roll;
  float pitch;
  float yaw;
};

class Vis {
public:
  float lerp(float a, float b, float t) { return a + t * (b - a); }

  std::vector<AttitudeFrame> collect_attitude_frames(std::istream &input) {
    std::vector<AttitudeFrame> frames;
    mavlink_message_t msg;
    mavlink_status_t status;
    uint8_t byte;

    while (input.read(reinterpret_cast<char *>(&byte), 1)) {
      if (mavlink_parse_char(MAVLINK_COMM_0, byte, &msg, &status)) {
        if (msg.msgid == MAVLINK_MSG_ID_ATTITUDE) {
          mavlink_attitude_t att;
          mavlink_msg_attitude_decode(&msg, &att);
          frames.push_back({att.time_boot_ms, att.roll, att.pitch, att.yaw});
        }
      }
    }
    return frames;
  }

  void visualize_attitude(std::vector<AttitudeFrame> &frames) {

    bool interp = false; // temp maybe i don't want to add this. Not sure.
    const int screenWidth = 800;
    const int screenHeight = 600;

    InitWindow(screenWidth, screenHeight, "MAVLink Attitude Playback");
    Camera3D camera = {0};
    camera.position = {5.0f, 5.0f, 5.0f};
    camera.target = {0.0f, 0.0f, 0.0f};
    camera.up = {0.0f, 1.0f, 0.0f};
    camera.fovy = 45.0f;
    camera.projection = CAMERA_PERSPECTIVE;

    SetTargetFPS(60);

    size_t index = 0;
    double start_time = GetTime();

    while (!WindowShouldClose() && index < frames.size()) {
      double elapsed_ms = (GetTime() - start_time) * 1000.0;

      while (index + 1 < frames.size() &&
             frames[index + 1].time_boot_ms < elapsed_ms) {
        ++index;
      }

      AttitudeFrame &frame = frames[index];
      if (interp) {
        double elapsed_ms = (GetTime() - start_time) * 1000.0;

        // Advance index to next frame, keeping [index, index+1] surrounding
        // current time
        while (index + 1 < frames.size() &&
               frames[index + 1].time_boot_ms < elapsed_ms) {
          ++index;
        }

        frame = frames[index];

        // Interpolate to next frame if possible
        // dumb interp, creates a bouncy effect not the best.
        if (index + 1 < frames.size()) {
          const AttitudeFrame &a = frames[index];
          const AttitudeFrame &b = frames[index + 1];

          float dt = b.time_boot_ms - a.time_boot_ms;
          float t = (elapsed_ms - a.time_boot_ms) / dt;

          frame.roll = lerp(a.roll, b.roll, t);
          frame.pitch = lerp(a.pitch, b.pitch, t);
          frame.yaw = lerp(a.yaw, b.yaw, t);
        }
      }

      BeginDrawing();
      ClearBackground(RAYWHITE);

      BeginMode3D(camera);
      DrawGrid(10, 1.0f);

      // Apply rotation using quaternions?
      Vector3 cubePos = {0.0f, 0.0f, 0.0f};
      Vector3 cubeSize = {1.0f, 1.0f, 1.0f};

      rlPushMatrix();
      rlTranslatef(cubePos.x, cubePos.y, cubePos.z);
      rlRotatef(frame.yaw * RAD2DEG, 0, 1, 0);
      rlRotatef(frame.pitch * RAD2DEG, 1, 0, 0);
      rlRotatef(frame.roll * RAD2DEG, 0, 0, 1);

      DrawCube({0, 0, 0}, cubeSize.x, cubeSize.y, cubeSize.z, RED);
      DrawCubeWires({0, 0, 0}, cubeSize.x, cubeSize.y, cubeSize.z, BLACK);
      rlPopMatrix();

      EndMode3D();

      DrawText(TextFormat("Frame %zu / %zu", index, frames.size()), 10, 10, 20,
               DARKGRAY);
      EndDrawing();
    }
    CloseWindow();
  }
};
