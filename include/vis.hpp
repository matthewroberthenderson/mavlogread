#pragma once
#include "raylib.h"
#include "raymath.h"
#include "rlgl.h"
#include <cmath>
#include <common/mavlink.h>
#include <sstream>
#include <vector>

const int groundSize = 100;     // in meters
const float tileSize = 2.0f;    // each tile is 2x2 meters


struct StateFrame {
    uint32_t time_boot_ms;
    float roll, pitch, yaw;
    float x = 0.0f, y = 0.0f, z = 0.0f; // NED position in meters
    float alt = 0.0f;
    bool has_position = false;
};


class Vis {
public:
  float lerp(float a, float b, float t) { return a + t * (b - a); }

  std::vector<StateFrame> collect_state_frames(std::istream& input) {
    std::vector<StateFrame> frames;
    mavlink_message_t msg;
    mavlink_status_t status;
    uint8_t byte;

    StateFrame current;

    while (input.read(reinterpret_cast<char*>(&byte), 1)) {
        if (mavlink_parse_char(MAVLINK_COMM_0, byte, &msg, &status)) {
            switch (msg.msgid) {
                case MAVLINK_MSG_ID_ATTITUDE: {
                    mavlink_attitude_t att;
                    mavlink_msg_attitude_decode(&msg, &att);
                    current.time_boot_ms = att.time_boot_ms;
                    current.roll = att.roll;
                    current.pitch = att.pitch;
                    current.yaw = att.yaw;
                    frames.push_back(current);
                    break;
                }
                case MAVLINK_MSG_ID_LOCAL_POSITION_NED: {
                    mavlink_local_position_ned_t pos;
                    mavlink_msg_local_position_ned_decode(&msg, &pos);
                    current.x = pos.x;
                    current.y = pos.y;
                    current.z = pos.z;
                    current.has_position = true;
                    break;
                }
            }
        }
    }

    return frames;
}

  void visualize_attitude(std::vector<StateFrame> &frames) {

    bool interp = true; // temp maybe i don't want to add this. Not sure.
    const int screenWidth = 800;
    const int screenHeight = 600;

    InitWindow(screenWidth, screenHeight, "MAVLink Attitude Playback");
    Camera3D camera = {0};
    camera.position = {5.0f, 5.0f, 5.0f};
    camera.target = {0.0f, 0.0f, 0.0f};
    camera.up = {0.0f, 1.0f, 0.0f};
    camera.fovy = 45.0f;
    camera.projection = CAMERA_PERSPECTIVE;



    float camYaw = 0.0f;
    float camPitch = 20.0f;
    float camDistance = 10.0f;
    bool isDragging = false;
    Vector2 lastMouse = { 0, 0 };


    SetTargetFPS(60);

    size_t index = 0;
    double start_time = GetTime();

    while (!WindowShouldClose() && index < frames.size()) {


      BeginDrawing();
      ClearBackground(RAYWHITE);
      
      double elapsed_ms = (GetTime() - start_time) * 1000.0;

      while (index + 1 < frames.size() &&
        frames[index + 1].time_boot_ms < elapsed_ms) {
        ++index;
      }


      StateFrame &frame = frames[index];
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
          const StateFrame &a = frames[index];
          const StateFrame &b = frames[index + 1];

          float dt = b.time_boot_ms - a.time_boot_ms;
          float t = (elapsed_ms - a.time_boot_ms) / dt;

          frame.roll = lerp(a.roll, b.roll, t);
          frame.pitch = lerp(a.pitch, b.pitch, t);
          frame.yaw = lerp(a.yaw, b.yaw, t);

          frame.x = lerp(a.x, b.x, t);
          frame.y = lerp(a.y, b.y, t);
          frame.z = lerp(a.z, b.z, t);

        }
      }
      
      // mouse button
      if (IsMouseButtonPressed(MOUSE_LEFT_BUTTON)) {
        isDragging = true;
        lastMouse = GetMousePosition();
      }
      if (IsMouseButtonReleased(MOUSE_LEFT_BUTTON)) {
          isDragging = false;
      }

      if (isDragging) {
          Vector2 currentMouse = GetMousePosition();
          Vector2 delta = Vector2Subtract(currentMouse, lastMouse);

          camYaw   += delta.x * 0.3f;
          camPitch += delta.y * 0.3f;

          camPitch = Clamp(camPitch, -89.0f, 89.0f);
          lastMouse = currentMouse;
      }


      // Convert spherical coordinates to cartesian
      float yawRad = camYaw * DEG2RAD;
      float pitchRad = camPitch * DEG2RAD;

    Vector3 offset = {
        camDistance * cosf(pitchRad) * sinf(yawRad),
        camDistance * sinf(pitchRad),
        camDistance * cosf(pitchRad) * cosf(yawRad)
    };



      camDistance -= GetMouseWheelMove() * 1.0f;
      camDistance = Clamp(camDistance, 2.0f, 50.0f);




      BeginMode3D(camera);
      DrawGrid(10, 1.0f);

      // Apply rotation using quaternions?
      Vector3 cubePos = {0.0f, 0.0f, 0.0f};
      Vector3 cubeSize = {1.0f, 1.0f, 1.0f};

      rlPushMatrix();
      // I'm doing the conversion everywhere here explicitly so it's more obvious.
      // Kind of helps me remember what frame we are in.
      // NED y = East ->  Raylib X (left-right)
      // NED z = Down ->  Raylib Y (up-down)
      // NED x = North -> Raylib Z (forward-back)
      // So that would be something like...
      // Vector3 cubePos = {
      //     frame.y,       // East -> X
      //     -frame.z,      // -Down -> Y (↑)
      //     -frame.x       // -North -> Z (→)
      // };
      
      int originX = static_cast<int>(floor(frame.y / tileSize)) * tileSize;
      int originZ = static_cast<int>(floor(-frame.x / tileSize)) * tileSize;

      for (int x = -groundSize/2; x < groundSize/2; ++x) {
          for (int z = -groundSize/2; z < groundSize/2; ++z) {
              int worldX = originX + x * tileSize;
              int worldZ = originZ + z * tileSize;

              Color color = ((x + z) % 2 == 0) ? LIGHTGRAY : DARKGRAY;

              DrawCube({ (float)worldX, 0.0f, (float)worldZ }, tileSize, 0.1f, tileSize, color);
          }
      }

      rlTranslatef(frame.y, -frame.z, -frame.x);
      rlRotatef(frame.yaw * RAD2DEG, 0, 1, 0);
      rlRotatef(frame.pitch * RAD2DEG, 1, 0, 0);
      rlRotatef(frame.roll * RAD2DEG, 0, 0, 1);



      DrawCube({0, 0, 0}, cubeSize.x, cubeSize.y, cubeSize.z, RED);
      DrawCubeWires({0, 0, 0}, cubeSize.x, cubeSize.y, cubeSize.z, BLACK);
      rlPopMatrix();

      camera.position = Vector3Add({frame.y, -frame.z, -frame.x}, offset); //Vector3Add({frame.x, frame.y, frame.z}, offset);
      camera.target = {frame.y, -frame.z, -frame.x};
      camera.up = { 0.0f, 1.0f, 0.0f };

      EndMode3D();

      DrawText(TextFormat("Frame %zu / %zu", index, frames.size()), 10, 10, 20,
               DARKGRAY);
      EndDrawing();
    }
    CloseWindow();
  }
};
