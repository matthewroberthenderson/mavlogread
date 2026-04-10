#pragma once
#include "raylib.h"
#include "raymath.h"
#include "rlgl.h"
#include <common/mavlink.h>
#include <iostream>
#include <vector>

// Combined frame for all telemetry
struct TelemetryFrame {
    uint32_t time_boot_ms;
    float roll, pitch, yaw;
    Vector3 localPos; 
};

class Vis {
public:
    std::vector<TelemetryFrame> collect_telemetry(std::istream &input) {
        std::vector<TelemetryFrame> frames;
        mavlink_message_t msg;
        mavlink_status_t status;
        uint8_t byte;

        float curRoll = 0, curPitch = 0, curYaw = 0;
        int32_t lat0 = 0, lon0 = 0;
        bool originSet = false;

        while (input.read(reinterpret_cast<char *>(&byte), 1)) {
            if (mavlink_parse_char(MAVLINK_COMM_0, byte, &msg, &status)) {
                switch (msg.msgid) {
                    case MAVLINK_MSG_ID_ATTITUDE: {
                        mavlink_attitude_t att;
                        mavlink_msg_attitude_decode(&msg, &att);
                        curRoll = att.roll;
                        curPitch = att.pitch;
                        curYaw = att.yaw;
                        break;
                    }
                    case MAVLINK_MSG_ID_GLOBAL_POSITION_INT: {
                        mavlink_global_position_int_t pos;
                        mavlink_msg_global_position_int_decode(&msg, &pos);

                        if (!originSet && pos.lat != 0) {
                            lat0 = pos.lat;
                            lon0 = pos.lon;
                            originSet = true;
                        }

                        // Mapping: Lon -> X, Alt -> Y, Lat -> -Z
                        float posZ = (float)(pos.lat - lat0) * 0.0111319f; 
                        float posX = (float)(pos.lon - lon0) * 0.0111319f;
                        float posY = (float)(pos.relative_alt) / 1000.0f;

                        frames.push_back({pos.time_boot_ms, curRoll, curPitch, curYaw, {posX, posY, -posZ}});
                        break;
                    }
                }
            }
        }
        return frames;
    }

    void visualize(const std::vector<TelemetryFrame> &frames) {
        if (frames.empty()) return;

        InitWindow(1280, 720, "MAVLink 3D Visualizer - Orbit Controls");
        
        Camera3D camera = { 0 };
        camera.position = (Vector3){ 15.0f, 15.0f, 15.0f }; // Initial distance
        camera.target = (Vector3){ 0.0f, 0.0f, 0.0f };    // Looking at aircraft
        camera.up = (Vector3){ 0.0f, 1.0f, 0.0f };
        camera.fovy = 45.0f;
        camera.projection = CAMERA_PERSPECTIVE;

        SetTargetFPS(60);
        
        double startSystemTime = GetTime();
        uint32_t startLogTime = frames[0].time_boot_ms;

        while (!WindowShouldClose()) {
            // 1. Update Playback Timing
            double elapsedMs = (GetTime() - startSystemTime) * 1000.0;
            uint32_t playbackTime = startLogTime + (uint32_t)elapsedMs;

            size_t idx = 0;
            while (idx + 1 < frames.size() && frames[idx + 1].time_boot_ms < playbackTime) idx++;
            
            if (idx + 1 >= frames.size()) break;

            const auto &f1 = frames[idx];
            const auto &f2 = frames[idx + 1];
            float t = (float)(playbackTime - f1.time_boot_ms) / (float)(f2.time_boot_ms - f1.time_boot_ms);

            // 2. Interpolate State
            Vector3 currentPos = Vector3Lerp(f1.localPos, f2.localPos, t);
            Quaternion qInterp = QuaternionSlerp(
                QuaternionFromEuler(f1.pitch, f1.yaw, f1.roll),
                QuaternionFromEuler(f2.pitch, f2.yaw, f2.roll), 
                t
            );
            Matrix matRotation = QuaternionToMatrix(qInterp);

            // 3. Update Camera Logic
            // UpdateCamera automatically handles Mouse Wheel (Zoom) and Mouse Drag (Orbit)
            // Use CAMERA_THIRD_PERSON to follow a target while allowing rotation around it
            UpdateCamera(&camera, CAMERA_THIRD_PERSON);
            
            // Re-sync camera target to aircraft position so it follows movement
            camera.target = currentPos;

            BeginDrawing();
                ClearBackground(SKYBLUE);
                BeginMode3D(camera);
                    
                    DrawGrid(100, 10.0f);
                    
                    rlPushMatrix();
                        rlTranslatef(currentPos.x, currentPos.y, currentPos.z);
                        rlMultMatrixf(MatrixToFloat(matRotation));
                        
                        // Airplane Model
                        DrawCube({ 0, 0, 0 }, 0.6f, 0.4f, 2.0f, RED);      // Body
                        DrawCube({ 0, 0.1f, 0.2f }, 3.5f, 0.1f, 0.6f, GRAY); // Wings
                        DrawCube({ 0, 0.2f, -0.8f }, 1.2f, 0.1f, 0.4f, GRAY); // Tail wings
                        DrawCubeWires({ 0, 0, 0 }, 0.6f, 0.4f, 2.0f, BLACK);
                    rlPopMatrix();

                EndMode3D();

                // UI
                DrawRectangle(10, 10, 320, 120, Fade(BLACK, 0.4f));
                DrawText("CONTROLS:", 20, 20, 10, GOLD);
                DrawText("- Right Click + Drag: Orbit", 20, 35, 10, WHITE);
                DrawText("- Mouse Wheel: Zoom", 20, 50, 10, WHITE);
                DrawText(TextFormat("Altitude: %.1f m", currentPos.y), 20, 75, 20, WHITE);
                DrawFPS(1180, 10);
            EndDrawing();
        }
        CloseWindow();
    }
};