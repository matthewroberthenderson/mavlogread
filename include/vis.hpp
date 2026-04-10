#pragma once
#include "raylib.h"
#include "raymath.h"
#include "rlgl.h"
#include <common/mavlink.h>
#include <iostream>
#include <vector>

struct TelemetryFrame {
    uint32_t time_boot_ms;
    float roll, pitch, yaw;
    Vector3 localPos; // X, Y, Z in meters from starting point
};

class Vis {
public:
    std::vector<TelemetryFrame> collect_telemetry(std::istream &input) {
        std::vector<TelemetryFrame> frames;
        mavlink_message_t msg;
        mavlink_status_t status;
        uint8_t byte;

        // State trackers
        float curRoll = 0, curPitch = 0, curYaw = 0;
        int32_t lat0 = 0, lon0 = 0, alt0 = 0;
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
                            alt0 = pos.relative_alt;
                            originSet = true;
                        }

                        // Convert GPS to Meters
                        // Lat/Lon are in 1e7 degrees. 1 deg ~= 111319 meters.
                        // I think this is right but realising these convertions would be different
                        // for some autopilots. 
                        float posZ = (float)(pos.lat - lat0) * 0.0111319f; 
                        float posX = (float)(pos.lon - lon0) * 0.0111319f;
                        float posY = (float)(pos.relative_alt) / 1000.0f; // mm to m

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

        InitWindow(1280, 720, "MAVLink LOG PLAYBACK");
        
        Camera3D camera = { 0 };
        camera.position = (Vector3){ 10.0f, 10.0f, 10.0f };
        camera.target = (Vector3){ 0.0f, 0.0f, 0.0f };
        camera.up = (Vector3){ 0.0f, 1.0f, 0.0f };
        camera.fovy = 45.0f;
        camera.projection = CAMERA_PERSPECTIVE;

        SetTargetFPS(60);
        double startSystemTime = GetTime();
        uint32_t startLogTime = frames[0].time_boot_ms;

        while (!WindowShouldClose()) {
            double elapsedMs = (GetTime() - startSystemTime) * 1000.0;
            uint32_t playbackTime = startLogTime + (uint32_t)elapsedMs;

            // Find frames for interp
            size_t idx = 0;
            while (idx + 1 < frames.size() && frames[idx + 1].time_boot_ms < playbackTime) idx++;
            
            if (idx + 1 >= frames.size()) break; // End of log

            const auto &f1 = frames[idx];
            const auto &f2 = frames[idx + 1];
            float t = (float)(playbackTime - f1.time_boot_ms) / (float)(f2.time_boot_ms - f1.time_boot_ms);

            // Interp Position
            Vector3 currentPos = Vector3Lerp(f1.localPos, f2.localPos, t);

            // SLERP Rotation (needs work)
            Quaternion q1 = QuaternionFromEuler(f1.pitch, f1.yaw, f1.roll);
            Quaternion q2 = QuaternionFromEuler(f2.pitch, f2.yaw, f2.roll);
            Quaternion qInterp = QuaternionSlerp(q1, q2, t);
            Matrix matRotation = QuaternionToMatrix(qInterp);

            // Camera follow
            camera.target = currentPos;
            camera.position = Vector3Add(currentPos, (Vector3){ 15.0f, 10.0f, 15.0f });

            BeginDrawing();
                ClearBackground(SKYBLUE);
                BeginMode3D(camera);
                    
                    DrawGrid(100, 10.0f); // Large ground grid
                    
                    rlPushMatrix();
                        // Move to interp pos / rot
                        rlTranslatef(currentPos.x, currentPos.y, currentPos.z);
                        rlMultMatrixf(MatrixToFloat(matRotation));
                        
                        // Dummy airframe object
                        DrawCube({ 0, 0, 0 }, 0.8f, 0.4f, 2.5f, RED);   // Fuselage
                        DrawCube({ 0, 0, 0.2f }, 4.0f, 0.1f, 0.7f, GRAY); // Wings
                        DrawCubeWires({ 0, 0, 0 }, 0.8f, 0.4f, 2.5f, BLACK);
                    rlPopMatrix();

                EndMode3D();

                DrawRectangle(10, 10, 250, 90, Fade(BLACK, 0.3f));
                DrawText(TextFormat("ALT: %.2f m", currentPos.y), 20, 20, 20, WHITE);
                DrawText(TextFormat("X: %.2f Y: %.2f", currentPos.x, currentPos.z), 20, 45, 20, WHITE);
                DrawFPS(1180, 10);
            EndDrawing();
        }
        CloseWindow();
    }
};
