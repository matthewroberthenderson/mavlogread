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

                        // COORDINATE MAPPING (NED to OpenGL Y-Up):
                        // North (Lat) -> -Z 
                        // East (Lon)  -> +X
                        // Down (Alt)  -> +Y (using relative_alt which is Up)
                        float posX = (float)(pos.lon - lon0) * 0.0111319f; 
                        float posY = (float)(pos.relative_alt) / 1000.0f;
                        float posZ = (float)(pos.lat - lat0) * -0.0111319f; 

                        frames.push_back({pos.time_boot_ms, curRoll, curPitch, curYaw, {posX, posY, posZ}});
                        break;
                    }
                }
            }
        }
        return frames;
    }

    void visualize(const std::vector<TelemetryFrame> &frames) {
    if (frames.empty()) return;

    InitWindow(1280, 720, "MAVLink 3D - Chase Cam Mode");
    
    Camera3D camera = { 0 };
    // Initial offset: 10 meters back, 5 meters up
    camera.position = (Vector3){ 0.0f, 5.0f, 10.0f }; 
    camera.target = (Vector3){ 0.0f, 0.0f, 0.0f };
    camera.up = (Vector3){ 0.0f, 1.0f, 0.0f };
    camera.fovy = 45.0f;
    camera.projection = CAMERA_PERSPECTIVE;

    SetTargetFPS(60);
    
    double startSystemTime = GetTime();
    uint32_t startLogTime = frames[0].time_boot_ms;
    Vector3 lastPos = frames[0].localPos;

    while (!WindowShouldClose()) {
        double elapsedMs = (GetTime() - startSystemTime) * 1000.0;
        uint32_t playbackTime = startLogTime + (uint32_t)elapsedMs;

        size_t idx = 0;
        while (idx + 1 < frames.size() && frames[idx + 1].time_boot_ms < playbackTime) idx++;
        if (idx + 1 >= frames.size()) break;

        const auto &f1 = frames[idx];
        const auto &f2 = frames[idx + 1];
        float t = (float)(playbackTime - f1.time_boot_ms) / (float)(f2.time_boot_ms - f1.time_boot_ms);

        Vector3 currentPos = Vector3Lerp(f1.localPos, f2.localPos, t);
        
        // Calculate the movement delta since the last frame
        Vector3 diff = Vector3Subtract(currentPos, lastPos);
        
        // --- CAMERA LOGIC: LOCK TO VEHICLE ---
        // We move the camera's position by the SAME amount the aircraft moved.
        // This maintains the "Constant Offset" even before user input.
        camera.position = Vector3Add(camera.position, diff);
        camera.target = currentPos;
        lastPos = currentPos;

        // Now allow Orbit (Right Click) and Zoom (Wheel) to modify that position
        UpdateCamera(&camera, CAMERA_THIRD_PERSON);

        // --- RENDER ---
        BeginDrawing();
            ClearBackground(SKYBLUE);
            BeginMode3D(camera);
                
                DrawGrid(100, 10.0f);
                
                // Draw a simple "Ground" so you can see movement
                DrawCircleV({0,0}, 1000, Fade(DARKGREEN, 0.5f));

                rlPushMatrix();
                    rlTranslatef(currentPos.x, currentPos.y, currentPos.z);
                    
                    Quaternion qInterp = QuaternionSlerp(
                        QuaternionFromEuler(f1.pitch, -f1.yaw, f1.roll),
                        QuaternionFromEuler(f2.pitch, -f2.yaw, f2.roll), 
                        t
                    );
                    rlMultMatrixf(MatrixToFloat(QuaternionToMatrix(qInterp)));
                    
                    // Local Axis Helpers
                    DrawLine3D({0,0,0}, {5,0,0}, RED);   // Right
                    DrawLine3D({0,0,0}, {0,5,0}, GREEN); // Up
                    DrawLine3D({0,0,0}, {0,0,5}, BLUE);  // Forward (Nose)
                    
                    // Airplane Mesh (Longer on Z axis)
                    DrawCube({ 0, 0, 0 }, 0.5f, 0.5f, 2.0f, RED); 
                    DrawCube({ 0, 0, 0.3f }, 3.0f, 0.1f, 0.6f, LIGHTGRAY);
                rlPopMatrix();

            EndMode3D();

            // UI
            DrawRectangle(10, 10, 250, 60, Fade(BLACK, 0.5f));
            DrawText(TextFormat("ALT: %.2f m", currentPos.y), 20, 20, 20, WHITE);
            DrawText("Right-Click to Orbit / Wheel to Zoom", 20, 45, 10, GRAY);
            
        EndDrawing();
    }
    CloseWindow();
}
};