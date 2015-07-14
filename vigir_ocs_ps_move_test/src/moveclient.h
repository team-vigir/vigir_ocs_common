/*********************************************************************
 * Software License Agreement (BSD License)
 *
 *  Copyright (c) 2013-2015, TORC Robotics, LLC ( Team ViGIR )
 *  All rights reserved.
 *
 *  Redistribution and use in source and binary forms, with or without
 *  modification, are permitted provided that the following conditions
 *  are met:
 *
 *   * Redistributions of source code must retain the above copyright
 *     notice, this list of conditions and the following disclaimer.
 *   * Redistributions in binary form must reproduce the above
 *     copyright notice, this list of conditions and the following
 *     disclaimer in the documentation and/or other materials provided
 *     with the distribution.
 *   * Neither the name of Team ViGIR, TORC Robotics, nor the names of its
 *     contributors may be used to endorse or promote products derived
 *     from this software without specific prior written permission.
 *
 *  THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS
 *  "AS IS" AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT
 *  LIMITED TO, THE IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS
 *  FOR A PARTICULAR PURPOSE ARE DISCLAIMED. IN NO EVENT SHALL THE
 *  COPYRIGHT OWNER OR CONTRIBUTORS BE LIABLE FOR ANY DIRECT, INDIRECT,
 *  INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES (INCLUDING,
 *  BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES;
 *  LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER
 *  CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT
 *  LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN
 *  ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE
 *  POSSIBILITY OF SUCH DAMAGE.
 *********************************************************************/
//@TODO_ADD_AUTHOR_INFO
#ifndef INC_MOVE_CLIENT_H
#define INC_MOVE_CLIENT_H


#ifdef __cplusplus
extern "C" {
#endif


// Defines
#define MAX_UDP_MSG_SIZE (65535 - (20 + 8)) // Max size of UDP packet payload: 0xffff - (sizeof(IP Header) + sizeof(UDP Header))
#define IMAGE_BUFFER_SIZE (60 * 1024)

#define CAMERA_FRAME_SPLIT_FORMAT_JPG 0x1
#define MAX_CAMERA_FRAME_SLICES 8

#define MOVE_PACKET_MAGIC 0xff0000dd
#define MOVE_PACKET_CODE_STANDARD 0x1
#define MOVE_PACKET_CODE_CAMERA_FRAME_SLICE 0x2

#define MOVE_SERVER_MAX_GEMS 4
#define MOVE_SERVER_MAX_NAVS 7
#define MOVE_TRANSFER_TIMEOUT 20000  // microseconds
#define MOVE_CLIENT_OK 0
#define MOVE_CLIENT_ERROR -1
#define CELL_PAD_MAX_CODES 64


typedef enum MOVE_CLIENT_REQUESTS {
    MOVE_CLIENT_REQUEST_INIT = 0x0,
    MOVE_CLIENT_REQUEST_PAUSE = 0x1,
    MOVE_CLIENT_REQUEST_RESUME = 0x2,
    MOVE_CLIENT_REQUEST_DELAY_CHANGE = 0x3,  // payload is ms between packets
    MOVE_CLIENT_REQUEST_PREPARE_CAMERA = 0x4,  // payload is maximum exposure(int) and image_quality(float)

    MOVE_CLIENT_REQUEST_POINTER_SET_LEFT = 0x7,  // payload is gem number
    MOVE_CLIENT_REQUEST_POINTER_SET_RIGHT = 0x8,  // payload is gem number
    MOVE_CLIENT_REQUEST_POINTER_SET_BOTTOM = 0x9,  // payload is gem number
    MOVE_CLIENT_REQUEST_POINTER_SET_TOP = 0x10,  // payload is gem number
    MOVE_CLIENT_REQUEST_POINTER_ENABLE = 0x11,  // payload is gem number
    MOVE_CLIENT_REQUEST_POINTER_DISABLE = 0x12,  // payload is gem number
    MOVE_CLIENT_REQUEST_CONTROLLER_RESET = 0x13,  // payload is gem number
    MOVE_CLIENT_REQUEST_POSITION_POINTER_SET_LEFT = 0x14,  // payload is gem number
    MOVE_CLIENT_REQUEST_POSITION_POINTER_SET_RIGHT = 0x15,  // payload is gem number
    MOVE_CLIENT_REQUEST_POSITION_POINTER_SET_BOTTOM = 0x16,  // payload is gem number
    MOVE_CLIENT_REQUEST_POSITION_POINTER_SET_TOP = 0x17,  // payload is gem number
    MOVE_CLIENT_REQUEST_POSITION_POINTER_ENABLE = 0x18,  // payload is gem number
    MOVE_CLIENT_REQUEST_POSITION_POINTER_DISABLE = 0x19,  // payload is gem number

    MOVE_CLIENT_REQUEST_FORCE_RGB = 0x20,  // payload is gem number followed by r,g,b floats
    MOVE_CLIENT_REQUEST_SET_RUMBLE = 0x21,  // payload is gem number followed by rumble value
    MOVE_CLIENT_REQUEST_TRACK_HUES = 0x22,  // payload is four hue track values

    MOVE_CLIENT_REQUEST_CAMERA_FRAME_DELAY_CHANGE = 0x23,  // acceptable delay: 16ms to 255 ms
    MOVE_CLIENT_REQUEST_CAMERA_FRAME_CONFIG_NUM_SLICES = 0x24,  // acceptable range: 1 to 8
    MOVE_CLIENT_REQUEST_CAMERA_FRAME_PAUSE = 0x25,
    MOVE_CLIENT_REQUEST_CAMERA_FRAME_RESUME = 0x26,

} MOVE_CLIENT_REQUESTS;


#ifndef ntohll
#define ntohll(x) (((unsigned long long)(ntohl((unsigned int)x)) << 32) | (unsigned long long)(ntohl((unsigned int)((unsigned long long)x >> 32))))
#endif


// Type translations
typedef unsigned char uint8_t;
typedef unsigned short uint16_t;
typedef unsigned int uint32_t;
//typedef unsigned long long uint64_t;
typedef int int32_t;
typedef long long system_time_t;
typedef float float4[4];


// Move structures
typedef struct _NavPadInfo {
    uint32_t port_status[MOVE_SERVER_MAX_NAVS];

} NavPadInfo;

typedef struct _NavPadData {
    int32_t len;
    uint16_t button[CELL_PAD_MAX_CODES];

} NavPadData;

typedef struct _MovePadData {
    uint16_t digital_buttons;
    uint16_t analog_trigger;

} MovePadData;

typedef struct _MoveState {
    float4 pos;
    float4 vel;
    float4 accel;
    float4 quat;
    float4 angvel;
    float4 angaccel;
    float4 handle_pos;
    float4 handle_vel;
    float4 handle_accel;
    MovePadData pad;
    system_time_t timestamp;
    float temperature;
    float camera_pitch_angle;
    uint32_t tracking_flags;

} MoveState;

typedef struct _MoveImageState {
    system_time_t frame_timestamp;
    system_time_t timestamp;
    float u;
    float v;
    float r;
    float projectionx;
    float projectiony;
    float distance;
    unsigned char visible;
    unsigned char r_valid;

} MoveImageState;

typedef struct _MoveSphereState {
    uint32_t tracking;
    uint32_t tracking_hue;
    float r;
    float g;
    float b;

} MoveSphereState;

typedef struct _MoveCameraState {
    int32_t exposure;
    float exposure_time;
    float gain;
    float pitch_angle;
    float pitch_angle_estimate;

} MoveCameraState;

typedef struct _MovePointerState
{
    uint32_t valid;
    float normalized_x;
    float normalized_y;

} MovePointerState;

typedef struct _MovePositionPointerState
{
    uint32_t valid;
    float normalized_x;
    float normalized_y;

} MovePositionPointerState;


// Header structure
typedef struct _MoveServerPacketHeader
{
    uint32_t magic;
    uint32_t move_me_server_version;
    uint32_t packet_code;
    uint32_t packet_length;
    uint32_t packet_index;

} MoveServerPacketHeader;


// Standard packet structures
typedef struct _MoveServerConfig
{
    int32_t num_image_slices;
    int32_t image_slice_format;

} MoveServerConfig;

typedef struct _MoveConnectionConfig
{
    uint32_t ms_delay_between_standard_packets;
    uint32_t ms_delay_between_camera_frame_packets;
    uint32_t camera_frame_packet_paused;

} MoveConnectionConfig;

typedef struct _MoveStatus
{
    uint32_t connected;
    uint32_t code;
    unsigned long long flags;

} MoveStatus;

typedef struct _MoveServerPacket
{
    MoveServerPacketHeader header;
    MoveServerConfig server_config;
    MoveConnectionConfig client_config;
    MoveStatus status[MOVE_SERVER_MAX_GEMS];
    MoveState state[MOVE_SERVER_MAX_GEMS];
    MoveImageState image_state[MOVE_SERVER_MAX_GEMS];
    MovePointerState pointer_state[MOVE_SERVER_MAX_GEMS];
    NavPadInfo pad_info;
    NavPadData pad_data[MOVE_SERVER_MAX_NAVS];
    MoveSphereState sphere_state[MOVE_SERVER_MAX_GEMS];
    MoveCameraState camera_state;
    MovePositionPointerState position_pointer_state[MOVE_SERVER_MAX_GEMS];

} MoveServerPacket;


// Camera packet structure
typedef struct _MoveServerCameraFrameSlicePacket {
    MoveServerPacketHeader header;
    unsigned char slice_num;
    unsigned char num_slices;
    unsigned char format;
    unsigned char buffer[IMAGE_BUFFER_SIZE];

} MoveServerCameraFrameSlicePacket;


// Request packet structures
typedef struct _MoveServerRequestPacketHeader
{
    uint32_t client_request;
    uint32_t payload_size;

} MoveServerRequestPacketHeader;

typedef struct _MoveServerRequestPacket
{
    MoveServerRequestPacketHeader header;
    uint32_t payload;

} MoveServerRequestPacket;

typedef struct _MoveServerRequestPacketRumble
{
    MoveServerRequestPacketHeader header;
    uint32_t gem_num;
    uint32_t rumble;

} MoveServerRequestPacketRumble;

typedef struct _MoveServerRequestPacketForceRGB {
    MoveServerRequestPacketHeader header;
    uint32_t gem_num;
    float r;
    float g;
    float b;

} MoveServerRequestPacketForceRGB;

typedef struct _MoveServerRequestPacketTrackHues {
    MoveServerRequestPacketHeader header;
    uint32_t req_hue_gem_0;
    uint32_t req_hue_gem_1;
    uint32_t req_hue_gem_2;
    uint32_t req_hue_gem_3;

} MoveServerRequestPacketTrackHues;

typedef struct _MoveServerRequestPacketPrepareCamera {
    MoveServerRequestPacketHeader header;
    uint32_t max_exposure;
    float image_quality;

} MoveServerRequestPacketPrepareCamera;


// Callback function pointers
typedef int (*UpdateSuccess)(MoveServerPacket *);
typedef int (*UpdateFailure)(int);

typedef int (*UpdateCameraSuccess)(MoveServerCameraFrameSlicePacket *);
typedef int (*UpdateCameraFailure)(int);


// Struct to hold callback and gem state to update
typedef struct _MoveStateDeferred {
    UpdateSuccess update_success;
    UpdateFailure update_failure;
    UpdateCameraSuccess update_camera_success;
    UpdateCameraFailure update_camera_failure;
    MoveServerPacket *move_server_packet;
    MoveServerCameraFrameSlicePacket *move_server_camera_frame_slice_packet;

} MoveStateDeferred;


// Prototypes
int updateMoveState(MoveStateDeferred *);

int startTransfer(void);
int stopTransfer(void);

float htonf(float);
float ntohf(float);

int serverConnect(const char *, const char *, MoveStateDeferred *);
int serverDisconnect(void);
int pause(void);
int resume(void);
int pauseCamera(void);
int resumeCamera(void);
int updateDelay(uint32_t);
int cameraUpdateDelay(uint32_t);
int rumble(uint32_t, uint32_t);
int forceRGB(uint32_t, float, float, float);
int trackHues(uint32_t, uint32_t, uint32_t, uint32_t);
int prepareCamera(uint32_t, float);
int cameraSetNumSlices(uint32_t);

#ifdef __cplusplus
}
#endif

#endif  // ... INC_MOVE_CLIENT_H
