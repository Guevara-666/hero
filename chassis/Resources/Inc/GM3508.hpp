// GM3508.hpp
#ifndef GM3508_HPP
#define GM3508_HPP

#include <cstdint>

class GM3508 {
public:
    void Init(uint32_t id);
    ~GM3508() = default;
    uint32_t txId(void);
    uint32_t rxId(void);
    float angle(void);
    float vel(void);
    float current(void);
    float temp(void);
    float targht_speed(void);
    void setInput(float current);
    bool encode(uint8_t *data);
    bool decode(uint8_t *data);
    void setspeed(float speed);

private:
    uint32_t id_;
    float input_;
    float angle_;
    float vel_;
    float current_;
    float temp_;
    float targht_speed_;
};

// 内联函数实现
inline void GM3508::Init(uint32_t id) {
    id_ = id;
}

inline uint32_t GM3508::txId(void) {
    return id_ + 0x200;
}

inline uint32_t GM3508::rxId(void) {
    return id_ + 0x204;
}

inline float GM3508::angle(void) {
    return angle_;
}

inline float GM3508::vel(void) {
    return vel_;
}

inline float GM3508::current(void) {
    return current_;
}

inline float GM3508::temp(void) {
    return temp_;
}

inline float GM3508::targht_speed(){
    return targht_speed_;
}

inline void GM3508::setInput(float current) {
    input_ = current;
}

inline void GM3508::setspeed(float speed){
    targht_speed_ = speed;
}

bool GM3508::encode(uint8_t *data) {
    if (!data) return false;
    
    int16_t current_cmd = static_cast<int16_t>(input_);
    
    // 根据电机ID确定在数据帧中的位置
    switch(id_) {
        case 1:
            data[0] = (current_cmd >> 8) & 0xFF;
            data[1] = current_cmd & 0xFF;
            break;
        case 2:
            data[2] = (current_cmd >> 8) & 0xFF;
            data[3] = current_cmd & 0xFF;
            break;
        case 3:
            data[4] = (current_cmd >> 8) & 0xFF;
            data[5] = current_cmd & 0xFF;
            break;
        case 4:
            data[6] = (current_cmd >> 8) & 0xFF;
            data[7] = current_cmd & 0xFF;
            break;
        default:
            return false;
    }
    
    return true;
}

inline bool GM3508::decode(uint8_t *data) {
    int16_t angle_raw = (static_cast<int16_t>(data[0]) << 8) | data[1];
    int16_t vel_raw = (static_cast<int16_t>(data[2]) << 8) | data[3];
    int16_t current_raw = (static_cast<int16_t>(data[4]) << 8) | data[5];
    uint8_t temp_raw = data[6];

    angle_ = static_cast<float>(angle_raw) * (360.0f / 8192.0f);
    vel_ = static_cast<float>(vel_raw) * (1.0f / 60.0f);
    current_ = static_cast<float>(current_raw) / 1000.0f;
    temp_ = static_cast<float>(temp_raw);

    return true;
}



#endif // GM3508_HPP