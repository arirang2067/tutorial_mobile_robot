#pragma once
#include <boost/asio.hpp>
#include <cstdint>
#include <vector>
#include <array>
#include <mutex>
#include <functional>
#include <string>

class SerialCom
{
public:
    // 프로토콜: [RMID][PCID][ID][PID][LEN][DATA..][CHK], CHK = ~sum+1
    static constexpr uint8_t  kIdAll        = 0xFE;
    static constexpr size_t   kMaxDataSize  = 64;     // 여유있게
    static constexpr size_t   kRxBufSize    = 256;    // 1프레임 여유

    struct Params {
        std::string device{""};
        unsigned int baudrate{0};
        uint8_t pc_id{0};    // MID_PC = 172
        uint8_t mdt_id{0};   // 183
        uint8_t mdui_id{0};  // 184
        std::vector<uint8_t> allowed_ids{1, kIdAll};
    };

    struct Frame {
        uint8_t  rmid{0};
        uint8_t  pcid{0};
        uint8_t  id{0};
        uint8_t  pid{0};
        uint8_t  length{0};
        std::array<uint8_t, kMaxDataSize> data{};
        uint8_t  checksum{0};
    };

public:
    SerialCom();
    explicit SerialCom(const Params& params);

    void SetParams(const Params& params);
    bool OpenPort();
    bool OpenPort(const std::string& device, unsigned int baudrate);
    void ClosePort();
    bool IsOpen() const;

    // 송신: 쓰기 구간만 락, 내부 버퍼 사용
    bool SendPacket(uint8_t rmid, uint8_t pcid, uint8_t id,
                    uint8_t pid, const uint8_t* data, uint8_t len);

    // 폴링 1회: 읽기(락) → 락 해제 → 파싱(무락) → on_frame 콜백
    void ReadAndParse(const std::function<void(const Frame&)>& on_frame);

    // 가끔 호출 가능한 가용 바이트
    std::size_t AvailableBytes();

private:
    // 내부
    std::size_t AvailableBytesUnlocked(); // 포트가 열려있다는 가정하에 사용
    void ResetParser();
    void FeedByte(uint8_t data, const std::function<void(const Frame&)>& on_frame);

    static uint8_t CalculateChecksum(const uint8_t* p, std::size_t len);
    static bool IsAllowedMotorId(uint8_t id, const std::vector<uint8_t>& allowed);

private:
    Params params_;
    mutable std::mutex port_mutex_;
    boost::asio::io_service io_;
    boost::asio::serial_port port_;

    // TX/RX 작업 버퍼
    std::array<uint8_t, 512> tx_buf_{};
    std::array<uint8_t, kRxBufSize> rx_buf_{};

    // 수신 파서 상태
    uint32_t rcv_step_{0};
    uint8_t  checksum_sum_{0};
    uint16_t max_data_num_{0};
    uint16_t data_num_{0};
    std::size_t packet_pos_{0};
};
