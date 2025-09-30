#ifndef TUTORIAL_MOBILE_ROBOT_SERIAL_COM_HPP_
#define TUTORIAL_MOBILE_ROBOT_SERIAL_COM_HPP_

#include <cstdint>
#include <string>
#include <functional>
#include <mutex>
#include <array>
#include <vector>
#include <boost/asio.hpp>

// 수업용 직렬통신 래퍼
// - 포트 열기/닫기
// - 패킷 전송(프로토콜 프레이밍 + 체크섬 2의 보수)
// - 수신 바이트 상태기계 파싱 + 콜백으로 프레임 전달
//
// 프로토콜 레이아웃(송/수신 동일):
//   [0] RMID(수신장치) | [1] PCID(송신장치) | [2] ID(모터ID/브로드캐스트) |
//   [3] PID | [4] LEN | [5..] DATA | [last] CHK(2의 보수; 모든 바이트 합 + CHK == 0)
//
// 주의: 함수 이름은 PascalCase, 멤버/변수는 snake_case

class SerialCom
{
public:
    // 고정 길이(기존 코드와 동일)
    static constexpr std::size_t kMaxPacketSize = 26;
    static constexpr std::size_t kMaxDataSize   = 21;

    // ID 상수(기존 코드 호환)
    static constexpr uint8_t kIdAll = 0xFE;

    // 수신 프레임 전달 타입
    struct Frame
    {
        uint8_t rmid   = 0; // 수신장치 ID(드라이버)
        uint8_t pcid   = 0; // 송신장치 ID(PC)
        uint8_t id     = 0; // 내부 모터/그룹 ID
        uint8_t pid    = 0; // 명령/데이터 구분 PID
        uint8_t length = 0; // payload 길이
        std::array<uint8_t, kMaxDataSize> data{}; // payload
        uint8_t checksum = 0; // 수신 프레임의 원본 체크섬
    };

    // 설정 파라미터
    struct Params
    {
        std::string device = "/dev/ttyUSB0";
        unsigned int baudrate = 19200;

        // 파서가 유효 프레임로 간주하기 위한 ID 규칙
        uint8_t pc_id  = 172; // MID_PC
        uint8_t mdt_id = 183; // MID_MDT
        uint8_t mdui_id= 184; // MID_MDUI

        // 수신시 허용할 모터 ID(3번째 바이트). 빈 벡터면 {1, kIdAll}을 자동 사용
        std::vector<uint8_t> allowed_ids{};
    };

public:
    SerialCom();                    // 기본 생성자
    explicit SerialCom(const Params& params);

    // 파라미터/ID 설정
    void SetParams(const Params& params);
    const Params& GetParams() const { return params_; }

    // 포트 제어
    bool OpenPort();                // params_.device/baudrate로 오픈
    bool OpenPort(const std::string& device, unsigned int baudrate);
    void ClosePort();
    bool IsOpen() const;

    // 송신(동기식)
    // - rmid: 수신 장치 ID(MDT/MDUI)
    // - pcid: 송신 장치 ID(PC)
    // - id  : 내부 모터/그룹 ID(보통 1 또는 kIdAll)
    // - pid : 명령/데이터 구분 코드
    // - data/len: payload(최대 kMaxDataSize)
    bool SendPacket(uint8_t rmid, uint8_t pcid, uint8_t id,
                    uint8_t pid, const uint8_t* data, uint8_t len);

    // 수신 + 파싱(상태기계). 필요할 때 주기적으로 호출(타이머/스레드 등)
    // on_frame(frame): 체크섬 통과한 완전한 프레임만 콜백
    void ReadAndParse(const std::function<void(const Frame&)>& on_frame);

    // 현재 포트에 읽기 가능한 바이트 수(리눅스 ioctl)
    std::size_t AvailableBytes();

private:
    // 내부 헬퍼
    static uint8_t CalculateChecksum(const uint8_t* p, std::size_t len);
    std::size_t AvailableBytesUnlocked();

    // 파서 상태 초기화
    void ResetParser();

    // 파서 상태기계(바이트 단위)
    void FeedByte(uint8_t byte, const std::function<void(const Frame&)>& on_frame);

private:
    Params params_;

    // Boost.Asio
    mutable std::mutex port_mutex_;
    boost::asio::io_service io_;
    boost::asio::serial_port port_;

    // 송신 버퍼(고정)
    std::array<uint8_t, kMaxPacketSize> tx_buf_{};

    // 파서 상태
    uint32_t rcv_step_      = 0;
    uint8_t  checksum_sum_  = 0;
    uint16_t max_data_num_  = 0;
    uint16_t data_num_      = 0;
    uint32_t packet_pos_    = 0;
    std::array<uint8_t, kMaxPacketSize> rx_buf_{};
};

#endif // TUTORIAL_MOBILE_ROBOT_SERIAL_COM_HPP_
