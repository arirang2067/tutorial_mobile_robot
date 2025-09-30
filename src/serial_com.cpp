#include "tutorial_mobile_robot/serial_com.hpp"

#include <cstring>
#include <iostream>
#include <algorithm>
#include <sys/ioctl.h>   // AvailableBytes()
#include <unistd.h>

namespace {
constexpr int MD_PROTOCOL_POS_PID        = 3;
constexpr int MD_PROTOCOL_POS_DATA_LEN   = 4;
constexpr int MD_PROTOCOL_POS_DATA_START = 5;

// 허용 ID 검사(빈 벡터면 {1, 0xFE} 허용)
inline bool IsAllowedMotorId(uint8_t id, const std::vector<uint8_t>& allowed)
{
    if (allowed.empty()) return (id == 1 || id == SerialCom::kIdAll);
    return std::find(allowed.begin(), allowed.end(), id) != allowed.end();
}
} // namespace

SerialCom::SerialCom()
: port_(io_)
{
    ResetParser();
}

SerialCom::SerialCom(const Params& params)
: params_(params), port_(io_)
{
    ResetParser();
}

void SerialCom::SetParams(const Params& params)
{
    std::lock_guard<std::mutex> lk(port_mutex_);
    params_ = params;
}

bool SerialCom::OpenPort()
{
    return OpenPort(params_.device, params_.baudrate);
}

bool SerialCom::OpenPort(const std::string& device, unsigned int baudrate)
{
    std::lock_guard<std::mutex> lk(port_mutex_);
    boost::system::error_code ec;

    if (port_.is_open()) {
        port_.close(ec);
    }

    port_.open(device, ec);
    if (ec) {
        std::cerr << "[SerialCom] open(" << device << ") failed: " << ec.message() << std::endl;
        return false;
    }

    port_.set_option(boost::asio::serial_port_base::baud_rate(baudrate), ec);
    if (ec) { std::cerr << "[SerialCom] set baud failed: " << ec.message() << std::endl; goto fail; }

    port_.set_option(boost::asio::serial_port_base::character_size(8), ec);
    if (ec) { std::cerr << "[SerialCom] set char size failed: " << ec.message() << std::endl; goto fail; }

    port_.set_option(boost::asio::serial_port_base::stop_bits(
        boost::asio::serial_port_base::stop_bits::one), ec);
    if (ec) { std::cerr << "[SerialCom] set stop bits failed: " << ec.message() << std::endl; goto fail; }

    port_.set_option(boost::asio::serial_port_base::parity(
        boost::asio::serial_port_base::parity::none), ec);
    if (ec) { std::cerr << "[SerialCom] set parity failed: " << ec.message() << std::endl; goto fail; }

    port_.set_option(boost::asio::serial_port_base::flow_control(
        boost::asio::serial_port_base::flow_control::none), ec);
    if (ec) { std::cerr << "[SerialCom] set flow control failed: " << ec.message() << std::endl; goto fail; }

    return true;

fail:
    port_.close(ec);
    return false;
}

void SerialCom::ClosePort()
{
    std::lock_guard<std::mutex> lk(port_mutex_);
    boost::system::error_code ec;
    if (port_.is_open()) {
        port_.close(ec);
        if (ec) {
            std::cerr << "[SerialCom] close failed: " << ec.message() << std::endl;
        }
    }
}

bool SerialCom::IsOpen() const
{
    std::lock_guard<std::mutex> lk(port_mutex_);
    return port_.is_open();
}

uint8_t SerialCom::CalculateChecksum(const uint8_t* p, std::size_t len)
{
    uint8_t sum = 0;
    for (std::size_t i = 0; i < len; ++i) sum += p[i];
    return static_cast<uint8_t>(~sum + 1); // 2의 보수
}

bool SerialCom::SendPacket(uint8_t rmid, uint8_t pcid, uint8_t id,
                           uint8_t pid, const uint8_t* data, uint8_t len)
{
    if (len > kMaxDataSize) {
        std::cerr << "[SerialCom] payload too long: " << (int)len << " > " << kMaxDataSize << std::endl;
        return false;
    }

    std::lock_guard<std::mutex> lk(port_mutex_);
    if (!port_.is_open()) {
        std::cerr << "[SerialCom] port is not open\n";
        return false;
    }

    std::size_t pos = 0;
    tx_buf_[pos++] = rmid;
    tx_buf_[pos++] = pcid;
    tx_buf_[pos++] = id;
    tx_buf_[pos++] = pid;
    tx_buf_[pos++] = len;

    if (len > 0 && data) {
        std::memcpy(tx_buf_.data() + pos, data, len);
        pos += len;
    }

    // ★ 수정: 계산과 pos 증가를 분리
    uint8_t cks = CalculateChecksum(tx_buf_.data(), pos);
    tx_buf_[pos++] = cks;

    boost::system::error_code ec;
    auto n = boost::asio::write(port_, boost::asio::buffer(tx_buf_.data(), pos), ec);
    if (ec || n != pos) {
        std::cerr << "[SerialCom] write failed: " << ec.message()
                  << " (wrote " << n << "/" << pos << ")\n";
        return false;
    }
    return true;
}

std::size_t SerialCom::AvailableBytes()
{
    std::lock_guard<std::mutex> lk(port_mutex_);
    return AvailableBytesUnlocked();
}

std::size_t SerialCom::AvailableBytesUnlocked()
{
    if (!port_.is_open()) return 0;

    int bytes = 0;
    if (ioctl(port_.native_handle(), FIONREAD, &bytes) == 0 && bytes > 0) {
        return static_cast<std::size_t>(bytes);
    }
    return 0;
}

void SerialCom::ResetParser()
{
    rcv_step_     = 0;
    checksum_sum_ = 0;
    max_data_num_ = 0;
    data_num_     = 0;
    packet_pos_   = 0;
}

void SerialCom::FeedByte(uint8_t data, const std::function<void(const Frame&)>& on_frame)
{
    switch (rcv_step_)
    {
    case 0: // [0] PCID(=우리)
        if (data == params_.pc_id) {
            packet_pos_   = 0;
            checksum_sum_ = data;
            rx_buf_[packet_pos_++] = data;
            rcv_step_ = 1;
        } else {
            packet_pos_ = 0;
        }
        break;

    case 1: // [1] 송신측 ID가 MDT/MDUI인지 확인(수신 프레임 기준)
        if (data == params_.mdt_id || data == params_.mdui_id) {
            checksum_sum_ += data;
            rx_buf_[packet_pos_++] = data;
            rcv_step_ = 2;
        } else {
            ResetParser();
        }
        break;

    case 2: // [2] 내부 모터/그룹 ID
        if (IsAllowedMotorId(data, params_.allowed_ids)) {
            checksum_sum_ += data;
            rx_buf_[packet_pos_++] = data;
            rcv_step_ = 3;
        } else {
            ResetParser();
        }
        break;

    case 3: // [3] PID
        checksum_sum_ += data;
        rx_buf_[packet_pos_++] = data;
        rcv_step_ = 4;
        break;

    case 4: // [4] LEN
        checksum_sum_ += data;
        rx_buf_[packet_pos_++] = data;
        max_data_num_ = data;
        data_num_ = 0;
        rcv_step_ = 5;
        break;

    case 5: // [5..] DATA
        checksum_sum_ += data;
        rx_buf_[packet_pos_++] = data;

        if (++data_num_ >= kMaxDataSize) {
            // 안전장치: 데이터 과다 → 리셋
            ResetParser();
            break;
        }
        if (data_num_ >= max_data_num_) {
            rcv_step_ = 6;
        }
        break;

    case 6: // [last] CHECKSUM
        checksum_sum_ += data;
        rx_buf_[packet_pos_++] = data;

        if (checksum_sum_ == 0) {
            // 유효 프레임 → 콜백 전달
            Frame f;
            f.rmid     = rx_buf_[0]; // 수신 프레임에서는 [0]이 PCID였지만,
            f.pcid     = rx_buf_[1]; // 기존 코드 레이아웃과 호환을 위해
            f.id       = rx_buf_[2];
            f.pid      = rx_buf_[MD_PROTOCOL_POS_PID];
            f.length   = rx_buf_[MD_PROTOCOL_POS_DATA_LEN];
            if (f.length > 0) {
                std::memcpy(f.data.data(),
                            rx_buf_.data() + MD_PROTOCOL_POS_DATA_START,
                            std::min<std::size_t>(f.length, kMaxDataSize));
            }
            f.checksum = rx_buf_[packet_pos_ - 1];

            if (on_frame) on_frame(f);
        }
        // 성공/실패 모두 파서 초기화
        ResetParser();
        break;

    default:
        ResetParser();
        break;
    }
}

void SerialCom::ReadAndParse(const std::function<void(const Frame&)>& on_frame)
{
    std::lock_guard<std::mutex> lk(port_mutex_);
    if (!port_.is_open()) return;

    std::size_t n_avail = AvailableBytesUnlocked();
    if (n_avail == 0) return;

    const std::size_t chunk = std::min<std::size_t>(n_avail, 256);
    std::vector<uint8_t> tmp(chunk);

    boost::system::error_code ec;
    auto n = boost::asio::read(port_, boost::asio::buffer(tmp.data(), tmp.size()), ec);
    if (ec) {
        std::cerr << "[SerialCom] read failed: " << ec.message() << std::endl;
        return;
    }

    for (std::size_t i = 0; i < n; ++i) {
        FeedByte(tmp[i], on_frame);
    }
}
