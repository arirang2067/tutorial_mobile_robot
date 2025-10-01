#include "tutorial_mobile_robot/serial_com.hpp"
#include <sys/ioctl.h>
#include <unistd.h>
#include <cstring>
#include <iostream>
#include <algorithm>

namespace {
constexpr int POS_PID        = 3;
constexpr int POS_DATA_LEN   = 4;
constexpr int POS_DATA_START = 5;

inline bool IsIn(const std::vector<uint8_t>& v, uint8_t x) {
    return std::find(v.begin(), v.end(), x) != v.end();
}
} // namespace

// ─────────────────────────────────────────────────────────────────────────────
// ctor
// ─────────────────────────────────────────────────────────────────────────────
SerialCom::SerialCom() : port_(io_) { ResetParser(); }
SerialCom::SerialCom(const Params& params) : params_(params), port_(io_) { ResetParser(); }

void SerialCom::SetParams(const Params& params)
{
    std::lock_guard<std::mutex> lk(port_mutex_);
    params_ = params;
}

bool SerialCom::OpenPort() { return OpenPort(params_.device, params_.baudrate); }

bool SerialCom::OpenPort(const std::string& device, unsigned int baudrate)
{
    std::lock_guard<std::mutex> lk(port_mutex_);
    boost::system::error_code ec;

    if (port_.is_open()) port_.close(ec);

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
        if (ec) std::cerr << "[SerialCom] close failed: " << ec.message() << std::endl;
    }
}

bool SerialCom::IsOpen() const
{
    std::lock_guard<std::mutex> lk(port_mutex_);
    return port_.is_open();
}

// ─────────────────────────────────────────────────────────────────────────────
// util
// ─────────────────────────────────────────────────────────────────────────────
uint8_t SerialCom::CalculateChecksum(const uint8_t* p, std::size_t len)
{
    uint8_t sum = 0;
    for (std::size_t i = 0; i < len; ++i) sum += p[i];
    return static_cast<uint8_t>(~sum + 1);
}

bool SerialCom::IsAllowedMotorId(uint8_t id, const std::vector<uint8_t>& allowed)
{
    if (allowed.empty()) return (id == 1 || id == kIdAll);
    return IsIn(allowed, id);
}

// ─────────────────────────────────────────────────────────────────────────────
// TX: 쓰기 구간만 락
// ─────────────────────────────────────────────────────────────────────────────
bool SerialCom::SendPacket(uint8_t rmid, uint8_t pcid, uint8_t id,
                           uint8_t pid, const uint8_t* data, uint8_t len)
{
    if (len > kMaxDataSize) {
        std::cerr << "[SerialCom] payload too long: " << (int)len << " > " << kMaxDataSize << std::endl;
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

    const uint8_t cks = CalculateChecksum(tx_buf_.data(), pos);
    tx_buf_[pos++] = cks;

    std::lock_guard<std::mutex> lk(port_mutex_);
    if (!port_.is_open()) {
        std::cerr << "[SerialCom] port is not open\n";
        return false;
    }

    boost::system::error_code ec;
    auto n = boost::asio::write(port_, boost::asio::buffer(tx_buf_.data(), pos), ec);
    if (ec || n != pos) {
        std::cerr << "[SerialCom] write failed: " << ec.message()
                  << " (wrote " << n << "/" << pos << ")\n";
        return false;
    }
    return true;
}

// ─────────────────────────────────────────────────────────────────────────────
// RX: 읽기(락) → 락 해제 후 파싱(무락). TX와의 락 경합 최소화!
// ─────────────────────────────────────────────────────────────────────────────
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
            Frame f;
            f.rmid   = rx_buf_[0]; // 역사적 호환(이전 코드 의미 유지)
            f.pcid   = rx_buf_[1];
            f.id     = rx_buf_[2];
            f.pid    = rx_buf_[POS_PID];
            f.length = rx_buf_[POS_DATA_LEN];
            if (f.length > 0) {
                std::memcpy(f.data.data(),
                            rx_buf_.data() + POS_DATA_START,
                            std::min<std::size_t>(f.length, kMaxDataSize));
            }
            f.checksum = rx_buf_[packet_pos_ - 1];

            if (on_frame) on_frame(f);
        }
        ResetParser();
        break;

    default:
        ResetParser();
        break;
    }
}

void SerialCom::ReadAndParse(const std::function<void(const Frame&)>& on_frame)
{
    // 1) 잠깐만 락을 잡고 커널 버퍼에서 한 번에 읽어온다.
    std::vector<uint8_t> local_bytes;
    {
        std::lock_guard<std::mutex> lk(port_mutex_);
        if (!port_.is_open()) return;
        const std::size_t n_avail = AvailableBytesUnlocked();
        if (n_avail == 0) return;

        const std::size_t chunk = std::min<std::size_t>(n_avail, kRxBufSize);
        local_bytes.resize(chunk);
        boost::system::error_code ec;
        auto n = boost::asio::read(port_, boost::asio::buffer(local_bytes.data(), local_bytes.size()), ec);
        if (ec) {
            std::cerr << "[SerialCom] read failed: " << ec.message() << std::endl;
            return;
        }
        local_bytes.resize(n);
    }

    // 2) 락 해제 상태에서 파싱 → TX와 경합 최소화
    for (uint8_t b : local_bytes) {
        FeedByte(b, on_frame);
    }
}
