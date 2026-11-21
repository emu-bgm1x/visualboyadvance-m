/* ff-bgm1x: 게임의 GB 사운드 드라이버 코드를 실행하는 작은 SM83(GB CPU) 해석기.
 * 에뮬레이터 코어와 무관하다. 롬·WRAM·HRAM·사운드 레지스터만 가진 기계에서 함수 하나를 불러 돌아올 때까지 실행한다.
 * 사운드 레지스터 쓰기는 자체 합성기(GbApuSynth)로 보낸다.
 * 실행하면서 RAM 을 어떻게 쓰는지(먼저 읽는지, 쓰는지)와 함수가 실제로 쓰는 입력 레지스터를 기록한다. */
#pragma once

#include <array>
#include <atomic>
#include <cstdint>
#include <functional>
#include <memory>
#include <utility>
#include <vector>

namespace QGBA {

class GbApuSynth;

// RAM 위치 번호: 0x0000~0x7FFF = WRAM (C000~CFFF 는 0x0000~, D000~DFFF 는 bank*0x1000~), 0x8000~0x807F = HRAM (FF80~FFFF)
constexpr int kGbRamSize = 0x8080;

// 드라이버가 RAM 을 쓰는 방식 (두 스레드가 같이 본다)
class GbDriverRamMap {
public:
	enum : uint8_t {
		ReadFirstUpdate = 1 << 0,   // 갱신 루틴이 한 번 실행 안에서 쓰기 전에 읽었다 (이전 실행에서 넘어온 값 또는 입력)
		WrittenUpdate = 1 << 1,     // 갱신 루틴이 썼다
		ReadFirstRequest = 1 << 2,  // 요청 함수가 쓰기 전에 읽었다
		WrittenRequest = 1 << 3,    // 요청 함수가 썼다
		TouchedUpdate = 1 << 4,     // 갱신 루틴이 읽거나 썼다
	};
	GbDriverRamMap() : m_flags(new std::atomic<uint8_t>[kGbRamSize]) { clear(); }
	void clear() {
		for (int i = 0; i < kGbRamSize; ++i) {
			m_flags[i].store(0, std::memory_order_relaxed);
		}
		for (auto& b : m_blocks) {
			b.store(0, std::memory_order_relaxed);
		}
		updates = 0;
	}
	uint8_t get(int i) const { return m_flags[i].load(std::memory_order_relaxed); }
	void mark(int i, uint8_t f) {
		if ((get(i) & f) != f) {
			m_flags[i].fetch_or(f, std::memory_order_relaxed);
		}
		if (f & (ReadFirstUpdate | WrittenUpdate | TouchedUpdate)) {
			m_blocks[i >> 4].store(1, std::memory_order_relaxed);
		}
	}
	// 갱신 루틴이 쓰기 전에 읽는 곳 (드라이버 상태 또는 드라이버 입력). 원본에서 여기에 쓰는 것이 요청이다
	bool relevant(int i) const { return get(i) & ReadFirstUpdate; }
	// 그림자 쪽 드라이버가 가진 값 (원본 RAM 으로 덮어쓰지 않는다)
	bool owned(int i) const {
		const uint8_t f = get(i);
		return (f & (ReadFirstUpdate | ReadFirstRequest)) && (f & (WrittenUpdate | WrittenRequest));
	}
	// 갱신 루틴이 읽기만 하는 곳 (게임이 정하는 값: 설정, 경고음 켜짐 등). 프레임마다 원본에서 가져온다
	bool updateInput(int i) const {
		const uint8_t f = get(i);
		return (f & ReadFirstUpdate) && !(f & (WrittenUpdate | WrittenRequest));
	}
	// 드라이버 RAM 영역: 갱신 루틴이 만진 16바이트 칸과, 그런 칸 사이의 64바이트 이하 틈.
	// 원본에서 여기에 쓰는 것을 요청으로 보고, 요청 함수가 여기 밖에 쓰면 게임 코드로 본다
	bool inRegion(int i) const {
		const int b = i >> 4;
		if (block(b)) {
			return true;
		}
		bool left = false, right = false;
		for (int k = 1; k <= 4 && !(left && right); ++k) {
			left = left || block(b - k);
			right = right || block(b + k);
		}
		return left && right;
	}
	// 그림자가 갱신 루틴을 돌린 횟수 (드라이버 RAM 을 충분히 알기 전에는 요청 함수를 배우지 않는다)
	std::atomic<uint32_t> updates{0};

private:
	bool block(int b) const {
		// WRAM 과 HRAM 사이는 이어지지 않는다
		if (b < 0 || b >= static_cast<int>(m_blocks.size())) {
			return false;
		}
		return m_blocks[b].load(std::memory_order_relaxed) != 0;
	}
	std::unique_ptr<std::atomic<uint8_t>[]> m_flags;
	std::array<std::atomic<uint8_t>, (kGbRamSize + 15) / 16> m_blocks;
};

// 원본 RAM 사본 (WRAM 32KB + HRAM)
struct GbRamImage {
	std::array<uint8_t, kGbRamSize> ram{};
};

class GbShadowCpu {
public:
	struct Regs {
		uint8_t a = 0, f = 0, b = 0, c = 0, d = 0, e = 0, h = 0, l = 0;
		uint16_t sp = 0, pc = 0;
	};
	enum class Mode { Update, Request, Probe };
	enum class Exit { Returned, Budget, Halt, Stop, Illegal };

	explicit GbShadowCpu(std::shared_ptr<const std::vector<uint8_t>> rom);

	void setApu(GbApuSynth* apu) { m_apu = apu; }
	void setRamMap(GbDriverRamMap* map) { m_map = map; }
	// 사운드 레지스터 쓰기를 따로 받는다 (reg = FF00 기준 하위 바이트)
	void setSoundHook(std::function<void(uint8_t reg, uint8_t value)> hook) { m_soundHook = std::move(hook); }

	// 원본 상태를 가져온다
	void loadRam(const GbRamImage& image) { m_ram = image.ram; }
	void setRamByte(int index, uint8_t value) { m_ram[index] = value; }
	uint8_t ramByte(int index) const { return m_ram[index]; }
	void setRomBank(int bank) { m_romBank = bank; }
	void setWramBank(int bank) { m_wramBank = bank & 7 ? bank & 7 : 1; }
	void setIo(uint8_t reg, uint8_t value) { m_io[reg & 0x7F] = value; }
	// 주소 -> RAM 위치 번호 (RAM 이 아니면 -1). D000~DFFF 는 지금 WRAM bank 기준
	int ramIndex(uint16_t addr) const;
	static int ramIndex(uint16_t addr, int wramBank);

	// entry 를 불러 돌아올 때까지(또는 budget 사이클) 실행
	Exit call(uint16_t entry, int romBank, const Regs& regs, int budgetCycles, Mode mode);
	// 마지막 call 에서 함수가 실제로 쓴 입력 레지스터 (비트 0..5 = B,C,D,E,H,L, 비트 7 = A)
	uint8_t inputRegs() const { return m_inputs; }
	const Regs& regs() const { return m_r; }
	uint64_t totalCycles() const { return m_totalCycles; }

private:
	uint8_t fetch();
	uint16_t fetch16();
	uint8_t read(uint16_t addr);           // 명령어 데이터 읽기 (기록함)
	void write(uint16_t addr, uint8_t v);  // 명령어 데이터 쓰기 (기록함)
	uint8_t rawRead(uint16_t addr);
	void rawWrite(uint16_t addr, uint8_t v);
	uint8_t ioRead(uint8_t reg);
	void ioWrite(uint8_t reg, uint8_t v);
	void mbcWrite(uint16_t addr, uint8_t v);
	void track(uint16_t addr, bool isWrite);
	void push16(uint16_t v, uint8_t taint);
	uint16_t pop16(uint8_t& taint);
	bool step();  // 명령 하나. false = 멈춤 (m_exit)

	// 레지스터: 0 B 1 C 2 D 3 E 4 H 5 L 6 (HL) 7 A
	uint8_t& reg8(int i);
	uint8_t getR(int i);
	void setR(int i, uint8_t v, uint8_t taint);
	uint16_t hl() const { return static_cast<uint16_t>((m_r.h << 8) | m_r.l); }
	void use(uint8_t taint) { m_inputs |= taint; }
	void alu(int op, uint8_t v, uint8_t taint);
	void cbOp();
	bool cond(int cc) const;

	std::shared_ptr<const std::vector<uint8_t>> m_rom;
	int m_mbc = 0;  // 0 없음, 1 MBC1, 2 MBC2, 3 MBC3, 5 MBC5
	int m_romBank = 1, m_mbc1Hi = 0;
	int m_wramBank = 1;
	int m_sramBank = 0;
	bool m_sramOn = false;
	std::array<uint8_t, kGbRamSize> m_ram{};
	std::vector<uint8_t> m_sram;
	std::array<uint8_t, 0x2000> m_vram{};
	std::array<uint8_t, 0x80> m_io{};
	uint8_t m_ie = 0;
	uint8_t m_ly = 0;
	bool m_soundOn = true;

	GbApuSynth* m_apu = nullptr;
	GbDriverRamMap* m_map = nullptr;
	std::vector<std::pair<int, uint8_t>> m_pendingMarks;  // 요청 함수 실행 중의 지도 표시 (정상 반환 때 넣음)
	std::function<void(uint8_t, uint8_t)> m_soundHook;

	Regs m_r;
	uint8_t m_taint[8] = {};
	uint8_t m_inputs = 0;
	std::vector<uint8_t> m_stackTaint;  // 스택 RAM 의 바이트별 오염 (push/pop 으로 옮겨진 입력 레지스터)
	Mode m_mode = Mode::Update;
	Exit m_exit = Exit::Returned;
	uint16_t m_entrySp = 0;
	uint32_t m_gen = 0;
	std::vector<uint32_t> m_seen;  // RAM 위치별 마지막으로 만진 call 번호
	int64_t m_cycles = 0;
	uint64_t m_totalCycles = 0;
};

}
