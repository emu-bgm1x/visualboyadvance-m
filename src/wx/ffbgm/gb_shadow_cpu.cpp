/* ff-bgm1x: GB 사운드 드라이버 실행용 SM83 해석기. 설명은 GbShadowCpu.h */
#include "wx/ffbgm/gb_shadow_cpu.h"

#include "wx/ffbgm/gsc_player.h"

namespace QGBA {

namespace {

constexpr uint8_t FZ = 0x80, FN = 0x40, FH = 0x20, FC = 0x10;

// 레지스터별 오염 비트 (B C D E H L - A)
constexpr uint8_t kTaintBit[8] = {1 << 0, 1 << 1, 1 << 2, 1 << 3, 1 << 4, 1 << 5, 0, 1 << 7};

}

GbShadowCpu::GbShadowCpu(std::shared_ptr<const std::vector<uint8_t>> rom)
	: m_rom(std::move(rom))
	, m_sram(0x8000, 0)
	, m_stackTaint(0x10000, 0)
	, m_seen(kGbRamSize, 0)
{
	const uint8_t type = m_rom->size() > 0x147 ? (*m_rom)[0x147] : 0;
	if (type >= 0x01 && type <= 0x03) {
		m_mbc = 1;
	} else if (type == 0x05 || type == 0x06) {
		m_mbc = 2;
	} else if (type >= 0x0F && type <= 0x13) {
		m_mbc = 3;
	} else if (type >= 0x19 && type <= 0x1E) {
		m_mbc = 5;
	} else if (type == 0x00 || type == 0x08 || type == 0x09) {
		m_mbc = 0;
	} else {
		m_mbc = 5;  // 그 밖의 칩은 2000~2FFF 쓰기를 bank 번호로 본다
	}
	m_io[0x26] = 0x80;
}

int GbShadowCpu::ramIndex(uint16_t addr, int wramBank) {
	if (addr >= 0xC000 && addr < 0xFE00) {
		const uint16_t a = addr >= 0xE000 ? addr - 0x2000 : addr;
		if (a < 0xD000) {
			return a - 0xC000;
		}
		return (wramBank & 7 ? wramBank & 7 : 1) * 0x1000 + (a - 0xD000);
	}
	if (addr >= 0xFF80) {
		return 0x8000 + (addr - 0xFF80);
	}
	return -1;
}

int GbShadowCpu::ramIndex(uint16_t addr) const {
	return ramIndex(addr, m_wramBank);
}

void GbShadowCpu::track(uint16_t addr, bool isWrite) {
	if (!m_map || m_mode == Mode::Probe) {
		return;
	}
	// 이 호출의 스택 (지역 변수 등)은 드라이버 RAM 으로 보지 않는다
	if (addr >= m_r.sp && addr < static_cast<uint32_t>(m_entrySp) + 0x20) {
		return;
	}
	const int i = ramIndex(addr);
	if (i < 0) {
		return;
	}
	const bool update = m_mode == Mode::Update;
	uint8_t f = update ? GbDriverRamMap::TouchedUpdate : 0;
	if (isWrite) {
		f |= update ? GbDriverRamMap::WrittenUpdate : GbDriverRamMap::WrittenRequest;
	} else if (m_seen[i] != m_gen) {
		f |= update ? GbDriverRamMap::ReadFirstUpdate : GbDriverRamMap::ReadFirstRequest;
	}
	m_seen[i] = m_gen;
	if (f && m_mode == Mode::Request) {
		m_pendingMarks.emplace_back(i, f);  // 요청 함수가 정상으로 돌아왔을 때만 드라이버 RAM 지도에 넣는다
	} else if (f) {
		m_map->mark(i, f);
	}
}

uint8_t GbShadowCpu::rawRead(uint16_t addr) {
	if (addr < 0x4000) {
		return addr < m_rom->size() ? (*m_rom)[addr] : 0xFF;
	}
	if (addr < 0x8000) {
		const size_t off = static_cast<size_t>(m_romBank) * 0x4000 + (addr - 0x4000);
		return m_rom->empty() ? 0xFF : (*m_rom)[off % m_rom->size()];
	}
	if (addr < 0xA000) {
		return m_vram[addr - 0x8000];
	}
	if (addr < 0xC000) {
		return m_sramOn ? m_sram[(static_cast<size_t>(m_sramBank & 3) * 0x2000 + (addr - 0xA000))] : 0xFF;
	}
	if (addr < 0xFE00) {
		return m_ram[ramIndex(addr)];
	}
	if (addr < 0xFF00) {
		return 0xFF;
	}
	if (addr < 0xFF80) {
		return ioRead(static_cast<uint8_t>(addr & 0x7F));
	}
	if (addr == 0xFFFF) {
		return m_ie;
	}
	return m_ram[0x8000 + (addr - 0xFF80)];
}

void GbShadowCpu::rawWrite(uint16_t addr, uint8_t v) {
	if (addr < 0x8000) {
		mbcWrite(addr, v);
	} else if (addr < 0xA000) {
		m_vram[addr - 0x8000] = v;
	} else if (addr < 0xC000) {
		if (m_sramOn) {
			m_sram[static_cast<size_t>(m_sramBank & 3) * 0x2000 + (addr - 0xA000)] = v;
		}
	} else if (addr < 0xFE00) {
		m_ram[ramIndex(addr)] = v;
	} else if (addr < 0xFF00) {
		// OAM: 버림
	} else if (addr < 0xFF80) {
		ioWrite(static_cast<uint8_t>(addr & 0x7F), v);
	} else if (addr == 0xFFFF) {
		m_ie = v;
	} else {
		m_ram[0x8000 + (addr - 0xFF80)] = v;
	}
}

uint8_t GbShadowCpu::read(uint16_t addr) {
	track(addr, false);
	return rawRead(addr);
}

void GbShadowCpu::write(uint16_t addr, uint8_t v) {
	track(addr, true);
	rawWrite(addr, v);
}

void GbShadowCpu::mbcWrite(uint16_t addr, uint8_t v) {
	switch (m_mbc) {
	case 0:
		break;
	case 1:
		if (addr < 0x2000) {
			m_sramOn = (v & 0xF) == 0xA;
		} else if (addr < 0x4000) {
			const int lo = (v & 0x1F) ? (v & 0x1F) : 1;
			m_romBank = (m_mbc1Hi << 5) | lo;
		} else if (addr < 0x6000) {
			m_mbc1Hi = v & 3;
			m_romBank = (m_mbc1Hi << 5) | (m_romBank & 0x1F);
		}
		break;
	case 2:
		if (addr < 0x4000) {
			if (addr & 0x100) {
				m_romBank = (v & 0xF) ? (v & 0xF) : 1;
			} else {
				m_sramOn = (v & 0xF) == 0xA;
			}
		}
		break;
	case 3:
		if (addr < 0x2000) {
			m_sramOn = (v & 0xF) == 0xA;
		} else if (addr < 0x4000) {
			m_romBank = (v & 0x7F) ? (v & 0x7F) : 1;
		} else if (addr < 0x6000) {
			m_sramBank = v;
		}
		break;
	default:
		if (addr < 0x2000) {
			m_sramOn = (v & 0xF) == 0xA;
		} else if (addr < 0x3000) {
			m_romBank = (m_romBank & 0x100) | v;
		} else if (addr < 0x4000) {
			m_romBank = (m_romBank & 0xFF) | ((v & 1) << 8);
		} else if (addr < 0x6000) {
			m_sramBank = v & 0xF;
		}
		break;
	}
}

uint8_t GbShadowCpu::ioRead(uint8_t reg) {
	if (reg >= 0x10 && reg < 0x40) {
		return m_apu ? m_apu->readHw(reg) : m_io[reg];
	}
	switch (reg) {
	case 0x00:
		return 0xCF;  // 버튼 없음
	case 0x04:
		return static_cast<uint8_t>((m_totalCycles + m_cycles) >> 8);  // DIV
	case 0x44:
		// LY 를 기다리는 반복이 끝나도록 읽을 때마다 나아간다
		m_ly = static_cast<uint8_t>((m_ly + 1) % 154);
		return m_ly;
	case 0x70:
		return static_cast<uint8_t>(0xF8 | m_wramBank);
	default:
		return m_io[reg];
	}
}

void GbShadowCpu::ioWrite(uint8_t reg, uint8_t v) {
	if (reg >= 0x10 && reg < 0x40) {
		if (reg == 0x26) {
			const bool on = v & 0x80;
			if (m_soundOn && !on && m_apu) {
				// 전원을 끄면 레지스터가 모두 0 이 된다
				for (uint8_t r = 0x10; r < 0x26; ++r) {
					m_apu->write(r, 0);
				}
			}
			m_soundOn = on;
		} else if (reg < 0x26 && !m_soundOn) {
			return;  // 전원이 꺼져 있는 동안은 무시
		}
		m_io[reg] = v;
		if (m_apu) {
			m_apu->write(reg, v);
		}
		if (m_soundHook) {
			m_soundHook(reg, v);
		}
		return;
	}
	if (reg == 0x70) {
		setWramBank(v);
	}
	m_io[reg] = v;
}

uint8_t GbShadowCpu::fetch() {
	const uint8_t v = rawRead(m_r.pc);
	++m_r.pc;
	return v;
}

uint16_t GbShadowCpu::fetch16() {
	const uint8_t lo = fetch();
	return static_cast<uint16_t>(lo | (fetch() << 8));
}

void GbShadowCpu::push16(uint16_t v, uint8_t taint) {
	--m_r.sp;
	rawWrite(m_r.sp, static_cast<uint8_t>(v >> 8));
	m_stackTaint[m_r.sp] = taint;
	--m_r.sp;
	rawWrite(m_r.sp, static_cast<uint8_t>(v));
	m_stackTaint[m_r.sp] = taint;
}

uint16_t GbShadowCpu::pop16(uint8_t& taint) {
	const uint8_t lo = rawRead(m_r.sp);
	taint = m_stackTaint[m_r.sp];
	++m_r.sp;
	const uint8_t hi = rawRead(m_r.sp);
	taint |= m_stackTaint[m_r.sp];
	++m_r.sp;
	return static_cast<uint16_t>(lo | (hi << 8));
}

uint8_t& GbShadowCpu::reg8(int i) {
	switch (i) {
	case 0: return m_r.b;
	case 1: return m_r.c;
	case 2: return m_r.d;
	case 3: return m_r.e;
	case 4: return m_r.h;
	case 5: return m_r.l;
	default: return m_r.a;
	}
}

uint8_t GbShadowCpu::getR(int i) {
	if (i == 6) {
		use(m_taint[4] | m_taint[5]);
		return read(hl());
	}
	return reg8(i);
}

void GbShadowCpu::setR(int i, uint8_t v, uint8_t taint) {
	if (i == 6) {
		use(m_taint[4] | m_taint[5] | taint);
		write(hl(), v);
		return;
	}
	reg8(i) = v;
	m_taint[i] = taint;
}

bool GbShadowCpu::cond(int cc) const {
	switch (cc & 3) {
	case 0: return !(m_r.f & FZ);
	case 1: return m_r.f & FZ;
	case 2: return !(m_r.f & FC);
	default: return m_r.f & FC;
	}
}

void GbShadowCpu::alu(int op, uint8_t v, uint8_t taint) {
	use(m_taint[7] | taint);
	const uint8_t a = m_r.a;
	const int carry = (m_r.f & FC) ? 1 : 0;
	int r;
	switch (op) {
	case 0:  // ADD
	case 1:  // ADC
	{
		const int c = op == 1 ? carry : 0;
		r = a + v + c;
		m_r.f = static_cast<uint8_t>(((r & 0xFF) ? 0 : FZ) | (((a & 0xF) + (v & 0xF) + c) > 0xF ? FH : 0) | (r > 0xFF ? FC : 0));
		m_r.a = static_cast<uint8_t>(r);
		break;
	}
	case 2:  // SUB
	case 3:  // SBC
	case 7:  // CP
	{
		const int c = op == 3 ? carry : 0;
		r = a - v - c;
		m_r.f = static_cast<uint8_t>(((r & 0xFF) ? 0 : FZ) | FN | (((a & 0xF) - (v & 0xF) - c) < 0 ? FH : 0) | (r < 0 ? FC : 0));
		if (op != 7) {
			m_r.a = static_cast<uint8_t>(r);
		}
		break;
	}
	case 4:  // AND
		m_r.a = a & v;
		m_r.f = static_cast<uint8_t>((m_r.a ? 0 : FZ) | FH);
		break;
	case 5:  // XOR
		m_r.a = a ^ v;
		m_r.f = m_r.a ? 0 : FZ;
		break;
	default:  // OR
		m_r.a = a | v;
		m_r.f = m_r.a ? 0 : FZ;
		break;
	}
	if (op != 7) {
		// xor a / sub a 는 입력과 무관한 0 이다
		m_taint[7] = 0;
	}
}

void GbShadowCpu::cbOp() {
	const uint8_t op = fetch();
	const int ri = op & 7;
	const int y = (op >> 3) & 7;
	m_cycles += ri == 6 ? 16 : 8;
	uint8_t v = getR(ri);
	use(ri == 6 ? 0 : m_taint[ri]);
	switch (op >> 6) {
	case 0: {
		uint8_t r = 0;
		const bool c = m_r.f & FC;
		bool out = false;
		switch (y) {
		case 0: out = v & 0x80; r = static_cast<uint8_t>((v << 1) | (v >> 7)); break;   // RLC
		case 1: out = v & 1; r = static_cast<uint8_t>((v >> 1) | (v << 7)); break;      // RRC
		case 2: out = v & 0x80; r = static_cast<uint8_t>((v << 1) | (c ? 1 : 0)); break; // RL
		case 3: out = v & 1; r = static_cast<uint8_t>((v >> 1) | (c ? 0x80 : 0)); break; // RR
		case 4: out = v & 0x80; r = static_cast<uint8_t>(v << 1); break;                 // SLA
		case 5: out = v & 1; r = static_cast<uint8_t>((v >> 1) | (v & 0x80)); break;    // SRA
		case 6: out = false; r = static_cast<uint8_t>((v >> 4) | (v << 4)); break;       // SWAP
		default: out = v & 1; r = static_cast<uint8_t>(v >> 1); break;                   // SRL
		}
		m_r.f = static_cast<uint8_t>((r ? 0 : FZ) | (out ? FC : 0));
		setR(ri, r, 0);
		break;
	}
	case 1:  // BIT
		m_r.f = static_cast<uint8_t>((m_r.f & FC) | FH | ((v >> y) & 1 ? 0 : FZ));
		if (ri == 6) {
			m_cycles -= 4;
		}
		break;
	case 2:  // RES
		setR(ri, static_cast<uint8_t>(v & ~(1 << y)), 0);
		break;
	default:  // SET
		setR(ri, static_cast<uint8_t>(v | (1 << y)), 0);
		break;
	}
}

bool GbShadowCpu::step() {
	const uint8_t op = fetch();
	auto pairTaint = [&](int p) -> uint8_t {
		switch (p) {
		case 0: return m_taint[0] | m_taint[1];
		case 1: return m_taint[2] | m_taint[3];
		case 2: return m_taint[4] | m_taint[5];
		default: return 0;  // SP
		}
	};
	auto getPair = [&](int p) -> uint16_t {
		switch (p) {
		case 0: return static_cast<uint16_t>((m_r.b << 8) | m_r.c);
		case 1: return static_cast<uint16_t>((m_r.d << 8) | m_r.e);
		case 2: return hl();
		default: return m_r.sp;
		}
	};
	auto setPair = [&](int p, uint16_t v, uint8_t taint) {
		switch (p) {
		case 0: m_r.b = v >> 8; m_r.c = v & 0xFF; m_taint[0] = m_taint[1] = taint; break;
		case 1: m_r.d = v >> 8; m_r.e = v & 0xFF; m_taint[2] = m_taint[3] = taint; break;
		case 2: m_r.h = v >> 8; m_r.l = v & 0xFF; m_taint[4] = m_taint[5] = taint; break;
		default: m_r.sp = v; break;
		}
	};
	auto setHl = [&](uint16_t v) {
		m_r.h = v >> 8;
		m_r.l = v & 0xFF;
	};

	// LD r,r' / HALT
	if (op >= 0x40 && op < 0x80) {
		if (op == 0x76) {
			m_cycles += 4;
			m_exit = Exit::Halt;
			return false;
		}
		const int dst = (op >> 3) & 7, src = op & 7;
		m_cycles += (dst == 6 || src == 6) ? 8 : 4;
		const uint8_t v = getR(src);
		setR(dst, v, src == 6 ? 0 : m_taint[src]);
		return true;
	}
	// ALU A,r
	if (op >= 0x80 && op < 0xC0) {
		const int src = op & 7;
		m_cycles += src == 6 ? 8 : 4;
		if (src == 7 && (op == 0xAF || op == 0x97)) {
			// xor a / sub a: 입력과 무관하게 0
			m_r.a = 0;
			m_r.f = op == 0xAF ? FZ : (FZ | FN);
			m_taint[7] = 0;
			return true;
		}
		const uint8_t v = getR(src);
		alu((op >> 3) & 7, v, src == 6 ? 0 : m_taint[src]);
		return true;
	}

	switch (op) {
	case 0x00:
		m_cycles += 4;
		break;
	case 0x01: case 0x11: case 0x21: case 0x31:
		m_cycles += 12;
		setPair(op >> 4, fetch16(), 0);
		break;
	case 0x02: case 0x12: {
		m_cycles += 8;
		const int p = op >> 4;
		use(pairTaint(p) | m_taint[7]);
		write(getPair(p), m_r.a);
		break;
	}
	case 0x22: case 0x32: {
		m_cycles += 8;
		use(pairTaint(2) | m_taint[7]);
		const uint16_t a = hl();
		write(a, m_r.a);
		setHl(static_cast<uint16_t>(op == 0x22 ? a + 1 : a - 1));
		break;
	}
	case 0x0A: case 0x1A: {
		m_cycles += 8;
		const int p = op >> 4;
		use(pairTaint(p));
		m_r.a = read(getPair(p));
		m_taint[7] = 0;
		break;
	}
	case 0x2A: case 0x3A: {
		m_cycles += 8;
		use(pairTaint(2));
		const uint16_t a = hl();
		m_r.a = read(a);
		m_taint[7] = 0;
		setHl(static_cast<uint16_t>(op == 0x2A ? a + 1 : a - 1));
		break;
	}
	case 0x03: case 0x13: case 0x23: case 0x33:
	case 0x0B: case 0x1B: case 0x2B: case 0x3B: {
		m_cycles += 8;
		const int p = (op >> 4) & 3;
		const uint8_t t = pairTaint(p);
		use(t);
		setPair(p, static_cast<uint16_t>(getPair(p) + ((op & 8) ? -1 : 1)), t);
		break;
	}
	case 0x04: case 0x0C: case 0x14: case 0x1C: case 0x24: case 0x2C: case 0x34: case 0x3C:
	case 0x05: case 0x0D: case 0x15: case 0x1D: case 0x25: case 0x2D: case 0x35: case 0x3D: {
		const int ri = (op >> 3) & 7;
		m_cycles += ri == 6 ? 12 : 4;
		const uint8_t t = ri == 6 ? 0 : m_taint[ri];
		use(t);
		const uint8_t v = getR(ri);
		const bool dec = op & 1;
		const uint8_t r = static_cast<uint8_t>(dec ? v - 1 : v + 1);
		uint8_t f = static_cast<uint8_t>((m_r.f & FC) | (r ? 0 : FZ));
		if (dec) {
			f |= FN | ((v & 0xF) == 0 ? FH : 0);
		} else {
			f |= (v & 0xF) == 0xF ? FH : 0;
		}
		m_r.f = f;
		setR(ri, r, t);
		break;
	}
	case 0x06: case 0x0E: case 0x16: case 0x1E: case 0x26: case 0x2E: case 0x36: case 0x3E: {
		const int ri = (op >> 3) & 7;
		m_cycles += ri == 6 ? 12 : 8;
		setR(ri, fetch(), 0);
		break;
	}
	case 0x07: case 0x0F: case 0x17: case 0x1F: {
		m_cycles += 4;
		use(m_taint[7]);
		const uint8_t a = m_r.a;
		const bool c = m_r.f & FC;
		bool out;
		switch (op) {
		case 0x07: out = a & 0x80; m_r.a = static_cast<uint8_t>((a << 1) | (a >> 7)); break;
		case 0x0F: out = a & 1; m_r.a = static_cast<uint8_t>((a >> 1) | (a << 7)); break;
		case 0x17: out = a & 0x80; m_r.a = static_cast<uint8_t>((a << 1) | (c ? 1 : 0)); break;
		default: out = a & 1; m_r.a = static_cast<uint8_t>((a >> 1) | (c ? 0x80 : 0)); break;
		}
		m_r.f = out ? FC : 0;
		break;
	}
	case 0x08: {
		m_cycles += 20;
		const uint16_t a = fetch16();
		write(a, static_cast<uint8_t>(m_r.sp));
		write(static_cast<uint16_t>(a + 1), static_cast<uint8_t>(m_r.sp >> 8));
		break;
	}
	case 0x09: case 0x19: case 0x29: case 0x39: {
		m_cycles += 8;
		const int p = op >> 4;
		use(pairTaint(p) | pairTaint(2));
		const uint32_t a = hl(), b = getPair(p);
		const uint32_t r = a + b;
		m_r.f = static_cast<uint8_t>((m_r.f & FZ) | (((a & 0xFFF) + (b & 0xFFF)) > 0xFFF ? FH : 0) | (r > 0xFFFF ? FC : 0));
		setHl(static_cast<uint16_t>(r));
		break;
	}
	case 0x10:
		m_cycles += 4;
		fetch();
		m_exit = Exit::Stop;
		return false;
	case 0x18: {
		m_cycles += 12;
		const int8_t e = static_cast<int8_t>(fetch());
		m_r.pc = static_cast<uint16_t>(m_r.pc + e);
		break;
	}
	case 0x20: case 0x28: case 0x30: case 0x38: {
		const int8_t e = static_cast<int8_t>(fetch());
		if (cond((op >> 3) & 3)) {
			m_cycles += 12;
			m_r.pc = static_cast<uint16_t>(m_r.pc + e);
		} else {
			m_cycles += 8;
		}
		break;
	}
	case 0x27: {
		m_cycles += 4;
		use(m_taint[7]);
		int a = m_r.a;
		if (!(m_r.f & FN)) {
			if ((m_r.f & FH) || (a & 0xF) > 9) a += 0x06;
			if ((m_r.f & FC) || a > 0x9F) a += 0x60;
		} else {
			if (m_r.f & FH) a = (a - 6) & 0xFF;
			if (m_r.f & FC) a -= 0x60;
		}
		uint8_t f = m_r.f & (FN | FC);
		if (a & 0x100) f |= FC;
		m_r.a = static_cast<uint8_t>(a);
		if (!m_r.a) f |= FZ;
		m_r.f = f;
		break;
	}
	case 0x2F:
		m_cycles += 4;
		use(m_taint[7]);
		m_r.a = static_cast<uint8_t>(~m_r.a);
		m_r.f |= FN | FH;
		break;
	case 0x37:
		m_cycles += 4;
		m_r.f = static_cast<uint8_t>((m_r.f & FZ) | FC);
		break;
	case 0x3F:
		m_cycles += 4;
		m_r.f = static_cast<uint8_t>((m_r.f & FZ) | ((m_r.f & FC) ? 0 : FC));
		break;
	case 0xC0: case 0xC8: case 0xD0: case 0xD8:
		if (cond((op >> 3) & 3)) {
			m_cycles += 20;
			uint8_t t;
			m_r.pc = pop16(t);
		} else {
			m_cycles += 8;
		}
		break;
	case 0xC9: case 0xD9: {
		m_cycles += 16;
		uint8_t t;
		m_r.pc = pop16(t);
		break;
	}
	case 0xC1: case 0xD1: case 0xE1: {
		m_cycles += 12;
		uint8_t t;
		const uint16_t v = pop16(t);
		setPair((op >> 4) & 3, v, t);
		break;
	}
	case 0xF1: {
		m_cycles += 12;
		uint8_t t;
		const uint16_t v = pop16(t);
		m_r.f = static_cast<uint8_t>(v & 0xF0);
		m_r.a = static_cast<uint8_t>(v >> 8);
		m_taint[7] = t;
		break;
	}
	case 0xC5: case 0xD5: case 0xE5: {
		m_cycles += 16;
		const int p = (op >> 4) & 3;
		push16(getPair(p), pairTaint(p));
		break;
	}
	case 0xF5:
		m_cycles += 16;
		push16(static_cast<uint16_t>((m_r.a << 8) | m_r.f), m_taint[7]);
		break;
	case 0xC2: case 0xCA: case 0xD2: case 0xDA: {
		const uint16_t a = fetch16();
		if (cond((op >> 3) & 3)) {
			m_cycles += 16;
			m_r.pc = a;
		} else {
			m_cycles += 12;
		}
		break;
	}
	case 0xC3:
		m_cycles += 16;
		m_r.pc = fetch16();
		break;
	case 0xE9:
		m_cycles += 4;
		use(pairTaint(2));
		m_r.pc = hl();
		break;
	case 0xC4: case 0xCC: case 0xD4: case 0xDC: {
		const uint16_t a = fetch16();
		if (cond((op >> 3) & 3)) {
			m_cycles += 24;
			push16(m_r.pc, 0);
			m_r.pc = a;
		} else {
			m_cycles += 12;
		}
		break;
	}
	case 0xCD: {
		m_cycles += 24;
		const uint16_t a = fetch16();
		push16(m_r.pc, 0);
		m_r.pc = a;
		break;
	}
	case 0xC6: case 0xCE: case 0xD6: case 0xDE: case 0xE6: case 0xEE: case 0xF6: case 0xFE:
		m_cycles += 8;
		alu((op >> 3) & 7, fetch(), 0);
		break;
	case 0xC7: case 0xCF: case 0xD7: case 0xDF: case 0xE7: case 0xEF: case 0xF7: case 0xFF:
		m_cycles += 16;
		push16(m_r.pc, 0);
		m_r.pc = op & 0x38;
		break;
	case 0xCB:
		cbOp();
		break;
	case 0xE0:
		m_cycles += 12;
		use(m_taint[7]);
		write(static_cast<uint16_t>(0xFF00 | fetch()), m_r.a);
		break;
	case 0xF0:
		m_cycles += 12;
		m_r.a = read(static_cast<uint16_t>(0xFF00 | fetch()));
		m_taint[7] = 0;
		break;
	case 0xE2:
		m_cycles += 8;
		use(m_taint[7] | m_taint[1]);
		write(static_cast<uint16_t>(0xFF00 | m_r.c), m_r.a);
		break;
	case 0xF2:
		m_cycles += 8;
		use(m_taint[1]);
		m_r.a = read(static_cast<uint16_t>(0xFF00 | m_r.c));
		m_taint[7] = 0;
		break;
	case 0xE8: case 0xF8: {
		const int8_t e = static_cast<int8_t>(fetch());
		const uint16_t sp = m_r.sp;
		const uint16_t r = static_cast<uint16_t>(sp + e);
		m_r.f = static_cast<uint8_t>((((sp & 0xF) + (e & 0xF)) > 0xF ? FH : 0) | (((sp & 0xFF) + (e & 0xFF)) > 0xFF ? FC : 0));
		if (op == 0xE8) {
			m_cycles += 16;
			m_r.sp = r;
		} else {
			m_cycles += 12;
			setHl(r);
			m_taint[4] = m_taint[5] = 0;
		}
		break;
	}
	case 0xF9:
		m_cycles += 8;
		use(pairTaint(2));
		m_r.sp = hl();
		break;
	case 0xEA:
		m_cycles += 16;
		use(m_taint[7]);
		write(fetch16(), m_r.a);
		break;
	case 0xFA:
		m_cycles += 16;
		m_r.a = read(fetch16());
		m_taint[7] = 0;
		break;
	case 0xF3: case 0xFB:
		m_cycles += 4;
		break;
	default:
		m_exit = Exit::Illegal;
		return false;
	}
	return true;
}

GbShadowCpu::Exit GbShadowCpu::call(uint16_t entry, int romBank, const Regs& regs, int budgetCycles, Mode mode) {
	m_r = regs;
	m_r.pc = entry;
	if (romBank > 0) {
		m_romBank = romBank;
	}
	m_mode = mode;
	m_entrySp = regs.sp;
	for (int i = 0; i < 8; ++i) {
		m_taint[i] = kTaintBit[i];
	}
	m_inputs = 0;
	m_pendingMarks.clear();
	++m_gen;
	if (!m_gen) {
		std::fill(m_seen.begin(), m_seen.end(), 0);
		m_gen = 1;
	}
	m_cycles = 0;
	m_exit = Exit::Returned;
	while (m_cycles < budgetCycles) {
		if (!step()) {
			break;
		}
		// 돌아옴: 불렀을 때의 돌아갈 주소를 꺼냈다
		if (m_r.sp > m_entrySp && m_r.sp - m_entrySp < 0x100) {
			m_exit = Exit::Returned;
			m_totalCycles += m_cycles;
			m_inputs &= 0xBF;
			if (m_map) {
				for (const auto& pm : m_pendingMarks) {
					m_map->mark(pm.first, pm.second);
				}
			}
			m_pendingMarks.clear();
			return m_exit;
		}
	}
	if (m_cycles >= budgetCycles) {
		m_exit = Exit::Budget;
	}
	m_pendingMarks.clear();
	m_totalCycles += m_cycles;
	m_inputs &= 0xBF;
	return m_exit;
}

}
