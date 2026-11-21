/* ff-bgm1x: 엔진에 상관없는 GB/GBC BGM·효과음 1배속. 설명은 GbDriverGeneric.h */
#include "wx/ffbgm/gb_driver_generic.h"

#include <algorithm>
#include <cmath>
#include <cstdio>

namespace QGBA {

namespace {

constexpr double kClock = 4194304.0;     // GB 사운드 클럭 (Hz)
constexpr double kFrameCycles = 70224.0;  // 한 화면 프레임
constexpr int kRecordRate = 131072;       // mGBA GB 오디오의 녹화 스트림 표본율
constexpr int kUpdateBudget = 400000;     // 갱신 루틴 한 번에 허용하는 사이클 (넘으면 멈춤)
constexpr int kRequestBudget = 2000000;   // 요청 함수 한 번
constexpr int kVotes = 3;                 // 같은 갱신 루틴을 이만큼 연속으로 보면 확정
constexpr int kMainVotes = 20;            // 주 흐름의 갱신 루틴: 이만큼의 프레임에서 같은 경로를 보면 확정
constexpr int kMainSndVotes = 3;         // 주 흐름의 사운드 함수(효과음 엔진 등): 이만큼의 프레임에서 같은 함수면 그 호출을 그림자에 넘긴다
constexpr int kSwitchVotes = 120;         // 확정한 뒤 다른 후보가 이만큼 연속이면 바꾼다 (사운드를 쓴 인터럽트 수)
constexpr int kSwitchVotesIdle = 8;       // 확정한 갱신 루틴이 그 뒤 사운드를 한 번도 쓰지 않았으면 이만큼으로 바꾼다
                                          // (게임 시작 때 한 번 불린 곡 요청 함수를 잘못 잡은 경우. 곡을 드물게 쓰는 드라이버는 120 번에 오래 걸린다)
constexpr size_t kMaxFrames = 1024;

bool isCallOpcode(uint8_t op) {
	switch (op) {
	case 0xCD: case 0xC4: case 0xCC: case 0xD4: case 0xDC:
	case 0xC7: case 0xCF: case 0xD7: case 0xDF: case 0xE7: case 0xEF: case 0xF7: case 0xFF:
		return true;
	default:
		return false;
	}
}

}

// ================================================================ GbDriverLink

std::shared_ptr<GbDriverLink::Queue> GbDriverLink::subscribe() {
	auto q = std::make_shared<Queue>();
	std::lock_guard<std::mutex> lock(m_mutex);
	m_queues.push_back(q);
	// 새 그림자는 앞서 보낸 원본 상태를 받지 못했다: 다음 프레임 끝에 다시 보내게 한다
	initRequested = true;
	return q;
}

void GbDriverLink::unsubscribe(const std::shared_ptr<Queue>& q) {
	std::lock_guard<std::mutex> lock(m_mutex);
	m_queues.erase(std::remove(m_queues.begin(), m_queues.end(), q), m_queues.end());
}

void GbDriverLink::post(const std::shared_ptr<const GbDriverEvent>& e) {
	std::lock_guard<std::mutex> lock(m_mutex);
	for (auto& q : m_queues) {
		std::lock_guard<std::mutex> ql(q->mutex);
		q->events.push_back(e);
		// 그림자가 오래 돌지 않으면 (오디오 정지 등) 쌓이지 않게 버린다. 버린 뒤에는 Init 를 기다린다
		while (q->events.size() > 20000) {
			q->events.pop_front();
		}
	}
}

void GbDriverLink::reportBadRequest(uint32_t key) {
	std::lock_guard<std::mutex> lock(m_mutex);
	m_badRequests.insert(key);
}

bool GbDriverLink::badRequest(uint32_t key) const {
	std::lock_guard<std::mutex> lock(m_mutex);
	return m_badRequests.count(key) != 0;
}

void GbDriverLink::setUpdate(const Routine& r) {
	std::lock_guard<std::mutex> lock(m_mutex);
	m_update = r;
}

void GbDriverLink::setRoutines(const std::vector<Routine>& r) {
	std::lock_guard<std::mutex> lock(m_mutex);
	m_routines = r;
}

std::vector<GbDriverLink::Routine> GbDriverLink::routines() const {
	std::lock_guard<std::mutex> lock(m_mutex);
	return m_routines;
}

GbDriverLink::Routine GbDriverLink::update() const {
	std::lock_guard<std::mutex> lock(m_mutex);
	return m_update;
}

void GbDriverLink::setLatest(std::shared_ptr<const GbRamImage> image) {
	std::lock_guard<std::mutex> lock(m_mutex);
	m_latest = std::move(image);
}

std::shared_ptr<const GbRamImage> GbDriverLink::latest() const {
	std::lock_guard<std::mutex> lock(m_mutex);
	return m_latest;
}

// ================================================================ GbDriverTracer

GbDriverTracer::GbDriverTracer(std::shared_ptr<GbDriverLink> link)
	: m_link(std::move(link))
{
	m_frames.reserve(kMaxFrames);
	m_levels.emplace_back();
}

void GbDriverTracer::dropStack() {
	m_frames.clear();
	m_levels.clear();
	m_levels.emplace_back();
	m_pendingCall = false;
	m_pendingJump = false;
	m_updateDepth = -1;
}

void GbDriverTracer::resync() {
	m_foreignStack = false;
	m_spLoaded = false;
	m_frames.clear();
	m_levels.clear();
	m_levels.emplace_back();
	m_pendingCall = false;
	m_updateDepth = -1;
	m_needInit = m_update.valid;
}

// 함수 첫 명령이 중계(jp hl, jp 주소, jr)면 도착지를 그 함수로 본다. 공용 중계 함수(인터럽트 벡터,
// "HL 이 가리키는 곳으로 뛰기" 등)를 거치는 서로 다른 함수를 같은 함수로 보지 않게.
// 중계는 레지스터를 바꾸지 않으므로 그림자에서 도착지를 같은 레지스터로 불러도 같다
uint16_t GbDriverTracer::resolveEntry(const Frame& f) const {
	uint16_t entry = f.entry;
	if (!m_rom || m_rom->empty()) {
		return entry;
	}
	const std::vector<uint8_t>& rom = *m_rom;
	auto romAt = [&](uint16_t addr) -> int {
		size_t off;
		if (addr < 0x4000) {
			off = addr;
		} else if (addr < 0x8000) {
			off = static_cast<size_t>(f.romBank) * 0x4000 + (addr - 0x4000);
		} else {
			return -1;  // RAM 의 코드는 따라가지 않는다
		}
		return off < rom.size() ? rom[off] : -1;
	};
	for (int hop = 0; hop < 4; ++hop) {
		const int op = romAt(entry);
		if (op == 0xE9) {  // jp hl
			entry = static_cast<uint16_t>((f.regs.h << 8) | f.regs.l);
		} else if (op == 0xC3) {  // jp nn
			const int lo = romAt(static_cast<uint16_t>(entry + 1)), hi = romAt(static_cast<uint16_t>(entry + 2));
			if (lo < 0 || hi < 0) break;
			entry = static_cast<uint16_t>(lo | (hi << 8));
		} else if (op == 0x18) {  // jr e
			const int e = romAt(static_cast<uint16_t>(entry + 1));
			if (e < 0) break;
			entry = static_cast<uint16_t>(entry + 2 + static_cast<int8_t>(e));
		} else {
			break;
		}
		if (entry >= 0x8000) {
			return f.entry;
		}
	}
	return entry;
}

void GbDriverTracer::push(const Frame& f) {
	if (m_frames.size() >= kMaxFrames) {
		// 스택을 따라가지 못함 (스택을 직접 옮기는 코드 등): 처음부터
		m_frames.clear();
		m_levels.clear();
		m_levels.emplace_back();
		m_updateDepth = -1;
	}
	const int idx = static_cast<int>(m_frames.size());
	if (f.isr) {
		Level lv;
		lv.base = idx;
		lv.vector = f.entry;
		m_levels.push_back(lv);
	}
	Frame g = f;
	g.entry = resolveEntry(g);
	g.impureMark = m_levels.back().impure;
	g.hardMark = m_levels.back().hard;
	g.trapMark = m_levels.back().traps;
	m_frames.push_back(g);
	if (f.isr || m_updateDepth >= 0) {
		return;
	}
	// 갱신 루틴은 중계를 따라간 주소로 기억한다 (점프 하나를 거쳐 들어오는 게임)
	const uint32_t k = key(g.entry, g.romBank);
	if (m_update.valid && (k == key(m_update.entry, m_update.romBank) || key(f.entry, f.romBank) == key(m_update.entry, m_update.romBank))) {
		m_updateDepth = idx;
		m_updVector = m_levels.size() > 1 ? m_levels.back().vector : 0;
		++m_updateCalls;
		++m_frameCalls;
		postMark();
		m_update.regs = f.regs;
		m_update.wramBank = f.wramBank;
		// 뱅크 0 영역의 갱신 루틴은 부른 쪽이 맞춰 둔 롬 뱅크(곡 데이터가 있는 뱅크 등)로 돈다
		if (f.entry < 0x4000) {
			m_update.romBank = f.romBank;
		}
		m_link->setUpdate(m_update);
		return;
	}
	if (m_update.valid && m_routines.size() > 1 && inRoutines(k)) {
		// 둘째 이후의 갱신 루틴: 안의 쓰기는 드라이버의 진행이다. 부를 때의 레지스터·뱅크를 그림자에 알린다
		m_updateDepth = idx;
		for (Frame& r : m_routines) {
			if (key(r.entry, r.romBank) == k) {
				r.regs = f.regs;
				r.wramBank = f.wramBank;
				if (f.entry < 0x4000) {
					r.romBank = f.romBank;
				}
			}
		}
		publishRoutines();
		return;
	}
	Level& lv = m_levels.back();
	if (lv.trapDepth < 0 && m_learned.count(k) && m_link->badRequest(k)) {
		// 그림자에서 정상으로 돌아오지 않은 함수: 다시 배우지 않는다
		m_learned.erase(k);
		m_bad[k] = 2;
		if (log) {
			char buf[96];
			std::snprintf(buf, sizeof(buf), "dropped request %02X:%04X (did not return in shadow)", static_cast<unsigned>(k >> 16), k & 0xFFFF);
			log(buf);
		}
	}
	if (lv.trapDepth < 0 && m_learned.count(k)) {
		// 배운 요청 함수: 호출을 그림자에 넘긴다
		lv.trapDepth = idx;
		++lv.traps;
		flushWrites();  // 호출 전에 게임이 쓴 것을 먼저
		auto e = std::make_shared<GbDriverEvent>();
		e->type = GbDriverEvent::Call;
		e->entry = f.entry;
		e->romBank = f.romBank;
		e->wramBank = f.wramBank;
		e->regs = f.regs;
		e->requestKey = k;
		auto image = std::make_shared<GbRamImage>();
		if (readRam) {
			readRam(*image);
		}
		e->image = image;
		m_link->post(e);
		++statCalls;
	}
}

void GbDriverTracer::postMark() {
	// 원본이 갱신 루틴을 부른 곳: 그림자는 이 앞의 요청·쓰기를 갱신 전에, 뒤의 것을 갱신 뒤에 넣는다
	flushWrites();
	auto e = std::make_shared<GbDriverEvent>();
	e->type = GbDriverEvent::Mark;
	e->index = static_cast<int>(statFrames);  // 원본 프레임 번호 (한 프레임에 여러 번 부른 것을 가린다)
	m_link->post(e);
}

void GbDriverTracer::finishBurst(Level& lv) {
	Burst& b = lv.burst;
	if (b.lcaDepth >= 0 && b.closed && b.lcaPure && !b.lca.isr && m_link->map.updates >= 60) {
		const uint32_t k = key(b.lca.entry, b.lca.romBank);
		// 갱신 루틴 자신은 요청 함수가 아니다 (갱신 루틴을 확정하기 전이나 다른 곳에서도 부르는 게임)
		if (m_bad[k] >= 2 || inRoutines(k) || (m_update.valid && k == key(m_update.entry, m_update.romBank))) {
			b = Burst();
			return;
		}
		if (m_learned.insert(k).second && log) {
			char buf[96];
			std::snprintf(buf, sizeof(buf), "learned request %02X:%04X", static_cast<unsigned>(k >> 16), k & 0xFFFF);
			log(buf);
		}
	}
	b = Burst();
}

std::vector<GbDriverTracer::Frame> GbDriverTracer::soundRoutines(const Level& lv) const {
	std::vector<Frame> out;
	for (const Level::SndPath& sp : lv.sndPaths) {
		std::vector<Frame> path = sp.frames;
		for (Frame& fr : path) {
			if (fr.tailEntry) {
				fr.entry = fr.tailEntry;
				fr.romBank = fr.tailBank;
				fr.regs = fr.tailRegs;
			}
		}
		// 이미 아는 갱신 루틴을 지나는 갈래는 그 루틴의 것, 배운 요청 함수를 지나는 갈래는 요청이다
		int known = -1;
		bool request = false;
		for (size_t i = 0; i < path.size(); ++i) {
			const uint32_t pk = key(path[i].entry, path[i].romBank);
			if (known < 0 && (inRoutines(pk) || (m_update.valid && pk == key(m_update.entry, m_update.romBank)))) {
				known = static_cast<int>(i);
			}
			request = request || m_learned.count(pk);
		}
		if (request && known < 0) {
			continue;
		}
		const int w = known >= 0 ? known : widenIndex(path, sp.pure);
		const Frame& c = path[w];
		if (c.isr) {
			continue;  // 인터럽트 처리 함수가 직접 쓴 것 (갱신 루틴으로 부를 수 없다)
		}
		const uint32_t k = key(c.entry, c.romBank);
		bool seen = false;
		for (const Frame& o : out) {
			seen = seen || key(o.entry, o.romBank) == k;
		}
		if (!seen) {
			out.push_back(c);
		}
	}
	return out;
}

bool GbDriverTracer::inRoutines(uint32_t k) const {
	for (const Frame& r : m_routines) {
		if (key(r.entry, r.romBank) == k) {
			return true;
		}
	}
	return false;
}

void GbDriverTracer::publishRoutines() {
	// 갱신 루틴을 요청 함수로 배워 두었으면 잊는다
	for (const Frame& r : m_routines) {
		m_learned.erase(key(r.entry, r.romBank));
	}
	std::vector<GbDriverLink::Routine> list;
	for (const Frame& r : m_routines) {
		GbDriverLink::Routine x;
		x.valid = true;
		x.entry = r.entry;
		x.romBank = r.romBank;
		x.wramBank = r.wramBank;
		x.regs = r.regs;
		list.push_back(x);
	}
	m_link->setRoutines(list);
}

void GbDriverTracer::addRoutines(const std::vector<Frame>& list) {
	bool changed = false;
	int at = -1;  // 이 인터럽트에서 앞에 나온 갱신 루틴의 목록 위치
	for (const Frame& n : list) {
		const uint32_t k = key(n.entry, n.romBank);
		int found = -1;
		for (size_t i = 0; i < m_routines.size(); ++i) {
			if (key(m_routines[i].entry, m_routines[i].romBank) == k) {
				found = static_cast<int>(i);
			}
		}
		if (found >= 0) {
			at = found;
			continue;
		}
		// 확정 뒤 새로 보인 함수는 여러 인터럽트에서 되풀이될 때만 갱신 루틴으로 받는다 (한 번 부르는 요청 경로를 거른다)
		if (++m_addVotes[k] < kMainVotes) {
			continue;
		}
		m_routines.insert(m_routines.begin() + (at + 1), n);
		at = at + 1;
		changed = true;
		if (log) {
			char buf[96];
			std::snprintf(buf, sizeof(buf), "update routine added %02X:%04X", n.romBank, n.entry);
			log(buf);
		}
	}
	if (changed) {
		publishRoutines();
	}
}

bool GbDriverTracer::canWiden(const Frame& parent, const Frame& child) const {
	if (sameArea(parent, child)) {
		return true;
	}
	if (parent.entry >= 0x4000 || child.entry < 0x4000 || !m_rom) {
		return false;
	}
	const std::vector<uint8_t>& rom = *m_rom;
	const uint8_t lo = child.entry & 0xFF, hi = child.entry >> 8;
	for (size_t a = parent.entry; a + 2 < 0x4000 && a + 2 < rom.size() && a < static_cast<size_t>(parent.entry) + 0x200; ++a) {
		const uint8_t op = rom[a];
		const bool callJump = op == 0xCD || op == 0xC4 || op == 0xCC || op == 0xD4 || op == 0xDC || op == 0xC3 || op == 0xC2 ||
		                      op == 0xCA || op == 0xD2 || op == 0xDA;
		if (callJump && rom[a + 1] == lo && rom[a + 2] == hi) {
			return true;
		}
	}
	return false;
}

void GbDriverTracer::popTop() {
	const int d = static_cast<int>(m_frames.size()) - 1;
	const Frame f = m_frames.back();
	Level& lv = m_levels.back();
	if (d == m_updateDepth) {
		m_updateDepth = -1;
	}
	if (d == lv.trapDepth) {
		lv.trapDepth = -1;
		const uint32_t fk = key(f.entry, f.romBank);
		if (lv.impure != f.impureMark && !m_mainSnd.count(fk)) {
			// 배운 함수가 이번에는 게임 RAM·화면을 건드렸다 (범용 복사 함수 등). 두 번이면 잊는다
			// (주 흐름의 사운드 함수는 자기 상태 RAM 을 쓰므로 빼고: 그림자가 같은 RAM 으로 돌린다)
			const uint32_t k = fk;
			if (++m_bad[k] >= 2) {
				m_learned.erase(k);
				if (log) {
					char buf[96];
					std::snprintf(buf, sizeof(buf), "forgot request %02X:%04X", static_cast<unsigned>(k >> 16), k & 0xFFFF);
					log(buf);
				}
			}
		}
	}
	if (lv.sndAny && d >= lv.base && lv.hard != f.hardMark) {
		const size_t i = static_cast<size_t>(d - lv.base);
		for (Level::SndPath& sp : lv.sndPaths) {
			if (i < sp.frames.size() && sp.frames[i].sp == f.sp && sp.frames[i].entry == f.entry) {
				sp.pure[i] = 0;
			}
		}
	}
	if (lv.sndAny && d >= lv.base) {
		// 경로 위 함수가 끝남: 화면·저장 RAM·사운드 밖 I/O 를 건드렸는지 적는다 (드라이버 RAM 을 아직 모를 때도 알 수 있는 것만) (갱신 루틴을 위로 넓힐 때 본다)
		const size_t i = static_cast<size_t>(d - lv.base);
		if (i < lv.sndTrap.size() && lv.traps != f.trapMark) {
			lv.sndTrap[i] = 1;
		}
		if (i < lv.sndPure.size() && lv.hard != f.hardMark) {
			lv.sndPure[i] = 0;
		}
	}
	if (lv.sndDepth >= 0 && d <= lv.sndBound) {
		lv.sndBound = d - 1;
	}
	Burst& b = lv.burst;
	if (b.lcaDepth >= 0 && d <= b.bound + (b.closed ? 0 : 0)) {
		const bool pure = lv.impure == f.impureMark;
		if (!b.closed && d == b.lcaDepth) {
			b.closed = true;
			b.lcaPure = pure;
			b.bound = d - 1;
		} else if (b.closed && d == b.bound) {
			// 품은 함수를 부른 함수가 끝남: 이 함수도 게임 RAM 을 건드리지 않았으면 요청 함수를 이 함수로 넓힌다
			const int minDepth = lv.base + (m_levels.size() > 1 ? 1 : 0);
			if (pure && b.lcaPure && !f.isr && d >= minDepth && canWiden(f, b.lca)) {
				b.lca = f;
				b.lcaDepth = d;
				b.bound = d - 1;
			} else {
				finishBurst(lv);
			}
		}
	}
	if (f.isr && m_levels.size() > 1) {
		// 인터럽트 끝: 사운드 레지스터를 쓴 코드를 모두 품은 함수가 갱신 루틴 후보
		// 사운드를 쓴 갈래가 여럿이고 그것들을 함께 품는 함수가 게임 쪽 일도 하면(인터럽트 처리 함수 등),
		// 갈래마다 찾은 함수를 모두 갱신 루틴으로 둔다 (원본과 같은 순서로 부른다)
		bool skipLca = false;
		if (lv.sndAny) {
			const std::vector<Frame> list = soundRoutines(lv);
			if (m_update.valid && m_updIdle) {
				for (const Frame& r : list) {
					if (inRoutines(key(r.entry, r.romBank))) {
						m_updIdle = false;
					}
				}
			}
			if (!m_update.valid && !list.empty()) {
				const uint32_t k0 = key(list[0].entry, list[0].romBank);
				if (m_listVotes && k0 == m_listKey) {
					++m_listVotes;
				} else {
					m_listKey = k0;
					m_listVotes = 1;
				}
			}
			// 갈래 하나: 여러 인터럽트의 공통 경로가 인터럽트 처리 함수까지 줄어 그 경로로는 고를 수 없으면 갈래로 고른다
			const bool lcaBlocked = !m_update.valid && list.size() == 1 && m_votes >= kVotes && !m_candPath.empty() &&
			                        m_candPath[widenIndex(m_candPath, m_candPure)].isr;
			if (list.size() > 1 || lcaBlocked) {
				skipLca = true;
				if (!m_update.valid) {
					if (m_listVotes >= kVotes) {
						const Frame& c = list[0];
						m_update.valid = true;
						m_updIdle = true;
						m_update.entry = c.entry;
						m_update.romBank = c.romBank;
						m_update.wramBank = c.wramBank;
						m_update.regs = c.regs;
						m_link->setUpdate(m_update);
						m_updPath.clear();
						m_updPure.clear();
						m_updIndex = 0;
						m_routines = list;
						publishRoutines();
						m_needInit = true;
						if (log) {
							std::string t = "update routines";
							for (const Frame& r : list) {
								char b[24];
								std::snprintf(b, sizeof(b), " %02X:%04X", r.romBank, r.entry);
								t += b;
							}
							log(t);
						}
					}
				} else {
					addRoutines(list);
				}
			} else if (list.size() == 1 && m_update.valid) {
				const uint32_t k = key(list[0].entry, list[0].romBank);
				if (inRoutines(k) && k != key(m_update.entry, m_update.romBank)) {
					skipLca = true;  // 다른 갱신 루틴만 쓴 인터럽트
				}
			}
		}
		if (lv.sndAny && !skipLca) {
			// 이번 인터럽트의 경로 (인터럽트 첫 함수 ~ 사운드를 쓴 코드를 모두 품은 함수).
			// 꼬리 호출로 넘겨준 함수는 넘겨받은 곳으로 바꾼다
			std::vector<Frame> path = lv.sndPath;
			std::vector<uint8_t> pure = lv.sndPure;
			if (path.empty()) {
				path.push_back(lv.snd);
				pure.push_back(0);
			}
			for (size_t i = 0; i + 1 < path.size(); ++i) {
				if (path[i].tailEntry) {
					path[i].entry = path[i].tailEntry;
					path[i].romBank = path[i].tailBank;
					path[i].regs = path[i].tailRegs;
				}
			}
			if (lv.sndTailOk) {
				path.back().entry = lv.snd.tailEntry;
				path.back().romBank = lv.snd.tailBank;
				path.back().regs = lv.snd.tailRegs;
			}
			if (m_update.valid) {
				// 확정한 뒤에도 지켜본다.
				// (1) 같은 인터럽트 경로에서 갈라져 다른 함수가 사운드를 쓰면 (채널마다 다른 함수를 번갈아 부르는 드라이버 등)
				//     두 경로가 함께 지나는 함수로 넓힌다
				// (2) 경로가 전혀 다른 후보가 오래 이어지면 (게임 시작 직후의 초기화 코드 등을 잘못 잡은 것) 그쪽으로 바꾼다.
				// 바꿀 때는 잘못 배운 요청 함수와 드라이버 RAM 지도를 버린 뒤 그림자를 원본 상태로 다시 맞춘다
				const size_t p = commonPath(m_updPath, path);
				Frame next;
				bool change = false;
				if (p > static_cast<size_t>(m_updIndex)) {
					mergePath(m_updPath, m_updPure, path, pure, p);
					m_challengeVotes = 0;
					m_updIdle = false;
					const int w = widenIndex(m_updPath, m_updPure);
					if (w != m_updIndex) {
						m_updIndex = w;
						next = m_updPath[w];
						change = true;
					}
				} else if (p >= 2 && canWidenChain(m_updPath, m_updIndex, static_cast<int>(p) - 1)) {
					mergePath(m_updPath, m_updPure, path, pure, p);
					m_challengeVotes = 0;
					m_updIndex = widenIndex(m_updPath, m_updPure);
					next = m_updPath[m_updIndex];
					change = true;
				} else {
					const size_t q = commonPath(m_challPath, path);
					if (m_challengeVotes && q >= 1) {
						mergePath(m_challPath, m_challPure, path, pure, q);
						if (++m_challengeVotes >= (m_updIdle ? kSwitchVotesIdle : kSwitchVotes)) {
							m_updPath = m_challPath;
							m_updPure = m_challPure;
							m_updIndex = widenIndex(m_updPath, m_updPure);
							next = m_updPath[m_updIndex];
							change = true;
							m_challengeVotes = 0;
						}
					} else {
						m_challPath = path;
						m_challPure = pure;
						m_challengeVotes = 1;
					}
				}
				if (change && !next.isr && key(next.entry, next.romBank) != key(m_update.entry, m_update.romBank)) {
					m_update.entry = next.entry;
					m_update.romBank = next.romBank;
					m_update.wramBank = next.wramBank;
					m_update.regs = next.regs;
					m_link->setUpdate(m_update);
					m_updIdle = true;
					m_routines.assign(1, next);
					m_addVotes.clear();
					publishRoutines();
					m_learned.clear();
					m_mainSndVotes.clear();
					m_mainSnd.clear();
					m_bad.clear();
					m_link->map.clear();
					m_warmed = false;
					m_needInit = true;
					if (log) {
						char buf[96];
						std::snprintf(buf, sizeof(buf), "update routine changed to %02X:%04X", next.romBank, next.entry);
						log(buf);
					}
				}
			} else {
				const size_t p = commonPath(m_candPath, path);
				if (m_votes && p >= 1) {
					mergePath(m_candPath, m_candPure, path, pure, p);
					++m_votes;
				} else {
					m_candPath = path;
					m_candPure = pure;
					m_votes = 1;
				}
				if (m_votes >= kVotes && !m_candPath[widenIndex(m_candPath, m_candPure)].isr) {
					m_updPath = m_candPath;
					m_updPure = m_candPure;
					m_updIndex = widenIndex(m_updPath, m_updPure);
					const Frame& c = m_updPath[m_updIndex];
					m_update.valid = true;
					m_updIdle = true;
					m_update.entry = c.entry;
					m_update.romBank = c.romBank;
					m_update.wramBank = c.wramBank;
					m_update.regs = c.regs;
					m_link->setUpdate(m_update);
					m_routines.assign(1, c);
					m_addVotes.clear();
					publishRoutines();
					m_needInit = true;
					if (log) {
						char buf[96];
						std::snprintf(buf, sizeof(buf), "update routine %02X:%04X", c.romBank, c.entry);
						log(buf);
					}
				}
			}
		}
		finishBurst(lv);
		m_levels.pop_back();
	}
	m_frames.pop_back();
}

void GbDriverTracer::fetch(const GbShadowCpu::Regs& r, uint8_t opcode, int romBank, int wramBank) {
	// 스택 옮기기: ld sp,nn / ld sp,hl / add sp,e 로 SP 가 크게 바뀌면 다른 스택을 쓰는 중이다
	// (pop 으로 빠르게 복사하는 코드 등). 원래 SP 근처로 돌아올 때까지 호출 스택을 지우지도 쌓지도 않는다
	if (m_spLoaded) {
		m_spLoaded = false;
		if (m_foreignStack) {
			// 원래 SP 근처로 돌아왔으면 끝 (여러 번 옮겼다 돌아오는 코드도 있다)
			const int back = static_cast<int>(r.sp) - static_cast<int>(m_savedSp);
			if (back >= -0x80 && back <= 0x80) {
				m_foreignStack = false;
			}
		} else if (!m_frames.empty()) {
			const int jump = static_cast<int>(r.sp) - static_cast<int>(m_lastSp);
			if (jump > 0x80 || jump < -0x80) {
				m_foreignStack = true;
				m_savedSp = m_lastSp;
				m_foreignSteps = 0;
			}
		}
	}
	if (opcode == 0x31 || opcode == 0xF9 || opcode == 0xE8) {
		m_spLoaded = true;
	}
	if (m_foreignStack && ++m_foreignSteps > 50000) {
		m_foreignStack = false;  // 돌아오지 않는다: 아예 옮긴 것으로 본다
		dropStack();
	}
	if (m_foreignStack) {
		m_pendingCall = false;
		m_pendingJump = false;
		m_lastPc = r.pc;
		return;
	}
	m_lastSp = r.sp;
	m_lastPc = r.pc;
	if (m_pendingCall) {
		m_pendingCall = false;
		if (r.sp == static_cast<uint16_t>(m_pendingSp - 2)) {
			Frame f;
			f.entry = r.pc;
			f.romBank = romBank;
			f.wramBank = wramBank;
			f.sp = r.sp;
			f.regs = r;
			push(f);
		}
	}
	while (!m_frames.empty() && r.sp > m_frames.back().sp) {
		popTop();
	}
	if (m_pendingJump) {
		m_pendingJump = false;
		if (r.pc != m_jumpNext && r.pc < 0x8000 && !m_frames.empty()) {
			Frame& top = m_frames.back();
			const bool otherArea = (r.pc < 0x4000) != (top.entry < 0x4000);
			const int dist = static_cast<int>(r.pc) - static_cast<int>(top.entry);
			if (!top.isr && (otherArea || dist > 0x400 || dist < -0x400)) {
				top.tailEntry = r.pc;
				top.tailBank = romBank;
				top.tailRegs = r;
				top.tailSerial = ++m_tailCounter;
				// 갱신 루틴으로 꼬리 호출해 들어감
				if (m_update.valid && m_updateDepth < 0 && key(r.pc, romBank) == key(m_update.entry, m_update.romBank)) {
					m_updateDepth = static_cast<int>(m_frames.size()) - 1;
					m_updVector = m_levels.size() > 1 ? m_levels.back().vector : 0;
					++m_updateCalls;
					++m_frameCalls;
					postMark();
					m_update.regs = r;
					m_update.wramBank = wramBank;
					if (r.pc < 0x4000) {
						m_update.romBank = romBank;
					}
					m_link->setUpdate(m_update);
				}
			}
		}
	}
	if (opcode == 0xC3 || opcode == 0xE9) {
		m_pendingJump = true;
		m_jumpNext = static_cast<uint16_t>(r.pc + (opcode == 0xC3 ? 3 : 1));
	}
	if (isCallOpcode(opcode)) {
		m_pendingCall = true;
		m_pendingSp = r.sp;
	} else if (opcode == 0x76) {
		impure();  // HALT: 요청 함수는 기다리지 않는다
	}
}

void GbDriverTracer::irq(const GbShadowCpu::Regs& r, uint16_t returnPc, int romBank, int wramBank) {
	if (m_foreignStack) {
		return;  // 다른 스택을 쓰는 중 (fetch 참고)
	}
	const uint16_t before = static_cast<uint16_t>(r.sp + 2);
	m_lastSp = before;
	if (m_pendingCall) {
		m_pendingCall = false;
		if (before == static_cast<uint16_t>(m_pendingSp - 2)) {
			// 호출 직후 첫 명령 전에 인터럽트가 걸림
			Frame f;
			f.entry = returnPc;
			f.romBank = romBank;
			f.wramBank = wramBank;
			f.sp = before;
			f.regs = r;
			f.regs.sp = before;
			f.regs.pc = returnPc;
			push(f);
		}
	}
	while (!m_frames.empty() && before > m_frames.back().sp) {
		popTop();
	}
	Frame f;
	f.entry = r.pc;
	f.romBank = romBank;
	f.wramBank = wramBank;
	f.sp = r.sp;
	f.isr = true;
	f.regs = r;
	push(f);
}

// 인터럽트 안의 사운드 레지스터 쓰기 하나: 쓴 코드를 모두 품는 함수(와 그 함수가 꼬리 호출로 넘겨준 곳)를 갱신한다
void GbDriverTracer::noteSoundWrite(Level& lv) {
	const int t = static_cast<int>(m_frames.size()) - 1;
	if (t >= lv.base && lv.sndPaths.size() < 16) {
		const size_t n = static_cast<size_t>(t - lv.base + 1);
		const bool same = !lv.sndPaths.empty() && lv.sndPaths.back().frames.size() == n &&
		                  lv.sndPaths.back().frames.back().sp == m_frames[t].sp && lv.sndPaths.back().frames.back().entry == m_frames[t].entry;
		if (!same) {
			Level::SndPath sp;
			sp.frames.assign(m_frames.begin() + lv.base, m_frames.begin() + t + 1);
			sp.pure.assign(n, 1);
			lv.sndPaths.push_back(std::move(sp));
		}
	}
	bool moved = false;
	if (!lv.sndAny) {
		lv.sndAny = true;
		lv.sndDepth = t;
		lv.sndBound = t;
		moved = true;
	} else if (lv.sndBound < lv.sndDepth) {
		lv.sndDepth = std::max(lv.sndBound, lv.base);
		lv.sndBound = lv.sndDepth;
		moved = true;
	}
	const Frame& f = m_frames[lv.sndDepth];
	if (moved) {
		// 경로: 처음에는 지금 호출 스택을 담고, 품는 함수가 위로 올라가면 그 아래를 잘라 낸다
		// (남은 함수는 처음부터 살아 있던 같은 함수들이다)
		const size_t n = lv.sndDepth >= lv.base ? static_cast<size_t>(lv.sndDepth - lv.base + 1) : 0;
		if (lv.sndPath.empty()) {
			lv.sndPath.assign(m_frames.begin() + lv.base, m_frames.begin() + lv.base + n);
			lv.sndPure.assign(n, 1);
			lv.sndTrap.assign(n, 0);
		} else if (n < lv.sndPath.size()) {
			lv.sndPath.resize(n);
			lv.sndPure.resize(n);
			lv.sndTrap.resize(n);
		}
		lv.snd = f;
		lv.sndTailSerial = f.tailSerial;
		lv.sndTailOk = f.tailEntry != 0;
	} else if (f.tailSerial != lv.sndTailSerial) {
		lv.sndTailOk = false;  // 넘겨주기 전에도 썼거나 여러 곳으로 넘겨줬다
	}
}

void GbDriverTracer::store(uint16_t addr, uint8_t value, uint16_t /*sp*/, int wramBank) {
	if (m_foreignStack) {
		return;  // 다른 스택을 쓰는 중 (fetch 참고)
	}
	// push / call / 인터럽트의 스택 쓰기: 마지막 명령을 가져올 때의 SP 근처.
	// (명령 도중의 SP 는 코어에 따라 다른 값이 들어 있고, 인터럽트가 걸릴 때는 몇 바이트 어긋나므로 SP-4 ~ SP+2)
	if (static_cast<uint16_t>(addr - static_cast<uint16_t>(m_lastSp - 4)) <= 6u) {
		return;
	}
	// 지역 변수: 지금 SP 부터 이 흐름(주 흐름 또는 인터럽트)의 첫 함수가 불린 때의 SP 까지는 지금 돌고 있는 함수들의 스택이다
	// (C 로 만든 코드는 add sp,-n 으로 자리를 만들어 지역 변수를 둔다). 게임 상태도 드라이버 RAM 도 아니다
	if (!m_frames.empty() && !m_levels.empty()) {
		const int base = m_levels.back().base;
		if (base >= 0 && base < static_cast<int>(m_frames.size())) {
			const int top = m_frames[base].sp + 1;
			if (static_cast<int>(addr) >= static_cast<int>(m_lastSp) - 4 && static_cast<int>(addr) <= top) {
				return;
			}
		}
	}
	if (addr < 0x8000) {
		return;  // 롬 bank 바꾸기 등
	}
	if (addr < 0xC000 || (addr >= 0xFE00 && addr < 0xFF00) || addr == 0xFFFF) {
		m_levels.back().hard++;
		impure(addr);  // 화면 메모리, 저장 RAM, OAM, 인터럽트 설정
		return;
	}
	if (addr >= 0xFF00 && addr < 0xFF80) {
		const uint8_t reg = addr & 0x7F;
		if (reg >= 0x10 && reg < 0x40) {
			// 사운드 레지스터 쓰기: 쓴 코드를 모두 품는 함수를 찾는다.
			// 인터럽트 안이면 인터럽트 끝에서, 주 흐름이면(화면을 기다린 뒤 메인 루프에서 드라이버를 부르는 게임) 프레임 끝에서 본다
			if (m_levels.size() > 1) {
				Level& lv = m_levels.back();
				noteSoundWrite(lv);
			} else if (!m_frames.empty() && (!m_update.valid || (!inUpdate() && m_levels.back().trapDepth < 0))) {
				// 갱신 루틴을 찾은 뒤에도 주 흐름에서 쓰는 사운드 레지스터는 모은다 (게임 코드가 직접 부르는 효과음 엔진 등).
				// 그림자에 넘긴 요청 함수 안의 쓰기는 그림자가 그 함수를 돌며 낸다
				noteSoundWrite(m_levels.back());
				if (m_update.valid) {
					// 그 쓰기는 그대로 그림자에 넘긴다 (곡 시작 함수가 직접 쓰는 좌우 출력 등). 앞의 RAM 쓰기와 순서를 지킨다
					flushWrites();
					auto e = std::make_shared<GbDriverEvent>();
					e->type = GbDriverEvent::SoundWrite;
					e->index = reg;
					e->value = value;
					m_link->post(e);
				}
			}
			return;
		}
		if (reg != 0x70 && reg != 0x0F) {
			m_levels.back().hard++;
			impure(addr);
		}
		return;
	}
	if (inUpdate()) {
		return;
	}
	const int idx = GbShadowCpu::ramIndex(addr, wramBank);
	if (idx < 0) {
		return;
	}
	// 상태 넘기기: 갱신 루틴 밖의 코드가 쓴 RAM 은 (드라이버 RAM 으로 아직 모르는 칸도) 모두 그림자에 넘긴다.
	// 갱신 루틴 자신의 쓰기(원본 드라이버의 진행)와 그림자에 넘긴 요청 함수 안의 쓰기만 뺀다
	if (m_levels.back().trapDepth < 0) {
		queueWrite(idx, value);
	}
	if (!m_link->map.inRegion(idx)) {
		// HRAM 은 롬 bank 기억 같은 공용 임시 값이 많아 게임 코드의 표시로 보지 않는다
		if (addr < 0xFF80) {
			impure(addr);
		}
		return;
	}
	Level& lv = m_levels.back();
	if (lv.trapDepth >= 0) {
		return;  // 그림자에 넘긴 요청 함수 안의 쓰기 (그림자가 그 함수를 직접 돈다)
	}
	const uint8_t flags = m_link->map.get(idx);
	if ((flags & GbDriverRamMap::WrittenUpdate) && !(flags & GbDriverRamMap::ReadFirstUpdate)) {
		return;  // 갱신 루틴이 늘 먼저 쓰는 임시 값 (롬 bank 기억 등)
	}
	if (traceWrites && log) {
		char buf[96];
		const int t0 = static_cast<int>(m_frames.size()) - 1;
		std::snprintf(buf, sizeof(buf), "f%u write %04X=%02X in %02X:%04X depth %d", statFrames, addr, value,
		              t0 >= 0 ? m_frames[t0].romBank : 0, t0 >= 0 ? m_frames[t0].entry : 0, t0);
		log(buf);
	}

	// 요청 함수 배우기: 갱신 루틴이 먼저 읽는 곳(드라이버 상태·입력)에 쓰는 것만 요청으로 본다
	if (!(flags & GbDriverRamMap::ReadFirstUpdate)) {
		return;
	}
	const int t = static_cast<int>(m_frames.size()) - 1;
	const int minDepth = lv.base + (m_levels.size() > 1 ? 1 : 0);
	if (t < minDepth) {
		return;
	}
	Burst& b = lv.burst;
	if (b.lcaDepth >= 0 && b.closed) {
		const int p = b.bound;
		if (p >= minDepth && b.lcaPure && lv.impure == m_frames[p].impureMark) {
			b.lcaDepth = p;
			b.lca = m_frames[p];
			b.closed = false;
			b.bound = p;
			return;
		}
		finishBurst(lv);
	}
	if (b.lcaDepth < 0) {
		++statBursts;
		b.lcaDepth = t;
		b.bound = t;
		b.lca = m_frames[t];
		b.closed = false;
	}
}

// 원본 RAM 사본에서 갱신 루틴을 몇 초 분량 돌린다 (소리는 버림). 그림자가 실제 시간으로 드라이버 RAM 을 알아 가기 전에
// (배속 중에는 그동안 게임이 여러 요청을 한다) 요청을 알아볼 수 있게 한다
void GbDriverTracer::warmUp() {
	if (m_warmed || !m_rom || !m_lastImage || !m_update.valid) {
		return;
	}
	m_warmed = true;
	GbApuSynth apu;
	GbShadowCpu probe(m_rom);
	probe.setApu(&apu);
	probe.setRamMap(&m_link->map);
	probe.loadRam(*m_lastImage);
	probe.setRomBank(m_lastRomBank);
	for (int i = 0; i < 180; ++i) {
		probe.setWramBank(m_update.wramBank);
		probe.call(m_update.entry, m_update.romBank, m_update.regs, kUpdateBudget, GbShadowCpu::Mode::Update);
		m_link->map.updates++;
	}
}

// 프레임 끝: 이번 프레임에 주 흐름에서 쓴 사운드 레지스터를 모두 품는 함수 경로를 모은다 (인터럽트 쪽과 같은 규칙:
// 여러 프레임의 공통 앞부분으로 자르고, 화면·저장 RAM·사운드 밖 I/O 를 건드리지 않은 함수까지 위로 넓힌다).
// 프레임 끝에 아직 살아 있던 함수(끝나지 않는 메인 루프 등)는 고르지 않는다. 인터럽트에서 갱신 루틴을 못 찾았을 때만 쓴다
void GbDriverTracer::mainSoundEnd() {
	if (m_levels.empty()) {
		return;
	}
	Level& lv = m_levels.front();
	if (!lv.sndAny) {
		return;
	}
	std::vector<Frame> path = lv.sndPath;
	std::vector<uint8_t> pure = lv.sndPure;
	const std::vector<uint8_t> trap = lv.sndTrap;
	const bool tailOk = lv.sndTailOk;
	const Frame snd = lv.snd;
	lv.sndAny = false;
	lv.sndDepth = lv.sndBound = -1;
	lv.sndPath.clear();
	lv.sndPure.clear();
	lv.sndTrap.clear();
	lv.sndTailOk = false;
	if (path.empty()) {
		return;
	}
	size_t alive = 0;
	while (alive < path.size() && lv.base + alive < m_frames.size() && m_frames[lv.base + alive].sp == path[alive].sp &&
	       m_frames[lv.base + alive].entry == path[alive].entry) {
		++alive;
	}
	for (size_t i = 0; i + 1 < path.size(); ++i) {
		if (path[i].tailEntry) {
			path[i].entry = path[i].tailEntry;
			path[i].romBank = path[i].tailBank;
			path[i].regs = path[i].tailRegs;
		}
	}
	if (tailOk) {
		path.back().entry = snd.tailEntry;
		path.back().romBank = snd.tailBank;
		path.back().regs = snd.tailRegs;
	}
	// 프레임마다 고른 함수(경로 끝에서 위로 넓힌 것, 프레임 끝에 살아 있던 함수는 빼고)를 센다.
	// 가끔 다른 함수(효과음을 직접 내는 코드 등)가 끼어도, 가장 많이 나온 함수가 충분히 많고 대부분이면 확정한다
	int w = widenIndex(path, pure);
	w = std::max(w, static_cast<int>(alive));
	if (w >= static_cast<int>(path.size())) {
		return;
	}
	if (m_update.valid) {
		// 갱신 루틴과 따로 주 흐름에서 사운드를 쓰는 함수: 여러 프레임에서 같은 함수면 그 호출을 그림자에 넘긴다
		// (요청 함수처럼 부를 때마다 같은 순서로 돌린다). 위로 넓히지 않고 쓰기를 모두 품는 가장 안쪽 함수로
		// (위 함수는 게임 일도 하는 주 루프 쪽 함수일 수 있다)
		const size_t inner = path.size() - 1;
		if (inner < alive || !pure[inner] || (inner < trap.size() && trap[inner])) {
			return;  // 화면·저장 RAM·사운드 밖 I/O 를 건드리거나 배운 요청 함수를 부른 함수는 게임 일을 하는 함수다
		}
		const Frame& c = path[inner];
		const uint32_t k = key(c.entry, c.romBank);
		if (c.isr || inRoutines(k) || k == key(m_update.entry, m_update.romBank) || m_learned.count(k) || m_bad[k] >= 2) {
			return;
		}
		if (++m_mainSndVotes[k] >= kMainSndVotes) {
			m_learned.insert(k);
			m_mainSnd.insert(k);
			m_mainSndVotes.erase(k);
			if (log) {
				char buf[96];
				std::snprintf(buf, sizeof(buf), "learned main-flow sound routine %02X:%04X", c.romBank, c.entry);
				log(buf);
			}
		}
		return;
	}
	const Frame& c = path[w];
	const uint32_t k = key(c.entry, c.romBank);
	MainCand& mc = m_mainCands[k];
	mc.frame = c;
	++mc.count;
	++m_mainTotal;
	if (mc.count < kMainVotes || mc.count * 10 < m_mainTotal * 7) {
		return;
	}
	m_updPath = path;
	m_updPure = pure;
	m_updIndex = w;
	m_update.valid = true;
	m_update.entry = c.entry;
	m_update.romBank = c.romBank;
	m_update.wramBank = c.wramBank;
	m_update.regs = c.regs;
	m_link->setUpdate(m_update);
	m_routines.assign(1, c);
	m_addVotes.clear();
	publishRoutines();
	m_needInit = true;
	m_mainCands.clear();
	m_mainTotal = 0;
	if (log) {
		char buf[96];
		std::snprintf(buf, sizeof(buf), "update routine %02X:%04X (main flow)", c.romBank, c.entry);
		log(buf);
	}
}

void GbDriverTracer::queueWrite(int idx, uint8_t value) {
	m_pendValue[idx] = value;
	if (!m_pendMark[idx]) {
		m_pendMark[idx] = 1;
		m_pendList.push_back(idx);
	}
	++statWrites;
}

// 모아 둔 쓰기를 한 번에 넘긴다 (칸마다 마지막 값). 요청 호출을 넘기기 전과 프레임 끝에 부른다
void GbDriverTracer::flushWrites() {
	if (m_pendList.empty()) {
		return;
	}
	auto e = std::make_shared<GbDriverEvent>();
	e->type = GbDriverEvent::WriteBatch;
	e->batch.reserve(m_pendList.size());
	for (int idx : m_pendList) {
		e->batch.emplace_back(idx, m_pendValue[idx]);
		m_pendMark[idx] = 0;
	}
	m_pendList.clear();
	m_link->post(e);
}

void GbDriverTracer::postInit() {
	if (!m_lastImage) {
		return;
	}
	warmUp();
	auto e = std::make_shared<GbDriverEvent>();
	e->type = GbDriverEvent::Init;
	e->image = m_lastImage;
	e->romBank = m_lastRomBank;
	e->wramBank = m_lastWramBank;
	e->io = m_lastIo;
	std::copy(m_lastIo.begin() + 0x10, m_lastIo.begin() + 0x40, e->sound.begin());
	m_link->post(e);
	m_needInit = false;
}

void GbDriverTracer::frameEnd(std::shared_ptr<const GbRamImage> image, const std::array<uint8_t, 0x80>& io, int romBank,
                              int wramBank, double frameCycles) {
	flushWrites();
	m_lastImage = std::move(image);
	m_lastIo = io;
	m_lastRomBank = romBank;
	m_lastWramBank = wramBank;
	m_link->setLatest(m_lastImage);
	++m_frameCount;
	++statFrames;
	mainSoundEnd();
	// 원본이 갱신 루틴을 실제 시간에 맞지 않게 불렀으면 (불러오기 등으로 소리 드라이버를 멈춘 프레임, 화면을 꺼서 길어진 프레임,
	// 타이머를 멈춘 동안) 그림자도 따라가게 알린다. 그림자는 실제 시간 박자로 돌기 때문에, 원본이 이 시간 동안 불렀어야 할 횟수와
	// 실제로 부른 횟수의 차이를 넘긴다.
	// 타이머 인터럽트로 부르는 드라이버: 간격은 타이머 설정(TMA·TAC)으로 정확히 안다 (곡마다 빠르기를 타이머로 바꾸는 드라이버가 있다).
	// 그 밖: 프레임마다 정수 번 부르는 드라이버만 (그 밖의 드라이버는 프레임마다 횟수가 원래 들쭉날쭉하다)
	{
		double expected = -1;
		if (m_update.valid && m_updVector == 0x50) {
			const uint8_t tac = io[0x07];
			if (tac & 4) {
				static const int kPeriod[4] = {1024, 16, 64, 256};
				const bool doubleSpeed = io[0x4D] & 0x80;
				m_timerInterval = (256 - io[0x06]) * kPeriod[tac & 3] / (doubleSpeed ? 2.0 : 1.0);
			}
			if (m_timerInterval > 0) {
				m_link->intervalCycles = m_timerInterval;
				// 타이머가 멈춰 있으면 그 시간만큼 그림자도 멈춘다 (마지막 간격으로 센 횟수를 건너뛴다)
				expected = frameCycles / m_timerInterval;
			}
		} else {
			m_link->intervalCycles = 0.0;
			const double n = m_link->callsPerFrame.load();
			const int ni = static_cast<int>(std::lround(n));
			const int ki = n > 0 ? static_cast<int>(std::lround(1 / n)) : 0;
			if (m_update.valid && ((ni >= 1 && std::fabs(n - ni) < 1e-9) || (ki >= 2 && std::fabs(n * ki - 1) < 1e-9))) {
				expected = n * (frameCycles / kFrameCycles);
			}
		}
		if (expected >= 0 && !m_needInit) {
			m_paceCarry += expected - m_frameCalls;
			// 보통 프레임의 몇 사이클 차이는 모아 두었다가 넘긴다.
			// k 프레임마다 한 번 부르는 드라이버는 부르지 않는 프레임에도 모자람이 쌓이므로 (1 - 1/k) 까지는 제 박자로 본다
			const double cpf = m_link->callsPerFrame.load();
			const double upper = cpf < 1 ? 1 - cpf : 0.0;
			double pace = 0;
			if (m_paceCarry > upper + 0.1) {
				pace = m_paceCarry - upper;
			} else if (m_paceCarry <= -0.1) {
				pace = m_paceCarry;
			}
			if (pace != 0) {
				auto e = std::make_shared<GbDriverEvent>();
				e->type = GbDriverEvent::Pace;
				e->pace = pace;
				m_link->post(e);
				m_paceCarry -= pace;
			}
		} else {
			m_paceCarry = 0;
		}
		m_frameCalls = 0;
	}
	if (m_frameCount >= 60) {
		// 프레임마다 갱신 루틴을 부르는 횟수 (타이머 인터럽트로 부르는 드라이버는 1 이 아니다)
		// 불러오기 등으로 잠깐 덜 부르는 구간은 무시한다: 앞 구간과 5% 안으로 같을 때만 바꾼다
		if (m_updateCalls) {
			const double rate = static_cast<double>(m_updateCalls) / m_frameCount;
			if (std::fabs(rate - m_rate) <= 0.05 * m_rate) {
				double r = (rate + m_rate) / 2;
				// 프레임마다 n 번 (화면 인터럽트) 이면 정확히 n 으로
				const double n = std::round(r);
				const double k = r > 0 ? std::round(1 / r) : 0;
				if (n >= 1 && std::fabs(r - n) <= 0.03 * n) {
					r = n;
				} else if (k >= 2 && std::fabs(r * k - 1) <= 0.03) {
					r = 1 / k;  // k 프레임마다 한 번
				}
				m_link->callsPerFrame = std::max(0.25, std::min(8.0, r));
			}
			m_rate = rate;
		}
		m_updateCalls = 0;
		m_frameCount = 0;
	}
	if (m_link->initRequested.exchange(false) && m_update.valid) {
		m_needInit = true;
	}
	if (m_needInit) {
		postInit();
	}
}

// ================================================================ GbDriverShadow

GbDriverShadow::GbDriverShadow(std::shared_ptr<const std::vector<uint8_t>> rom, std::shared_ptr<GbDriverLink> link,
                               bool recording)
	: m_link(std::move(link))
	, m_recording(recording)
	, m_cpu(std::move(rom))
{
	m_queue = m_link->subscribe();
	m_cpu.setApu(&m_apu);
	m_cpu.setRamMap(&m_link->map);
	if (s_nativeApu) {
		m_native = s_nativeApu();
	}
	if (m_native) {
		m_cpu.setSoundHook([this](uint8_t reg, uint8_t value) {
			if (m_inRequest) {
				m_requestWrites.emplace_back(reg, value);
			} else {
				m_native->write(reg, value);
			}
		});
	}
}

std::function<std::unique_ptr<GbNativeApu>()> GbDriverShadow::s_nativeApu;

void GbDriverShadow::nativeWrite(uint8_t reg, uint8_t value) {
	if (m_native) {
		m_native->write(reg, value);
	}
}

GbDriverShadow::~GbDriverShadow() {
	m_link->unsubscribe(m_queue);
}

void GbDriverShadow::apply(const GbDriverEvent& e) {
	switch (e.type) {
	case GbDriverEvent::Init: {
		m_cpu.loadRam(*e.image);
		m_cpu.setRomBank(e.romBank);
		m_cpu.setWramBank(e.wramBank);
		for (int r = 0; r < 0x80; ++r) {
			if (r < 0x10 || r >= 0x40) {
				m_cpu.setIo(static_cast<uint8_t>(r), e.io[r]);
			}
		}
		m_apu.write(0x26, e.sound[0x26 - 0x10] | 0x80);
		nativeWrite(0x26, e.sound[0x26 - 0x10] | 0x80);
		// 파형 RAM 을 먼저 (채널 3 이 울리는 동안은 파형 RAM 을 바로 쓸 수 없다)
		for (int r = 0x30; r < 0x40; ++r) {
			m_apu.write(static_cast<uint8_t>(r), e.sound[r - 0x10]);
			nativeWrite(static_cast<uint8_t>(r), e.sound[r - 0x10]);
			m_cpu.setIo(static_cast<uint8_t>(r), e.sound[r - 0x10]);
		}
		// 원본에서 울리고 있는 채널(NR52 의 채널 상태 비트)은 그림자에서도 켠다. 한 번 켠 뒤 주파수만 바꾸며 계속 울리는
		// 드라이버는 다시 켜지 않으므로, 켜지 않으면 그 채널이 끝까지 울리지 않는다. 울리지 않는 채널은 켜지 않는다
		const uint8_t status = e.sound[0x26 - 0x10];
		for (int r = 0x10; r < 0x26; ++r) {
			uint8_t v = e.sound[r - 0x10];
			const int ch = r == 0x14 ? 0 : r == 0x19 ? 1 : r == 0x1E ? 2 : r == 0x23 ? 3 : -1;
			if (ch >= 0) {
				v = (status >> ch) & 1 ? (v | 0x80) : (v & 0x7F);
			}
			m_apu.write(static_cast<uint8_t>(r), v);
			nativeWrite(static_cast<uint8_t>(r), v);
			m_cpu.setIo(static_cast<uint8_t>(r), v & (ch >= 0 ? 0x7F : 0xFF));
		}
		m_ready = true;
		break;
	}
	case GbDriverEvent::Pace:
		break;  // pollPace 에서 다룬다
	case GbDriverEvent::Mark:
		break;
	case GbDriverEvent::SoundWrite:
		if (m_ready) {
			const uint8_t r = static_cast<uint8_t>(e.index);
			m_apu.write(r, e.value);
			nativeWrite(r, e.value);
			m_cpu.setIo(r, e.value);
		}
		break;
	case GbDriverEvent::Write:
		if (m_ready) {
			m_cpu.setRamByte(e.index, e.value);
		}
		break;
	case GbDriverEvent::WriteBatch:
		if (m_ready) {
			for (const auto& w : e.batch) {
				m_cpu.setRamByte(w.first, w.second);
			}
		}
		break;
	case GbDriverEvent::Call: {
		if (!m_ready || !e.image) {
			break;
		}
		// RAM 은 원본에서 갱신 루틴 밖의 쓰기를 모두 넘겨받고 있으므로(WriteBatch) 원본 사본으로 덮지 않는다.
		// 덮으면 그림자가 진행 중인 상태(1배속이라 원본보다 뒤에 있는 소리의 저장 값 등)가 원본 시점 값으로 바뀐다
		// 요청 함수가 정상으로 돌아오지 않으면(화면을 기다리는 HALT, 한도 초과, 잘못된 명령 = 게임 코드)
		// 그림자 상태를 부르기 전으로 되돌리고, 그 함수는 요청 함수가 아니라고 알린다
		const GbShadowCpu cpuBefore = m_cpu;
		const GbApuSynth apuBefore = m_apu;
		m_cpu.setWramBank(e.wramBank);
		// 스택으로 인수를 넘기는 함수 (C 로 만든 게임): 부를 때의 원본 스택(SP 부터 64 바이트)을 넘겨받은 사본에서 가져온다.
		// 스택 쓰기는 지역 변수라 RAM 넘기기에서 빠지므로, 그림자 스택에는 원본이 넣은 인수가 없다
		for (int i = 0; i < 64; ++i) {
			const int idx = GbShadowCpu::ramIndex(static_cast<uint16_t>(e.regs.sp + i), e.wramBank);
			if (idx >= 0) {
				m_cpu.setRamByte(idx, e.image->ram[idx]);
			}
		}
		m_inRequest = true;
		m_requestWrites.clear();
		const GbShadowCpu::Exit ex = m_cpu.call(e.entry, e.romBank, e.regs, kRequestBudget, GbShadowCpu::Mode::Request);
		m_inRequest = false;
		if (ex != GbShadowCpu::Exit::Returned) {
			m_cpu = cpuBefore;
			m_apu = apuBefore;
			m_link->reportBadRequest(e.requestKey);
		} else {
			for (const auto& w : m_requestWrites) {
				nativeWrite(w.first, w.second);
			}
		}
		m_requestWrites.clear();
		if (m_link->log && !m_recording) {
			char buf[128];
			std::snprintf(buf, sizeof(buf), "shadow call %02X:%04X a=%02X de=%02X%02X exit %d inputs %02X", e.romBank, e.entry,
			              e.regs.a, e.regs.d, e.regs.e, static_cast<int>(ex), m_cpu.inputRegs());
			m_link->log(buf);
		}
		break;
	}
	}
}

// 원본이 갱신 루틴을 실제 시간보다 덜(더) 부른 만큼 다음 박자를 늦춘다(당긴다). 다른 이벤트는 박자 뒤에 넣으려고 모아 둔다
void GbDriverShadow::pollPace(double interval) {
	std::deque<std::shared_ptr<const GbDriverEvent>> events;
	{
		std::lock_guard<std::mutex> lock(m_queue->mutex);
		events.swap(m_queue->events);
	}
	for (auto& e : events) {
		if (e->type == GbDriverEvent::Pace) {
			if (m_ready) {
				m_frameCycles = std::max(-4 * interval, m_frameCycles + e->pace * interval);
			}
		} else {
			m_pending.push_back(std::move(e));
		}
	}
}

void GbDriverShadow::tick(double interval) {
	// 원본과 같은 순서: 이 박자의 갱신 루틴을 먼저 돌리고, 그 사이 원본에서 온 요청·쓰기는 그 뒤에 넣는다
	// (원본도 프레임 안에서 받은 요청을 다음 갱신 루틴에서 반영한다)
	pollPace(interval);
	std::deque<std::shared_ptr<const GbDriverEvent>> events;
	events.swap(m_pending);
	// 원본이 마지막으로 갱신 루틴을 부른 프레임에서 부른 곳들 (Mark). 그 앞의 요청·쓰기는 갱신 전에 넣는다
	// (갱신이 없는 프레임에 받은 요청 등).
	// 원본이 한 프레임 안에서 갱신 루틴을 여러 번 불렀으면 (요청을 바로 반영하려고 게임 코드가 직접 부르는 등)
	// 그 사이의 쓰기와 갱신을 원본과 같은 순서로 그만큼 돌린다
	std::vector<size_t> marks;
	for (size_t i = 0; i < events.size(); ++i) {
		if (events[i]->type == GbDriverEvent::Mark) {
			if (!marks.empty() && events[marks.back()]->index != events[i]->index) {
				marks.clear();  // 앞 프레임의 것: 마지막 프레임 것만 따로 돌린다
			}
			marks.push_back(i);
		}
	}
	size_t next = 0;
	if (marks.empty()) {
		runUpdate();
	} else {
		for (size_t m : marks) {
			for (; next <= m; ++next) {
				apply(*events[next]);
			}
			runUpdate();
		}
		// 더 돌린 횟수는 원본이 실제 시간보다 더 부른 것이라 Pace 가 그만큼 박자를 당긴다: 여기서 돌렸으므로 되돌린다
		m_frameCycles += static_cast<double>(marks.size() - 1) * interval;
	}
	for (; next < events.size(); ++next) {
		apply(*events[next]);
	}
}

void GbDriverShadow::runUpdate() {
	m_apu.setMutedChannels(m_mutedMask);
	const float gain = m_gain;
	for (int i = 0; i < 4; ++i) {
		m_apu.setChannelGain(i, gain);
	}
	if (m_native) {
		m_native->setMutedChannels(m_mutedMask);
		m_native->setGain(gain);
	}
	if (!m_ready) {
		return;
	}
	const GbDriverLink::Routine u = m_link->update();
	if (!u.valid) {
		return;
	}
	std::vector<GbDriverLink::Routine> list = m_link->routines();
	if (list.empty()) {
		list.push_back(u);
	}
	auto rkey = [](const GbDriverLink::Routine& r) { return (static_cast<uint32_t>(r.entry < 0x4000 ? 0 : r.romBank) << 16) | r.entry; };
	for (const GbDriverLink::Routine& r0 : list) {
		const GbDriverLink::Routine& r = rkey(r0) == rkey(u) ? u : r0;
		// 게임이 정하는 값(설정 등)도 원본의 쓰기로 넘겨받으므로 따로 가져오지 않는다
		m_cpu.setWramBank(r.wramBank);
		const GbShadowCpu::Exit ex = m_cpu.call(r.entry, r.romBank, r.regs, kUpdateBudget, GbShadowCpu::Mode::Update);
		if (ex != GbShadowCpu::Exit::Returned && m_link->log) {
			char buf[96];
			std::snprintf(buf, sizeof(buf), "shadow update exit %d at pc %04X", static_cast<int>(ex), m_cpu.regs().pc);
			m_link->log(buf);
		}
	}
	m_link->map.updates++;
}

void GbDriverShadow::render(int frames, int sampleRate, float* out) {
	const double perSample = kClock / sampleRate;
	const double fixed = m_link->intervalCycles.load();
	const double interval = fixed > 0 ? fixed : kFrameCycles / std::max(0.25, m_link->callsPerFrame.load());
	for (int i = 0; i < frames; ++i) {
		double need = perSample;
		double accL = 0, accR = 0;
		while (need > 0) {
			if (m_frameCycles <= 0) {
				pollPace(interval);
				// 원본이 갱신 루틴을 더 많이 불렀으면 (Pace 가 음수) 박자가 여러 개 밀려 있다: 모두 따라잡는다
				while (m_frameCycles <= 0) {
					tick(interval);
					m_frameCycles += interval;
				}
			}
			const double step = std::min(need, m_frameCycles);
			float l, r;
			m_apu.run(step, l, r);
			if (m_native) {
				m_native->run(step, l, r);
			}
			accL += l * step;
			accR += r * step;
			need -= step;
			m_frameCycles -= step;
		}
		out[i * 2] = static_cast<float>(accL / perSample);
		out[i * 2 + 1] = static_cast<float>(accR / perSample);
	}
	// 직류 제거 (GscPlayer 와 같음)
	const float k = static_cast<float>(std::pow(65368.0 / 65536.0, 131072.0 / sampleRate));
	for (int i = 0; i < frames; ++i) {
		const float l = out[i * 2] - m_capL;
		m_capL = out[i * 2] - l * k;
		out[i * 2] = l;
		const float r = out[i * 2 + 1] - m_capR;
		m_capR = out[i * 2 + 1] - r * k;
		out[i * 2 + 1] = r;
	}
}

void GbDriverShadow::mixInto(int16_t* stereo, int frames, int sampleRate, int volume) {
	if (frames <= 0 || sampleRate <= 0) {
		return;
	}
	std::vector<float> buf(static_cast<size_t>(frames) * 2);
	render(frames, sampleRate, buf.data());
	for (int i = 0; i < frames * 2; ++i) {
		const int v = stereo[i] + static_cast<int>(buf[i] * volume / 0x100);
		stereo[i] = static_cast<int16_t>(std::max(-32768, std::min(32767, v)));
	}
}

void GbDriverShadow::runFrame() {
	const double perFrame = kFrameCycles * kRecordRate / kClock;
	m_recCarry += perFrame;
	const int n = static_cast<int>(m_recCarry);
	m_recCarry -= n;
	std::vector<float> buf(static_cast<size_t>(n) * 2);
	render(n, kRecordRate, buf.data());
	for (int i = 0; i < n * 2; ++i) {
		m_streamFifo.push_back(static_cast<int16_t>(std::max(-32768.f, std::min(32767.f, buf[i]))));
	}
	while (m_streamFifo.size() > 2 * 32768) {
		m_streamFifo.pop_front();
	}
}

bool GbDriverShadow::popStreamSample(int16_t& left, int16_t& right) {
	if (m_streamFifo.size() < 2) {
		return false;
	}
	left = m_streamFifo.front();
	m_streamFifo.pop_front();
	right = m_streamFifo.front();
	m_streamFifo.pop_front();
	return true;
}

}
