/* ff-bgm1x: 엔진에 상관없는 GB/GBC BGM·효과음 1배속.
 * 게임 자신의 사운드 드라이버 코드를 자체 CPU(GbShadowCpu)로 실제 시간 기준으로 돌리고, 자체 합성기(GbApuSynth)로 낸다.
 * 앱이 자기 코어의 GB 사운드 장치를 하나 더 만들어 주면(GbNativeApu) 소리는 그쪽으로 낸다 (원본과 같은 합성).
 *
 * 원본 코어 쪽 (GbDriverTracer, 에뮬레이션 스레드): 원본 CPU 의 명령·인터럽트·메모리 쓰기를 받아 호출 스택을 따라간다.
 *   - 갱신 루틴: 인터럽트 안에서 사운드 레지스터(FF10~FF3F)를 쓰는 코드를 모두 품는 가장 깊은 함수.
 *   - 드라이버 RAM: 그림자 CPU 가 갱신 루틴을 돌리며 쓰기 전에 읽는 RAM (GbDriverRamMap).
 *   - 요청: 갱신 루틴 밖의 코드가 드라이버 RAM 에 쓰는 것. 그 쓰기를 품고 게임 RAM 은 건드리지 않는 가장 바깥 함수를
 *     요청 함수로 배우고(처음 한 번은 쓴 값을 그대로 넘긴다), 다음부터는 그 함수의 호출(레지스터·롬 bank·RAM 사본)을 넘긴다.
 * 그림자 쪽 (GbDriverShadow, 오디오 스레드 또는 녹화): 넘어온 호출을 같은 함수로 부르고, 갱신 루틴을 실제 시간 박자로 부른다. */
#pragma once

#include <algorithm>
#include <array>
#include <atomic>
#include <cstdint>
#include <cstdio>
#include <deque>
#include <functional>
#include <memory>
#include <mutex>
#include <string>
#include <unordered_map>
#include <unordered_set>
#include <vector>

#include "wx/ffbgm/gb_shadow_cpu.h"
#include "wx/ffbgm/gsc_player.h"

namespace QGBA {

struct GbDriverEvent {
	enum Type { Init, Call, Write, WriteBatch, Pace, Mark, SoundWrite } type = Write;  // Mark: 원본이 갱신 루틴을 부른 곳, SoundWrite: 주 흐름의 사운드 레지스터 쓰기 (index = FF00 기준 하위 바이트)
	// Pace: 원본이 실제 시간에 맞춰 불렀어야 할 갱신 루틴 횟수 - 실제로 부른 횟수 (소수). 그림자는 그만큼 다음 박자를 늦춘다 (음수면 당긴다)
	double pace = 0;
	// Init / Call
	uint16_t entry = 0;
	int romBank = 1;
	int wramBank = 1;
	GbShadowCpu::Regs regs;
	std::shared_ptr<const GbRamImage> image;
	std::array<uint8_t, 0x30> sound{};  // Init: FF10~FF3F
	std::array<uint8_t, 0x80> io{};     // Init: FF00~FF7F
	// Write
	int index = 0;
	uint8_t value = 0;
	// WriteBatch: (RAM 위치, 값)
	std::vector<std::pair<int, uint8_t>> batch;
	// Call: 소리 분류용 (요청 함수 번호, 입력 값)
	uint32_t requestKey = 0;
};

// 원본(추적기)과 그림자(재생기)가 함께 쓰는 것
class GbDriverLink {
public:
	GbDriverRamMap map;

	struct Queue {
		std::mutex mutex;
		std::deque<std::shared_ptr<const GbDriverEvent>> events;
	};
	std::shared_ptr<Queue> subscribe();
	std::atomic<bool> initRequested{false};  // 새 그림자가 구독했다 (원본 상태를 다시 보내야 한다)
	void unsubscribe(const std::shared_ptr<Queue>& q);
	void post(const std::shared_ptr<const GbDriverEvent>& e);

	struct Routine {
		bool valid = false;
		uint16_t entry = 0;
		int romBank = 1;
		int wramBank = 1;
		GbShadowCpu::Regs regs;
	};
	void setUpdate(const Routine& r);
	// 갱신 루틴이 여럿일 때 (화면 인터럽트가 음량 조절 함수와 음악 엔진을 따로 부르는 게임 등): 원본이 부르는 순서대로.
	// 비어 있으면 update() 하나만 부른다
	void setRoutines(const std::vector<Routine>& r);
	std::vector<Routine> routines() const;
	// 그림자에서 정상으로 돌아오지 않은 요청 함수 (추적기가 다시 배우지 않는다)
	void reportBadRequest(uint32_t key);
	bool badRequest(uint32_t key) const;
	Routine update() const;
	std::atomic<double> callsPerFrame{1.0};
	// 갱신 루틴을 부르는 간격 (4.19MHz 사이클). 0 이면 callsPerFrame 으로 정한다 (타이머 인터럽트로 부르는 드라이버는 타이머 설정으로 정확히 안다)
	std::atomic<double> intervalCycles{0.0};
	// 확인용 기록 (없으면 남기지 않는다)
	std::function<void(const std::string&)> log;

	// 원본 RAM 의 마지막 사본 (갱신 루틴이 읽기만 하는 값을 가져온다)
	void setLatest(std::shared_ptr<const GbRamImage> image);
	std::shared_ptr<const GbRamImage> latest() const;

private:
	mutable std::mutex m_mutex;
	std::vector<std::shared_ptr<Queue>> m_queues;
	Routine m_update;
	std::vector<Routine> m_routines;
	std::shared_ptr<const GbRamImage> m_latest;
	std::unordered_set<uint32_t> m_badRequests;
};

class GbDriverTracer {
public:
	explicit GbDriverTracer(std::shared_ptr<GbDriverLink> link);
	// 갱신 루틴을 찾은 순간 원본 RAM 사본으로 미리 돌려 드라이버 RAM 을 알아 두는 데 쓴다
	void setRom(std::shared_ptr<const std::vector<uint8_t>> rom) { m_rom = std::move(rom); }

	// 원본 CPU 에서 (에뮬레이션 스레드)
	void fetch(const GbShadowCpu::Regs& r, uint8_t opcode, int romBank, int wramBank);  // r.pc = 명령 주소
	void irq(const GbShadowCpu::Regs& r, uint16_t returnPc, int romBank, int wramBank);  // r.pc = 인터럽트 벡터, r.sp = 넣은 뒤
	void store(uint16_t addr, uint8_t value, uint16_t sp, int wramBank);
	// 프레임 끝: image = 지금 원본 RAM, sound/io = 지금 IO 레지스터
	// frameCycles: 이번 프레임의 실제 길이 (4.19MHz 사이클)
	void frameEnd(std::shared_ptr<const GbRamImage> image, const std::array<uint8_t, 0x80>& io, int romBank, int wramBank,
	              double frameCycles = 70224.0);
	// 리셋·상태 불러오기: 호출 스택을 버리고 그림자를 원본 상태로 다시 맞춘다
	void resync();

	bool updateKnown() const { return m_update.valid; }
	const GbDriverLink::Routine& updateRoutine() const { return m_update; }
	size_t learnedCount() const { return m_learned.size(); }
	std::vector<uint32_t> learned() const { return std::vector<uint32_t>(m_learned.begin(), m_learned.end()); }
	// 확인용 기록 (없으면 남기지 않는다)
	std::function<void(const std::string&)> log;

	// 요청 호출에 RAM 사본을 붙이기 위해 원본 RAM 을 읽는다 (호출 쪽이 정한다)
	std::function<void(GbRamImage&)> readRam;
	// 확인용 수: 넘긴 값 쓰기, 넘긴 호출, 버스트
	uint32_t statWrites = 0, statCalls = 0, statBursts = 0, statFrames = 0;
	bool traceWrites = false;
	uint32_t traceFrom = 0, traceTo = 0;

private:
	struct Frame {
		uint16_t entry = 0;
		int romBank = 0;
		int wramBank = 1;
		uint16_t sp = 0;
		bool isr = false;
		GbShadowCpu::Regs regs;
		uint32_t impureMark = 0;
		uint32_t hardMark = 0;
		uint32_t trapMark = 0;  // 들어올 때의 Level::traps (이 함수 안에서 배운 요청 함수를 불렀는지)
		// 꼬리 호출: 이 함수가 멀리 jp 해서 넘겨준 곳 (뱅크를 바꾸고 다른 함수로 뛰어 들어가는 코드)
		uint16_t tailEntry = 0;
		int tailBank = 0;
		GbShadowCpu::Regs tailRegs;
		uint32_t tailSerial = 0;
	};
	struct Burst {
		int lcaDepth = -1;  // 드라이버 RAM 쓰기를 모두 품는 함수 (호출 스택 위치)
		int bound = -1;     // 이 위치까지의 함수는 버스트 시작 뒤 그대로 살아 있다
		bool closed = false;
		bool lcaPure = false;
		Frame lca;
	};
	struct Level {  // 주 흐름 또는 인터럽트 하나
		int base = 0;        // 이 단계 첫 함수의 스택 위치
		uint16_t vector = 0; // 인터럽트 주소 (0x40 화면, 0x48 LCD, 0x50 타이머 ...). 주 흐름은 0
		int trapDepth = -1;  // 배운 요청 함수 호출 중이면 그 위치
		uint32_t impure = 0; // 게임 RAM·화면을 건드린 횟수
		uint32_t hard = 0;   // 그 가운데 화면·저장 RAM·사운드 밖 I/O 를 건드린 횟수
		uint32_t traps = 0;  // 배운 요청 함수를 그림자에 넘긴 횟수
		uint32_t savedImpure = 0;
		Burst burst;
		// 인터럽트: 사운드 레지스터 쓰기를 품는 함수 (갱신 루틴 찾기)
		int sndDepth = -1, sndBound = -1;
		Frame snd;
		bool sndAny = false;
		// 그 함수가 꼬리 호출로 넘겨준 곳에서만 사운드를 썼으면, 갱신 루틴은 넘겨받은 곳이다
		uint32_t sndTailSerial = 0;
		bool sndTailOk = false;
		// 인터럽트 첫 함수부터 그 함수까지의 호출 경로와, 경로 위 함수마다 화면·저장 RAM·사운드 밖 I/O 를 건드리지 않았는지 (1 = 안 건드림)
		std::vector<Frame> sndPath;
		std::vector<uint8_t> sndPure;
		std::vector<uint8_t> sndTrap;  // sndPath 의 함수가 배운 요청 함수를 불렀는지
		// 사운드를 쓴 코드마다의 호출 경로 (갈래가 여럿이면 갈래마다 갱신 루틴을 하나씩 찾는다)
		struct SndPath {
			std::vector<Frame> frames;
			std::vector<uint8_t> pure;
		};
		std::vector<SndPath> sndPaths;
	};
	// 인터럽트 하나에서 사운드를 쓴 갈래마다, 게임 쪽 일을 하지 않는 가장 바깥 함수 (부른 순서대로, 겹치지 않게)
	std::vector<Frame> soundRoutines(const Level& lv) const;
	// 확정한 갱신 루틴 목록 (첫째가 m_update). 원본에서 새 갈래가 보이면 그 순서에 맞춰 끼워 넣는다
	std::vector<Frame> m_routines;
	void publishRoutines();
	void addRoutines(const std::vector<Frame>& list);
	bool inRoutines(uint32_t k) const;
	uint32_t m_listKey = 0;
	int m_listVotes = 0;
	std::unordered_map<uint32_t, int> m_addVotes;
	std::unordered_map<uint32_t, int> m_mainSndVotes;
	std::unordered_set<uint32_t> m_mainSnd;  // 배운 주 흐름 사운드 함수 (게임 RAM 을 건드려도 잊지 않는다)  // 갱신 루틴을 찾은 뒤 주 흐름에서 사운드를 쓴 함수가 나온 프레임 수  // 확정 뒤 새로 보인 갱신 루틴 후보가 나온 인터럽트 수
	// 두 경로가 처음부터 같은 함수를 지나는 길이
	static size_t commonPath(const std::vector<Frame>& a, const std::vector<Frame>& b) {
		size_t i = 0;
		while (i < a.size() && i < b.size() && key(a[i].entry, a[i].romBank) == key(b[i].entry, b[i].romBank)) {
			++i;
		}
		return i;
	}
	// a 를 b 와 같은 앞부분 p 개로 자르고, 함수마다 두 번 다 안 건드렸을 때만 안 건드림으로 둔다
	static void mergePath(std::vector<Frame>& a, std::vector<uint8_t>& aPure, const std::vector<Frame>& b, const std::vector<uint8_t>& bPure, size_t p) {
		a.resize(p);
		aPure.resize(p);
		for (size_t i = 0; i < p; ++i) {
			aPure[i] = aPure[i] && i < bPure.size() && bPure[i];
		}
	}
	// 둘 다 뱅크 0 영역이거나 둘 다 같은 바꿀 수 있는 뱅크
	static bool sameArea(const Frame& a, const Frame& b) {
		if (a.entry < 0x4000 || b.entry < 0x4000) {
			return a.entry < 0x4000 && b.entry < 0x4000;
		}
		return a.romBank == b.romBank;
	}
	// parent 가 child 를 부른 함수로 갱신 루틴을 넓혀도 되는지: 같은 영역이면 된다.
	// 뱅크 0 함수가 다른 뱅크의 함수를 불렀으면, 그 함수 코드 안에 child 를 직접 부르는 명령(call/jp 주소)이 있을 때만 된다
	// (레지스터로 받은 주소로 뛰는 뱅크 바꾸기 중계는 부를 때마다 하는 일이 다르다)
	bool canWiden(const Frame& parent, const Frame& child) const;
	// 갱신 루틴: 경로 끝(사운드를 쓴 코드를 모두 품은 함수)에서, 화면·저장 RAM·사운드 밖 I/O 를 건드리지 않은 함수가 부른 동안 위로 넓힌다.
	// 인터럽트 첫 함수로는 넓히지 않는다
	int widenIndex(const std::vector<Frame>& path, const std::vector<uint8_t>& pure) const {
		int i = static_cast<int>(path.size()) - 1;
		while (i - 1 >= 1 && pure[i - 1] && canWiden(path[i - 1], path[i])) {
			--i;
		}
		return std::max(i, 0);
	}
	// from 에서 to 까지 (to < from) 위로 넓힐 수 있는지
	bool canWidenChain(const std::vector<Frame>& path, int from, int to) const {
		for (int i = from; i > to; --i) {
			if (!canWiden(path[i - 1], path[i])) {
				return false;
			}
		}
		return true;
	}
	static uint32_t key(uint16_t entry, int bank) {
		return (static_cast<uint32_t>(entry < 0x4000 ? 0 : bank) << 16) | entry;
	}
	void push(const Frame& f);
	uint16_t resolveEntry(const Frame& f) const;
	void popTop();
	void finishBurst(Level& lv);
	void postMark();
	void impure(uint16_t addr = 0) {
		m_levels.back().impure++;
		if (traceWrites && log && (m_levels.back().burst.lcaDepth >= 0 || (statFrames >= traceFrom && statFrames < traceTo))) {
			char buf[64];
			std::snprintf(buf, sizeof(buf), "f%u impure %04X pc %04X sp %04X", statFrames, addr, m_lastPc, m_lastSp);
			log(buf);
		}
	}
	bool inUpdate() const { return m_updateDepth >= 0; }
	void postInit();
	void queueWrite(int idx, uint8_t value);
	void flushWrites();
	std::vector<uint8_t> m_pendValue = std::vector<uint8_t>(kGbRamSize, 0);
	std::vector<uint8_t> m_pendMark = std::vector<uint8_t>(kGbRamSize, 0);
	std::vector<int> m_pendList;

	std::shared_ptr<GbDriverLink> m_link;
	std::shared_ptr<const std::vector<uint8_t>> m_rom;
	bool m_warmed = false;
	void warmUp();
	std::vector<Frame> m_frames;
	std::vector<Level> m_levels;
	bool m_pendingCall = false;
	bool m_pendingJump = false;
	bool m_spLoaded = false;     // 바로 앞 명령이 SP 를 직접 바꿨다
	bool m_foreignStack = false; // 다른 스택을 쓰는 중
	uint16_t m_savedSp = 0;      // 옮기기 전 SP
	uint32_t m_foreignSteps = 0;
	void dropStack();
	uint16_t m_jumpNext = 0;     // jp 를 하지 않았다면 다음 명령 주소
	uint32_t m_tailCounter = 0;
	void noteSoundWrite(Level& lv);
	uint16_t m_lastSp = 0, m_lastPc = 0;
	uint16_t m_pendingSp = 0;
	int m_updateDepth = -1;

	GbDriverLink::Routine m_update;
	std::vector<Frame> m_candPath;  // 확정 전 후보 경로 (여러 인터럽트가 함께 지난 앞부분)
	std::vector<uint8_t> m_candPure;
	int m_votes = 0;
	std::vector<Frame> m_updPath;   // 확정한 갱신 루틴의 경로 (m_updIndex 가 갱신 루틴)
	std::vector<uint8_t> m_updPure;
	int m_updIndex = 0;
	// 주 흐름에서 드라이버를 부르는 게임: 프레임마다 주 흐름의 사운드 쓰기를 품는 함수를 세어 고른다
	struct MainCand {
		Frame frame;
		int count = 0;
	};
	std::unordered_map<uint32_t, MainCand> m_mainCands;
	int m_mainTotal = 0;
	void mainSoundEnd();
	std::vector<Frame> m_challPath;  // 확정한 경로와 전혀 다른 후보
	std::vector<uint8_t> m_challPure;
	int m_challengeVotes = 0;
	bool m_updIdle = false;  // 확정(또는 교체)한 뒤 갱신 루틴이 아직 사운드를 한 번도 쓰지 않음
	bool m_needInit = false;
	std::unordered_set<uint32_t> m_learned;
	std::unordered_map<uint32_t, int> m_bad;  // 배운 함수가 게임 RAM 을 건드린 횟수
	uint64_t m_updateCalls = 0, m_frameCount = 0;
	int m_frameCalls = 0;  // 이번 프레임에 원본이 갱신 루틴을 부른 횟수
	double m_paceCarry = 0; // 원본이 실제 시간에 맞춰 불렀어야 할 횟수 - 실제로 부른 횟수 (넘기지 않은 소수 부분)
	uint16_t m_updVector = 0;      // 갱신 루틴을 부른 인터럽트 주소 (주 흐름이면 0)
	double m_timerInterval = 0;     // 마지막으로 켜져 있던 타이머의 간격
	double m_rate = 1.0;
	std::shared_ptr<const GbRamImage> m_lastImage;
	std::array<uint8_t, 0x80> m_lastIo{};
	int m_lastRomBank = 1, m_lastWramBank = 1;
	uint32_t m_storeCount = 0;
};

// 앱 코어의 GB 사운드 장치 (원본 코어와 따로 하나 더 만든 것). 그림자의 사운드 레지스터 쓰기를 받아 소리를 낸다
class GbNativeApu {
public:
	virtual ~GbNativeApu() = default;
	virtual void write(uint8_t reg, uint8_t value) = 0;  // reg = FF00 기준 하위 바이트 (0x10~0x3F)
	// cycles(4.19MHz 기준)만큼 진행하며 그 동안의 평균 출력을 L/R 로 (직류 제거 전, int16 범위의 실수)
	virtual void run(double cycles, float& left, float& right) = 0;
	virtual void setMutedChannels(uint8_t mask) = 0;
	virtual void setGain(float gain) = 0;
	// 길이·엔벨로프 박자(512Hz)의 다음 박까지 남은 사이클 (원본 코어의 박자에 맞출 때)
	virtual void setFramePhase(int cycles) { (void) cycles; }
};

// 그림자: 게임 드라이버를 실제 시간으로 돌려 소리를 만든다
class GbDriverShadow {
public:
	// 앱이 정한다: 그림자마다 GB 사운드 장치를 하나씩 만든다 (비어 있거나 nullptr 를 돌려주면 자체 합성기)
	static std::function<std::unique_ptr<GbNativeApu>()> s_nativeApu;

	GbDriverShadow(std::shared_ptr<const std::vector<uint8_t>> rom, std::shared_ptr<GbDriverLink> link, bool recording);
	~GbDriverShadow();

	void setMutedChannels(uint8_t mask) { m_mutedMask = mask; }
	void setGain(float gain) { m_gain = gain; }
	// 실시간용: sampleRate 로 frames 개를 만들어 stereo 에 더한다 (volume 0x100 = 그대로)
	void mixInto(int16_t* stereo, int frames, int sampleRate, int volume);
	// 녹화용: 게임 프레임 하나만큼
	void runFrame();
	bool popStreamSample(int16_t& left, int16_t& right);
	bool ready() const { return m_ready; }

private:
	void tick(double interval);
	void runUpdate();
	void apply(const GbDriverEvent& e);
	void render(int frames, int sampleRate, float* out);

	std::shared_ptr<GbDriverLink> m_link;
	std::shared_ptr<GbDriverLink::Queue> m_queue;
	bool m_recording;
	GbApuSynth m_apu;  // 레지스터 읽기(채널 켜짐 등)용. 앱 장치가 없으면 소리도 여기서 낸다
	std::unique_ptr<GbNativeApu> m_native;
	bool m_inRequest = false;
	std::vector<std::pair<uint8_t, uint8_t>> m_requestWrites;  // 요청 함수 실행 중의 사운드 쓰기 (정상 반환 때 앱 장치에 넣는다)
	void nativeWrite(uint8_t reg, uint8_t value);
	GbShadowCpu m_cpu;
	bool m_ready = false;
	std::deque<std::shared_ptr<const GbDriverEvent>> m_pending;  // 다음 박자 뒤에 넣을 이벤트
	void pollPace(double interval);
	std::atomic<uint8_t> m_mutedMask{0};
	std::atomic<float> m_gain{1.0f};
	double m_frameCycles = 0;
	std::deque<int16_t> m_streamFifo;
	float m_capL = 0, m_capR = 0;
	double m_recCarry = 0;
};

}
