/*  
 *  4chSampler.ino - 4 channel samplerapplication
 *  Author Interested-In-Spresense
 *
 *  Based on Spresense.ino from
 *  https://github.com/Interested-In-Spresense/Spresense-Playground/tree/master/IntegrationWithProcessing/LiveCamera/viaUSBSerial/YUV/Spresense
 *
 *  This library is free software; you can redistribute it and/or
 *  modify it under the terms of the GNU Lesser General Public
 *  License as published by the Free Software Foundation; either
 *  version 2.1 of the License, or (at your option) any later version.
 *
 *  This library is distributed in the hope that it will be useful,
 *  but WITHOUT ANY WARRANTY; without even the implied warranty of
 *  MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the GNU
 *  Lesser General Public License for more details.
 *
 *  You should have received a copy of the GNU Lesser General Public
 *  License along with this library; if not, write to the Free Software
 *  Foundation, Inc., 51 Franklin St, Fifth Floor, Boston, MA  02110-1301  USA
 */

#include <SDHCI.h>
#include <OutputMixer.h>
#include <MemoryUtil.h>
#include <stdio.h>
#include <stdlib.h>

#include <arch/board/board.h>

#define READSAMPLE  (240)
#define BYTEWIDTH   (2)
#define CHNUM       (2)
#define READSIZE    (READSAMPLE * BYTEWIDTH * CHNUM)
#define RREREQUEST  (3)

#define RING_FRAMES   (30)
#define RING_SAMPLES  (RING_FRAMES * READSAMPLE)
#define BUF_SAMPLES   (RING_SAMPLES * CHNUM)
#define BUF_BYTES     (BUF_SAMPLES * BYTEWIDTH)

/* [LOG] debug print switch (0:off, 1:on) */
#define BB_DEBUG_LOG (0)

SDClass theSD;
OutputMixer *theMixer;

/* ---- 2 Mixers × 2ch = 4 slots ----
   slot: 0 = mixer0 ch0, 1 = mixer0 ch1, 2 = mixer1 ch0, 3 = mixer1 ch1
*/
#define MIXERS     (2)
#define SLOTS      (MIXERS * CHNUM)

static File     g_file[SLOTS];
static bool     g_active[SLOTS] = { false, false, false, false };

static uint32_t g_tick = 1;
static uint32_t g_last_tick[SLOTS] = { 0, 0, 0, 0 };

/* Shared read buffer (single-thread loop usage) */
static uint8_t g_mono[READSAMPLE * 2]; // 480 bytes

/* Slot mapping helpers */
static inline int slot_to_mixer(int slot) { return slot / CHNUM; }
static inline int slot_to_ch(int slot)    { return slot % CHNUM; }

/* ---------------- BridgeBuffer (2ch interleaved, per-mixer instance) ---------------- */
class BridgeBuffer {
private:
  // Interleaved stereo: [L0,R0,L1,R1,...]
  int16_t  m_buf[BUF_SAMPLES];

  // Read pointer: mono-sample index of next frame start (0..RING_SAMPLES-1)
  uint32_t m_rp;

  // Used mono samples ahead of rp for each channel: 0..RING_SAMPLES
  uint32_t m_used[CHNUM];

  // channel attach (write enable)
  bool     m_attached[CHNUM];

  static inline uint32_t byte_off_from_rp(uint32_t rp_mono_sample) {
    return (rp_mono_sample % RING_SAMPLES) * (CHNUM * BYTEWIDTH);
  }

  void memcpy_frame_out(uint8_t* dst) const {
    const uint8_t* src = reinterpret_cast<const uint8_t*>(m_buf);
    uint32_t off = byte_off_from_rp(m_rp);

    if (off + READSIZE <= BUF_BYTES) {
      memcpy(dst, src + off, READSIZE);
    } else {
      uint32_t first = BUF_BYTES - off;
      memcpy(dst, src + off, first);
      memcpy(dst + first, src, READSIZE - first);
    }
  }

  void memset_frame_zero() {
    uint8_t* p = reinterpret_cast<uint8_t*>(m_buf);
    uint32_t off = byte_off_from_rp(m_rp);

    if (off + READSIZE <= BUF_BYTES) {
      memset(p + off, 0, READSIZE);
    } else {
      uint32_t first = BUF_BYTES - off;
      memset(p + off, 0, first);
      memset(p, 0, READSIZE - first);
    }
  }

public:
  BridgeBuffer() { clearbuf(); }
  ~BridgeBuffer() {}

  void clearbuf(void)
  {
    memset(m_buf, 0, sizeof(m_buf));
    m_rp = 0;
    for (int ch = 0; ch < CHNUM; ch++) {
      m_used[ch] = 0;
      m_attached[ch] = false;
    }
#if BB_DEBUG_LOG
    printf("BUF_BYTES=%u, RING_SAMPLES=%u, READSIZE=%u\n",
           (unsigned)BUF_BYTES, (unsigned)RING_SAMPLES, (unsigned)READSIZE);
#endif
  }

  void attach(int ch)
  {
    if (ch < 0 || ch >= CHNUM) return;
    m_attached[ch] = true;
    m_used[ch] = 0; // start from current rp
  }

  void detach(int ch)
  {
    if (ch < 0 || ch >= CHNUM) return;

    // Policy: zero only "unconsumed" part (minimum necessary).
    uint32_t need = m_used[ch];

    if (m_attached[ch] && need > 0) {
      uint32_t pos = m_rp;
      for (uint32_t i = 0; i < need; i++) {
        uint32_t s = pos + i;
        if (s >= RING_SAMPLES) s -= RING_SAMPLES;
        m_buf[s * 2 + ch] = 0;
      }
    }

    m_attached[ch] = false;
    m_used[ch] = 0;
  }

  uint32_t writable_samples(int ch) const
  {
    if (ch < 0 || ch >= CHNUM) return 0;
    if (!m_attached[ch]) return 0;
    return RING_SAMPLES - m_used[ch];
  }

  int32_t write_samples(int ch, const int16_t* src, uint32_t req_samples)
  {
    if (!src || req_samples == 0) return 0;
    if (ch < 0 || ch >= CHNUM) return -1;
    if (!m_attached[ch]) return -1;

    uint32_t free = writable_samples(ch);
    if (free < req_samples) return -1; // reject

    uint32_t wp = (m_rp + m_used[ch]) % RING_SAMPLES;

    for (uint32_t i = 0; i < req_samples; i++) {
      uint32_t s = wp + i;
      if (s >= RING_SAMPLES) s -= RING_SAMPLES;
      m_buf[s * 2 + ch] = src[i];
    }

    m_used[ch] += req_samples;
    return (int32_t)req_samples;
  }

  int32_t readbuf(uint8_t *dst)
  {
    if (!dst) return -1;

    // If no channel has data, return 0 (keep app behavior)
    if (m_used[0] == 0 && m_used[1] == 0) {
      return 0;
    }

    // Output current frame (buffer guarantees zeros)
    memcpy_frame_out(dst);

    // Clear the frame after read so future unwritten parts stay 0
    memset_frame_zero();

    // Advance rp by one frame (READSAMPLE mono samples)
    m_rp = (m_rp + READSAMPLE) % RING_SAMPLES;

    // Consume used samples per channel independently
    for (int ch = 0; ch < CHNUM; ch++) {
      if (!m_attached[ch]) {
        m_used[ch] = 0;
        continue;
      }
      if (m_used[ch] >= READSAMPLE) m_used[ch] -= READSAMPLE;
      else m_used[ch] = 0;
    }

    return (int32_t)READSIZE;
  }
};

/* One buffer per Mixer */
static BridgeBuffer g_buf[MIXERS];

/* ---------------- App state ---------------- */
bool ErrEnd = false;

static enum State {
  Ready = 0,
  Active,
  Stopping,
} s_state = Ready;


/* ---------------- getFrame common  ---------------- */
static bool getFrame(AsPcmDataParam *pcm,
                            uint8_t mix,
                            MemMgrLite::PoolId pool,
                            int32_t identifier)
{
  if (pcm->mh.allocSeg(pool, READSIZE) != ERR_OK) {
    return false;
  }

  pcm->identifier = identifier;
  pcm->callback   = 0;
  pcm->bit_length = 16;

  pcm->size   = g_buf[mix].readbuf((uint8_t *)pcm->mh.getPa());
  pcm->is_end = false;
  pcm->is_valid = true;

  if (pcm->size < 0) {
    puts("Cannot read SD card!");
    return false;
  }

  if (pcm->size == 0) {
    pcm->size = READSIZE;
    memset(pcm->mh.getPa(), 0, READSIZE);
  }

  pcm->sample = pcm->size / BYTEWIDTH / CHNUM;
  return true;
}

static inline bool getFrame0(AsPcmDataParam *pcm, int32_t identifier)
{
  return getFrame(pcm, 0, S0_REND_PCM_BUF_POOL, identifier);
}

static inline bool getFrame1(AsPcmDataParam *pcm, int32_t identifier)
{
  return getFrame(pcm, 1, S0_REND_PCM_SUB_BUF_POOL, identifier);
}

/* ---------------- Slot IO ---------------- */
// push mono into slot's mixer/channel (EOF handled here)
static int32_t pushBridgeBuffer(int slot)
{
  int mix = slot_to_mixer(slot);
  int ch  = slot_to_ch(slot);

  if (!g_active[slot]) return 0;

  if (!g_file[slot]) {
    g_active[slot] = false;
    g_last_tick[slot] = 0;
    g_buf[mix].detach(ch);
    return 0;
  }

  if (g_buf[mix].writable_samples(ch) < READSAMPLE) {
    return -1; // no space now
  }

  int rd = g_file[slot].read(g_mono, sizeof(g_mono));
  if (rd < 0) return -1;

  if (rd < (int)sizeof(g_mono)) {
    g_file[slot].close();
    g_buf[mix].detach(ch);
    g_active[slot] = false;
    g_last_tick[slot] = 0;
    return 0; // EOF
  }

  const int16_t* in = (const int16_t*)g_mono;
  int32_t w = g_buf[mix].write_samples(ch, in, READSAMPLE);
  if (w > 0) {
    g_last_tick[slot] = g_tick++;
  }
  return w;
}

static void stop_slot(int slot)
{
  int mix = slot_to_mixer(slot);
  int ch  = slot_to_ch(slot);

  if (g_file[slot]) g_file[slot].close();
  g_active[slot] = false;
  g_last_tick[slot] = 0;
  g_buf[mix].detach(ch);
}

/* Allocation: Mixer0 free -> Mixer1 free -> steal global LRU */
static int find_free_slot_in_mixer(int mix)
{
  int base = mix * CHNUM;
  for (int i = 0; i < CHNUM; i++) {
    int slot = base + i;
    if (!g_active[slot]) return slot;
  }
  return -1;
}

static int choose_lru_slot_global(void)
{
  uint32_t best = 0xFFFFFFFFu;
  int best_slot = 0;

  for (int slot = 0; slot < SLOTS; slot++) {
    if (!g_active[slot]) continue;
    if (g_last_tick[slot] < best) {
      best = g_last_tick[slot];
      best_slot = slot;
    }
  }
  return best_slot;
}

static bool start_sound(uint8_t fileno)
{
  int slot = find_free_slot_in_mixer(0);
  if (slot < 0) slot = find_free_slot_in_mixer(1);

  if (slot < 0) {
    slot = choose_lru_slot_global();
    stop_slot(slot); // steal oldest
  }

  int mix = slot_to_mixer(slot);
  int ch  = slot_to_ch(slot);

  char fullpath[64] = { 0 };
  snprintf(fullpath, sizeof(fullpath), "AUDIO/sound%u.raw", (unsigned)fileno);

  g_file[slot] = theSD.open(fullpath);
  if (!g_file[slot]) {
    printf("File open error: %s\n", fullpath);
    return false;
  }

  g_buf[mix].attach(ch);
  g_active[slot] = true;
  g_last_tick[slot] = g_tick++;

  // preload Three frames for this slot
  for (int i = 0; i < RREREQUEST; i++) {
    if (pushBridgeBuffer(slot) <= 0) break;
  }

#if BB_DEBUG_LOG
  printf("start sound%u -> mixer%d ch%d (slot=%d)\n",
         (unsigned)fileno, mix, ch, slot);
#endif

  return true;
}

/* Start rendering if needed + start one sound */
static bool start(uint8_t fileno)
{
#if BB_DEBUG_LOG
  printf("start(%u)\n", (unsigned)fileno);
#endif

  if (s_state == Ready) {
    // fresh start
    for (int m = 0; m < MIXERS; m++) g_buf[m].clearbuf();
    for (int slot = 0; slot < SLOTS; slot++) {
      if (g_file[slot]) g_file[slot].close();
      g_active[slot] = false;
      g_last_tick[slot] = 0;
      int mix = slot_to_mixer(slot);
      int ch  = slot_to_ch(slot);
      g_buf[mix].detach(ch);
    }
  }

  if (!start_sound(fileno)) {
    return false;
  }

  // When first activation: prime both mixers' pipelines
  if (s_state == Ready) {
    for (int i = 0; i < RREREQUEST; i++) {
      AsPcmDataParam pcm;
      if (!getFrame0(&pcm, 0)) break;
      int err = theMixer->sendData(OutputMixer0, outmixer0_send_callback, pcm);
      if (err != OUTPUTMIXER_ECODE_OK) {
        printf("OutputMixer0 send error: %d\n", err);
        return false;
      }
    }

    for (int i = 0; i < RREREQUEST; i++) {
      AsPcmDataParam pcm;
      if (!getFrame1(&pcm, 1)) break;
      int err = theMixer->sendData(OutputMixer1, outmixer1_send_callback, pcm);
      if (err != OUTPUTMIXER_ECODE_OK) {
        printf("OutputMixer1 send error: %d\n", err);
        return false;
      }
    }

    s_state = Active;
  }

  return true;
}

/* Mixer stop only (both mixers) */
static void stop(void)
{
  printf("stop_all\n");

  // Send end to both mixers
  {
    AsPcmDataParam pcm;
    if (getFrame0(&pcm, 0)) {
      pcm.is_end = true;
      int err = theMixer->sendData(OutputMixer0, outmixer0_send_callback, pcm);
      if (err != OUTPUTMIXER_ECODE_OK) printf("OutputMixer0 send error: %d\n", err);
    }
  }
  {
    AsPcmDataParam pcm;
    if (getFrame1(&pcm, 1)) {
      pcm.is_end = true;
      int err = theMixer->sendData(OutputMixer1, outmixer1_send_callback, pcm);
      if (err != OUTPUTMIXER_ECODE_OK) printf("OutputMixer1 send error: %d\n", err);
    }
  }

  // Clear all
  for (int m = 0; m < MIXERS; m++) g_buf[m].clearbuf();
  for (int slot = 0; slot < SLOTS; slot++) {
    if (g_file[slot]) g_file[slot].close();
    g_active[slot] = false;
    g_last_tick[slot] = 0;
    int mix = slot_to_mixer(slot);
    int ch  = slot_to_ch(slot);
    g_buf[mix].detach(ch);
  }
}

/* ---------------- Mixer callbacks  ---------------- */
static void outmixer_send_callback(uint8_t mix,
                                   int dev,
                                   int32_t identifier,
                                   bool is_end)
{
  AsPcmDataParam pcm;

  while (true) {
    if (s_state == Stopping) break;

    bool ok = (mix == 0) ? getFrame0(&pcm, identifier)
                         : getFrame1(&pcm, identifier);
    if (!ok) break;

    pcm.is_end   = false;
    pcm.is_valid = true;

    int err = theMixer->sendData(dev,
                                 (mix == 0) ? outmixer0_send_callback
                                           : outmixer1_send_callback,
                                 pcm);
    if (err != OUTPUTMIXER_ECODE_OK) {
      printf("OutputMixer%u send error: %d\n", (unsigned)mix, err);
      break;
    }
  }

  (void)is_end;
}

static void outmixer0_send_callback(int32_t identifier, bool is_end)
{
  outmixer_send_callback(0, OutputMixer0, identifier, is_end);
}

static void outmixer1_send_callback(int32_t identifier, bool is_end)
{
  outmixer_send_callback(1, OutputMixer1, identifier, is_end);
}

/* ---------------- Attention / Done callbacks ---------------- */
static void outputmixer_done_callback(MsgQueId requester_dtq,
                                      MsgType reply_of,
                                      AsOutputMixDoneParam *done_param)
{
  (void)requester_dtq;
  (void)done_param;
#if BB_DEBUG_LOG
  printf(">> %x done\n", reply_of);
#else
  (void)reply_of;
#endif
}

static void attention_cb(const ErrorAttentionParam *atprm)
{
  puts("Attention!");
  if (atprm->error_code >= AS_ATTENTION_CODE_WARNING) {
    ErrEnd = true;
  }
}

/* ---------------- setup / loop ---------------- */
void setup()
{
  printf("setup() start\n");

  Serial.begin(115200);

  initMemoryPools();
  createStaticPools(MEM_LAYOUT_PLAYER);

  while (!theSD.begin()) {
    Serial.println("Insert SD card.");
  }

  theMixer  = OutputMixer::getInstance();
  theMixer->activateBaseband();

  theMixer->create(attention_cb);
  theMixer->setRenderingClkMode(OUTPUTMIXER_RNDCLK_NORMAL);

  // Activate both mixers (both HP output is OK)
  theMixer->activate(OutputMixer0, HPOutputDevice, outputmixer_done_callback);
  theMixer->activate(OutputMixer1, HPOutputDevice, outputmixer_done_callback);

  usleep(100 * 1000);

  theMixer->setVolume(-16, -16, -16);
  board_external_amp_mute_control(false);

  printf("setup() complete\n");
}

void loop()
{
  if (ErrEnd) {
    printf("Error End\n");
    goto stop_rendering;
  }

  // Menu operation
  if (Serial.available() > 0)
  {
    char c = (char)Serial.read();

    // 0-9: play sound0.raw ... sound9.raw
    if (c >= '0' && c <= '9') {
      uint8_t fileno = (uint8_t)(c - '0');
      (void)start(fileno);
    }
    else {
      switch (c) {
        case 's': // stop (Mixer stop)
          if (s_state == Active) {
            stop();
          }
          s_state = Stopping;
          break;

        default:
          break;
      }
    }
  }

  switch (s_state) {
    case Stopping:
      break;

    case Ready:
      break;

    case Active:
      // Feed frames for active slots
      for (int i = 0; i < 10; i++) {
        for (int slot = 0; slot < SLOTS; slot++) {
          if (!g_active[slot]) continue;
          (void)pushBridgeBuffer(slot); // EOF handled inside
        }
      }
      break;

    default:
      break;
  }

  return;

stop_rendering:
  board_external_amp_mute_control(true);

  for (int slot = 0; slot < SLOTS; slot++) {
    if (g_file[slot]) g_file[slot].close();
  }

  theMixer->deactivate(OutputMixer0);
  theMixer->deactivate(OutputMixer1);
  theMixer->end();

  puts("Exit Rendering");
  exit(1);
}
