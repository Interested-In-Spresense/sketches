/*
 *  VariableSpeedPlayer.ino - A variable-speed audio player sample
 *  
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
 *  Foundation, Inc., 51 Franklin St, Fifth Floor, Boston,  MA 02110-1301  USA
 */

#include <SDHCI.h>
#include <MediaPlayer.h>
#include <OutputMixer.h>
#include <MemoryUtil.h>

SDClass theSD;

MediaPlayer *thePlayer;
OutputMixer *theMixer;

File myFile;

bool ErrEnd = false;

float speed = 1.0F;
float step = 0.02F;

/**
 * @brief Audio attention callback
 *
 * When audio internal error occurs, this function will be called back.
 */

static void attention_cb(const ErrorAttentionParam *atprm) {
  puts("Attention!");

  if (atprm->error_code >= AS_ATTENTION_CODE_WARNING) {
    ErrEnd = true;
  }
}

/**
 * @brief Mixer done callback procedure
 *
 * @param [in] requester_dtq    MsgQueId type
 * @param [in] reply_of         MsgType type
 * @param [in,out] done_param   AsOutputMixDoneParam type pointer
 */
static void outputmixer_done_callback(MsgQueId requester_dtq,
                                      MsgType reply_of,
                                      AsOutputMixDoneParam *done_param) {
  return;
}

/**
 * @brief Mixer data send callback procedure
 *
 * @param [in] identifier   Device identifier
 * @param [in] is_end       For normal request give false, for stop request give true
 */
static void outmixer_send_callback(int32_t identifier, bool is_end) {
  AsRequestNextParam next;

  next.type = (!is_end) ? AsNextNormalRequest : AsNextStopResRequest;

  AS_RequestNextPlayerProcess(AS_PLAYER_ID_0, &next);

  return;
}

/**
 * @brief Player done callback procedure
 *
 * @param [in] event        AsPlayerEvent type indicator
 * @param [in] result       Result
 * @param [in] sub_result   Sub result
 *
 * @return true on success, false otherwise
 */
static bool mediaplayer_done_callback(AsPlayerEvent event, uint32_t result, uint32_t sub_result) {
  printf("mp cb %x %lx %lx\n", event, result, sub_result);

  return true;
}


/**
 * @brief Player decode callback procedure
 *
 * @param [in] pcm_param    AsPcmDataParam type
 */

int fractional_resample(int16_t *in, int16_t *out, int in_samples, float ratio, float *phase)
{
  static int16_t prevL = 0, prevR = 0;
  static float x = *phase;

  if (in_samples <= 0) return 0;
  int out_samples = 0;

  while (x < (float)in_samples - 1) {
    int n = (int)x;
    float t = x - n;

    int16_t L0, R0, L1, R1;

    if (n < 0) {
      L0 = prevL;
      R0 = prevR;
      L1 = in[0];
      R1 = in[1];
    } else {
      L0 = in[2*n];
      R0 = in[2*n+1];
      L1 = in[2*(n+1)];
      R1 = in[2*(n+1)+1];
    }

    out[2*out_samples]     = (int16_t)((1.0f - t)*L0 + t*L1);
    out[2*out_samples + 1] = (int16_t)((1.0f - t)*R0 + t*R1);

    out_samples++;
    x += ratio;
  }

  x -= (float)in_samples;
  prevL = in[2*(in_samples-1)];
  prevR = in[2*(in_samples-1)+1];

  return out_samples;
}

int16_t buffer[1152 * 2 * 2];

void mediaplayer_decode_callback(AsPcmDataParam pcm_param) {

  static float phase = 0.0f;

  int samples = fractional_resample((int16_t *)pcm_param.mh.getPa(), buffer, pcm_param.size / 4, speed, &phase);

  printf("samples=%d,speed=%f\n",samples,speed);

  memcpy(pcm_param.mh.getPa(), buffer, samples * 4);
  pcm_param.size = samples * 4;

  theMixer->sendData(OutputMixer0,
                     outmixer_send_callback,
                     pcm_param);
}

/**
 * @brief Setup Player and Mixer
 *
 * Set output device to Speakers/Headphones <br>
 * Initialize main player to decode stereo mp3 stream with 48 kb/s sample rate <br>
 * System directory "/mnt/sd0/BIN" will be searched for MP3 decoder (MP3DEC file)
 * Open "Sound.mp3" file <br>
 * Set volume to -16.0 dB
 */
void setup() {
  /* Open serial communications and wait for port to open */
  Serial.begin(115200);
  while (!Serial) {
    ; /* wait for serial port to connect. Needed for native USB port only */
  }

  /* Initialize memory pools and message libs */

  initMemoryPools();
  createStaticPools(MEM_LAYOUT_PLAYER);

  thePlayer = MediaPlayer::getInstance();
  theMixer = OutputMixer::getInstance();

  /* Set rendering clock */

  theMixer->setRenderingClkMode(OUTPUTMIXER_RNDCLK_NORMAL);

  /* start audio system */

  thePlayer->begin();
  theMixer->begin();

  puts("initialization Audio Library");

  /* Create Objects */

  thePlayer->create(MediaPlayer::Player0, attention_cb);

  theMixer->create(attention_cb);

  /* Activate Player Object */

  thePlayer->activate(MediaPlayer::Player0, mediaplayer_done_callback);

  /* Activate Mixer Object.
   * Set output device to speaker with 2nd argument.
   * If you want to change the output device to I2S,
   * specify "I2SOutputDevice" as an argument.
   */

  theMixer->activate(OutputMixer0, HPOutputDevice, outputmixer_done_callback);

  usleep(100 * 1000);

  /*
   * Initialize main player to decode stereo mp3 stream with 48 kb/s sample rate
   * Search for MP3 codec in "/mnt/sd0/BIN" directory
   */
  thePlayer->init(MediaPlayer::Player0, AS_CODECTYPE_MP3, "/mnt/sd0/BIN", AS_SAMPLINGRATE_48000, AS_CHANNEL_STEREO);

  /* Initialize SD */
  while (!theSD.begin()) {
    /* wait until SD card is mounted. */
    Serial.println("Insert SD card.");
  }

  myFile = theSD.open("Sound.mp3");

  /* Verify file open */
  if (!myFile) {
    printf("File open error\n");
    exit(1);
  }
  printf("Open! 0x%08lx\n", (uint32_t)myFile);

  /* Send first frames to be decoded */
  err_t err = thePlayer->writeFrames(MediaPlayer::Player0, myFile);

  if (err != MEDIAPLAYER_ECODE_OK) {
    printf("File Read Error! =%d\n", err);
    myFile.close();
    exit(1);
  }

  puts("Play!");

  /* Main volume set to -16.0 dB, Main player and sub player set to 0 dB */
  theMixer->setVolume(-160, 0, 0);

  // Start Player
  thePlayer->start(MediaPlayer::Player0, mediaplayer_decode_callback);
}

#define MAX_SPEED 2.0F
#define MIN_SPEED 0.5F

/**
 * @brief Play audio frames until file ends
 */
void loop() {

  if (Serial.available() > 0) {
    char c = Serial.read();
    switch (c) {
      case '+' :
      speed += step;
      if (speed > MAX_SPEED) speed = MAX_SPEED;
      break;
      case '-' :
      speed -= step;
      if (speed < MIN_SPEED) speed = MIN_SPEED;
      break;
    }
  }

  /* Send new frames to decode in a loop until file ends */
  err_t err = thePlayer->writeFrames(MediaPlayer::Player0, myFile);

  /*  Tell when player file ends */
  if (err == MEDIAPLAYER_ECODE_FILEEND) {
    printf("Main player File End!\n");
  }

  /* Show error code from player and stop */
  if (err) {
    printf("Main player error code: %d\n", err);
    goto stop_player;
  }

  if (ErrEnd) {
    printf("Error End\n");
    goto stop_player;
  }

  /* This sleep is adjusted by the time to read the audio stream file.
   * Please adjust in according with the processing contents
   * being processed at the same time by Application.
   *
   * The usleep() function suspends execution of the calling thread for usec
   * microseconds. But the timer resolution depends on the OS system tick time
   * which is 10 milliseconds (10,000 microseconds) by default. Therefore,
   * it will sleep for a longer time than the time requested here.
   */

  usleep(40000);

  /* Don't go further and continue play */

  return;

stop_player:
  thePlayer->stop(MediaPlayer::Player0);
  myFile.close();
  thePlayer->deactivate(MediaPlayer::Player0);
  thePlayer->end();
  exit(1);
}
