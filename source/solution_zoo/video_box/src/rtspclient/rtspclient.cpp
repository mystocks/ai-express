#include "rtspclient/rtspclient.h"

#include "hb_comm_vdec.h"
#include "hb_vdec.h"
#include "hb_vp_api.h"
#include "hobotlog/hobotlog.hpp"

#include "mediapipemanager/mediapipemanager.h"
#include "rtspclient/rtspclient.h"

// Forward function definitions:

// RTSP 'response handlers':
void continueAfterDESCRIBE(RTSPClient *rtspClient, int resultCode,
                           char *resultString);
void continueAfterSETUP(RTSPClient *rtspClient, int resultCode,
                        char *resultString);
void continueAfterPLAY(RTSPClient *rtspClient, int resultCode,
                       char *resultString);

// Other event handler functions:
void subsessionAfterPlaying(
    void *clientData); // called when a stream's subsession (e.g., audio or
                       // video substream) ends
void subsessionByeHandler(
    void *clientData); // called when a RTCP "BYE" is received for a subsession
void streamTimerHandler(void *clientData);
// called at the end of a stream's expected duration (if the stream has not
// already signaled its end using a RTCP "BYE")

// Used to iterate through each stream's 'subsessions', setting up each one:
void setupNextSubsession(RTSPClient *rtspClient);

// Used to shut down and close a stream (including its "RTSPClient" object):
// void shutdownStream(RTSPClient* rtspClient, int exitCode = 1);

// A function that outputs a string that identifies each stream (for debugging
// output).  Modify this if you wish:
UsageEnvironment &operator<<(UsageEnvironment &env,
                             const RTSPClient &rtspClient) {
  return env << "[URL:\"" << rtspClient.url() << "\"]: ";
}

// A function that outputs a string that identifies each subsession (for
// debugging output).  Modify this if you wish:
UsageEnvironment &operator<<(UsageEnvironment &env,
                             const MediaSubsession &subsession) {
  return env << subsession.mediumName() << "/" << subsession.codecName();
}

// Implementation of "StreamClientState":

StreamClientState::StreamClientState()
    : iter(NULL), session(NULL), subsession(NULL), streamTimerTask(NULL),
      duration(0.0) {}

StreamClientState::~StreamClientState() {
  delete iter;
  if (session != NULL) {
    // We also need to delete "session", and unschedule "streamTimerTask" (if
    // set)
    UsageEnvironment &env = session->envir(); // alias

    env.taskScheduler().unscheduleDelayedTask(streamTimerTask);
    Medium::close(session);
  }
}

// Implementation of "ourRTSPClient":
void ourRTSPClient::SetOutputFileName(const std::string &file_name) {
  file_name_ = file_name;
};

const std::string &ourRTSPClient::GetOutputFileName(void) {
  return file_name_;
};

void ourRTSPClient::SetChannel(int channel) { channel_ = channel; };

int ourRTSPClient::GetChannel(void) const { return channel_; };

ourRTSPClient *ourRTSPClient::createNew(UsageEnvironment &env,
                                        char const *rtspURL, int verbosityLevel,
                                        char const *applicationName,
                                        portNumBits tunnelOverHTTPPortNum) {
  return new ourRTSPClient(env, rtspURL, verbosityLevel, applicationName,
                           tunnelOverHTTPPortNum);
}

ourRTSPClient::ourRTSPClient(UsageEnvironment &env, char const *rtspURL,
                             int verbosityLevel, char const *applicationName,
                             portNumBits tunnelOverHTTPPortNum)
    : RTSPClient(env, rtspURL, verbosityLevel, applicationName,
                 tunnelOverHTTPPortNum, -1) {}

ourRTSPClient::~ourRTSPClient() {}

// Implementation of "DummySink":
#define DUMMY_SINK_RECEIVE_BUFFER_SIZE 100000
void DummySink::SetFileName(const std::string &file_name) {
  file_name_ = file_name;
};

int DummySink::SaveToFile(void *data, const int data_size) {
  std::ofstream outfile;
  if (file_name_ == "") {
    return -1;
  }
  outfile.open(file_name_, std::ios::app | std::ios::out | std::ios::binary);
  outfile.write(reinterpret_cast<char *>(data), data_size);
  return 0;
}

void DummySink::SetChannel(int channel) { channel_ = channel; };

int DummySink::GetChannel(void) const { return channel_; };

DummySink *DummySink::createNew(UsageEnvironment &env,
                                MediaSubsession &subsession,
                                char const *streamId, int buffer_size,
                                int buffer_count) {
  return new DummySink(env, subsession, streamId, buffer_size, buffer_count);
}

DummySink::DummySink(UsageEnvironment &env, MediaSubsession &subsession,
                     char const *streamId, int buffer_size, int buffer_count)
    : MediaSink(env), subsession_(subsession), buffer_size_(buffer_size),
      buffer_count_(buffer_count), channel_(-1), first_frame_(true),
      waiting_(true), frame_count_(0) {
  int ret = 0;
  stream_id_ = strDup(streamId);
  ret = HB_SYS_Alloc(&buffers_pyh_, (void **)&buffers_vir_,
                     buffer_size_ * buffer_count_);
  if (ret != 0) {
    LOGE << "DummySink sys alloc failed";
  }
}

DummySink::~DummySink() {
  delete[] buffers_vir_;
  delete[] stream_id_;
}

void DummySink::afterGettingFrame(void *clientData, unsigned frameSize,
                                  unsigned numTruncatedBytes,
                                  struct timeval presentationTime,
                                  unsigned durationInMicroseconds) {
  DummySink *sink = (DummySink *)clientData;
  sink->afterGettingFrame(frameSize, numTruncatedBytes, presentationTime,
                          durationInMicroseconds);
}

// If you don't want to see debugging output for each received frame, then
// comment out the following line:
#define DEBUG_PRINT_EACH_RECEIVED_FRAME 1

void DummySink::afterGettingFrame(unsigned frameSize,
                                  unsigned numTruncatedBytes,
                                  struct timeval presentationTime,
                                  unsigned /*durationInMicroseconds*/) {
  // We've just received a frame of data.  (Optionally) print out information
  // about it:
#ifdef DEBUG_PRINT_EACH_RECEIVED_FRAME
  if (stream_id_ != NULL)
    envir() << "Stream \"" << stream_id_ << "\"; ";
  envir() << subsession_.mediumName() << "/" << subsession_.codecName()
          << ":\tReceived " << frameSize << " bytes";
  if (numTruncatedBytes > 0)
    envir() << " (with " << numTruncatedBytes << " bytes truncated)";
  char uSecsStr[6 + 1]; // used to output the 'microseconds' part of the
                        // presentation time
  sprintf(uSecsStr, "%06u", (unsigned)presentationTime.tv_usec);
  envir() << ".\tPresentation time: " << (int)presentationTime.tv_sec << "."
          << uSecsStr;
  if (subsession_.rtpSource() != NULL &&
      !subsession_.rtpSource()->hasBeenSynchronizedUsingRTCP()) {
    envir() << "!"; // mark the debugging output to indicate that this
                    // presentation time is not RTCP-synchronized
  }
#ifdef DEBUG_PRINT_NPT
  envir() << "\tNPT: " << subsession_.getNormalPlayTime(presentationTime);
#endif
  envir() << "\n";
#endif
  // unsigned char start_code[4] = {0x00, 0x00, 0x00, 0x01};
  u_int8_t *buffer =
      buffers_vir_ + (frame_count_ % buffer_count_) * buffer_size_;
  uint64_t buffer_phy =
      buffers_pyh_ + (frame_count_ % buffer_count_) * buffer_size_;
  if (waiting_) {
    if ((buffer[4] & 0x1F) == 0x07 && (buffer[4] & 0x80) == 0x00 &&
        (buffer[4] & 0x60) != 0x00) {
      waiting_ = false;
    } else {
      frame_count_++;
      // Then continue, to request the next frame of data:
      continuePlaying();
      return;
    }
  }
  VIDEO_STREAM_S pstStream;
  memset(&pstStream, 0, sizeof(VIDEO_STREAM_S));
  pstStream.pstPack.phy_ptr = buffer_phy;
  pstStream.pstPack.vir_ptr = (char *)buffer;
  pstStream.pstPack.pts = frame_count_;
  pstStream.pstPack.src_idx = frame_count_ % buffer_count_;
  pstStream.pstPack.size = frameSize + 4;
  pstStream.pstPack.stream_end = HB_FALSE;
  int ret = 0;
  ret = pipe_line_->Input(&pstStream);
  printf("HB_VDEC_SendStream\n");
  if (ret != 0) {
    LOGE << "HB_VDEC_SendStream failed.";
  }
  SaveToFile(buffer, frameSize + 4);
#if 0
  if (first_frame_) {
    // envir() << "first frame, insert frame\n";
    unsigned int num = 0;
    // unsigned char start_code[4] = {0x00, 0x00, 0x00, 0x01};
    subsession_.fmtp_spropparametersets();
    SPropRecord *sps = parseSPropParameterSets(subsession_.fmtp_spropparametersets(), num);
    // SaveToFile(start_code, 4);
    // SaveToFile(sps[0].sPropBytes, sps[0].sPropLength);
    // SaveToFile(start_code, 4);
    // SaveToFile(sps[1].sPropBytes, sps[1].sPropLength);
    // int full_frame_size = frameSize + 4 + sps[0].sPropLength + 4 + sps[1].sPropLength;
    int full_frame_size = 4 + sps[0].sPropLength + 4 + sps[1].sPropLength;
    VIDEO_STREAM_S pstStream;
    memset(&pstStream, 0, sizeof(VIDEO_STREAM_S));
    pstStream.pstPack.phy_ptr = buffer_phy;
    pstStream.pstPack.vir_ptr = (char *)buffer;
    pstStream.pstPack.pts = frame_count_;
    pstStream.pstPack.src_idx = frame_count_ % buffer_count_;
    pstStream.pstPack.size = full_frame_size;
    pstStream.pstPack.stream_end = HB_FALSE;

    int ret = 0;
    // ret = HB_VDEC_SendStream(0, &pstStream, 40);
    ret = pipe_line_->Input(&pstStream);
    printf("HB_VDEC_SendStream\n");
    if (ret != 0) {
        printf("HB_VDEC_SendStream failed %d\n", ret);
    }
    SaveToFile(buffer, full_frame_size);
    first_frame_ = false;
  } else {
    VIDEO_STREAM_S pstStream;
    memset(&pstStream, 0, sizeof(VIDEO_STREAM_S));
    pstStream.pstPack.phy_ptr = buffer_phy;
    pstStream.pstPack.vir_ptr = (char *)buffer;
    pstStream.pstPack.pts = frame_count_;
    pstStream.pstPack.src_idx = frame_count_ % buffer_count_;
    pstStream.pstPack.size = frameSize + 4;
    pstStream.pstPack.stream_end = HB_FALSE;

    int ret = 0;
    ret = pipe_line_->Input(&pstStream);
    printf("HB_VDEC_SendStream\n");
    if (ret != 0) {
        printf("HB_VDEC_SendStream failed %d\n", ret);
    }
    // memcpy((void *)buffer, start_code, 4);
    // SaveToFile(buffer, frameSize);
    SaveToFile(buffer, frameSize + 4);
  }
#endif

  frame_count_++;
  // Then continue, to request the next frame of data:
  continuePlaying();
}
void DummySink::AddPipeLine(
    std::shared_ptr<horizon::vision::MediaPipeLine> pipe_line) {
  pipe_line_ = pipe_line;
}

Boolean DummySink::continuePlaying() {
  if (fSource == NULL)
    return False; // sanity check (should not happen)
  unsigned char start_code[4] = {0x00, 0x00, 0x00, 0x01};

  u_int8_t *buffer =
      buffers_vir_ + (frame_count_ % buffer_count_) * buffer_size_;

  // Request the next frame of data from our input source. "afterGettingFrame()"
  // will get called later, when it arrives:
#if 0
  if (first_frame_) {
    unsigned int num = 0;
    subsession_.fmtp_spropparametersets();
    SPropRecord *sps = parseSPropParameterSets(subsession_.fmtp_spropparametersets(), num);
    memcpy((void *)buffer, start_code, 4);
    buffer += 4;
    memcpy((void *)buffer, sps[0].sPropBytes, sps[0].sPropLength);
    buffer += sps[0].sPropLength;
    memcpy((void *)buffer, start_code, 4);
    buffer += 4;
    memcpy((void *)buffer, sps[1].sPropBytes, sps[1].sPropLength);
    buffer += sps[1].sPropLength;
  } else {
    memcpy((void *)buffer, start_code, 4);
    buffer += 4;
  }
#endif
  memcpy((void *)buffer, start_code, 4);
  buffer += 4;

  fSource->getNextFrame(buffer, buffer_size_, afterGettingFrame, this,
                        onSourceClosure, this);
  return True;
}

#define RTSP_CLIENT_VERBOSITY_LEVEL                                            \
  1 // by default, print verbose output from each "RTSPClient"

static unsigned rtspClientCount =
    0; // Counts how many streams (i.e., "RTSPClient"s) are currently in use.

ourRTSPClient *openURL(UsageEnvironment &env, char const *progName,
                       char const *rtspURL, const std::string &file_name) {
  // Begin by creating a "RTSPClient" object.  Note that there is a separate
  // "RTSPClient" object for each stream that we wish to receive (even if more
  // than stream uses the same "rtsp://" URL).
  ourRTSPClient *rtspClient = ourRTSPClient::createNew(
      env, rtspURL, RTSP_CLIENT_VERBOSITY_LEVEL, progName);
  if (rtspClient == NULL) {
    env << "Failed to create a RTSP client for URL \"" << rtspURL
        << "\": " << env.getResultMsg() << "\n";
    return nullptr;
  }
  rtspClient->SetOutputFileName(file_name);
  rtspClient->SetChannel(rtspClientCount);

  env << "Set output file name:" << file_name.c_str();

  ++rtspClientCount;

  // Next, send a RTSP "DESCRIBE" command, to get a SDP description for the
  // stream. Note that this command - like all RTSP commands - is sent
  // asynchronously; we do not block, waiting for a response. Instead, the
  // following function call returns immediately, and we handle the RTSP
  // response later, from within the event loop:
  rtspClient->sendDescribeCommand(continueAfterDESCRIBE);
  return rtspClient;
}

// Implementation of the RTSP 'response handlers':

void continueAfterDESCRIBE(RTSPClient *rtspClient, int resultCode,
                           char *resultString) {
  do {
    UsageEnvironment &env = rtspClient->envir();                 // alias
    StreamClientState &scs = ((ourRTSPClient *)rtspClient)->scs; // alias

    if (resultCode != 0) {
      env << *rtspClient << "Failed to get a SDP description: " << resultString
          << "\n";
      delete[] resultString;
      break;
    }

    char *const sdpDescription = resultString;
    env << *rtspClient << "Got a SDP description:\n" << sdpDescription << "\n";

    // Create a media session object from this SDP description:
    scs.session = MediaSession::createNew(env, sdpDescription);
    delete[] sdpDescription; // because we don't need it anymore
    if (scs.session == NULL) {
      env << *rtspClient
          << "Failed to create a MediaSession object from the SDP description: "
          << env.getResultMsg() << "\n";
      break;
    } else if (!scs.session->hasSubsessions()) {
      env << *rtspClient
          << "This session has no media subsessions (i.e., no \"m=\" lines)\n";
      break;
    }

    // Then, create and set up our data source objects for the session.  We do
    // this by iterating over the session's 'subsessions', calling
    // "MediaSubsession::initiate()", and then sending a RTSP "SETUP" command,
    // on each one. (Each 'subsession' will have its own data source.)
    scs.iter = new MediaSubsessionIterator(*scs.session);
    // ad
    setupNextSubsession(rtspClient);
    return;
  } while (0);

  // An unrecoverable error occurred with this stream.
  shutdownStream(rtspClient);
}

// By default, we request that the server stream its data using RTP/UDP.
// If, instead, you want to request that the server stream via RTP-over-TCP,
// change the following to True:
#define REQUEST_STREAMING_OVER_TCP False

void setupNextSubsession(RTSPClient *rtspClient) {
  UsageEnvironment &env = rtspClient->envir();                 // alias
  StreamClientState &scs = ((ourRTSPClient *)rtspClient)->scs; // alias

  scs.subsession = scs.iter->next();
  if (scs.subsession != NULL) {
    if (!scs.subsession->initiate()) {
      env << *rtspClient << "Failed to initiate the \"" << *scs.subsession
          << "\" subsession: " << env.getResultMsg() << "\n";
      setupNextSubsession(
          rtspClient); // give up on this subsession; go to the next one
    } else {
      env << *rtspClient << "Initiated the \"" << *scs.subsession
          << "\" subsession (";
      if (scs.subsession->rtcpIsMuxed()) {
        env << "client port " << scs.subsession->clientPortNum();
      } else {
        env << "client ports " << scs.subsession->clientPortNum() << "-"
            << scs.subsession->clientPortNum() + 1;
      }
      env << ")\n";

      // Continue setting up this subsession, by sending a RTSP "SETUP" command:
      rtspClient->sendSetupCommand(*scs.subsession, continueAfterSETUP, False,
                                   REQUEST_STREAMING_OVER_TCP);
    }
    return;
  }

  // We've finished setting up all of the subsessions.  Now, send a RTSP "PLAY"
  // command to start the streaming:
  if (scs.session->absStartTime() != NULL) {
    // Special case: The stream is indexed by 'absolute' time, so send an
    // appropriate "PLAY" command:
    rtspClient->sendPlayCommand(*scs.session, continueAfterPLAY,
                                scs.session->absStartTime(),
                                scs.session->absEndTime());
  } else {
    scs.duration = scs.session->playEndTime() - scs.session->playStartTime();
    rtspClient->sendPlayCommand(*scs.session, continueAfterPLAY);
  }
}

void continueAfterSETUP(RTSPClient *rtspClient, int resultCode,
                        char *resultString) {
  do {
    UsageEnvironment &env = rtspClient->envir(); // alias
    ourRTSPClient *our_rtsp_client =
        reinterpret_cast<ourRTSPClient *>(rtspClient);
    StreamClientState &scs = our_rtsp_client->scs; // alias

    if (resultCode != 0) {
      env << *rtspClient << "Failed to set up the \"" << *scs.subsession
          << "\" subsession: " << resultString << "\n";
      break;
    }
    std::string media_name(scs.subsession->mediumName());
    if (media_name.compare("video") != 0) {
      env << *rtspClient << "Skip " << scs.subsession->mediumName()
          << " stream";
      break;
    }

    env << *rtspClient << "Set up the \"" << *scs.subsession
        << "\" subsession (";
    if (scs.subsession->rtcpIsMuxed()) {
      env << "client port " << scs.subsession->clientPortNum();
    } else {
      env << "client ports " << scs.subsession->clientPortNum() << "-"
          << scs.subsession->clientPortNum() + 1;
    }
    env << ")\n";

    // Having successfully setup the subsession, create a data sink for it, and
    // call "startPlaying()" on it. (This will prepare the data sink to receive
    // data; the actual flow of data from the client won't start happening until
    // later, after we've sent a RTSP "PLAY" command.)

    scs.subsession->sink =
        DummySink::createNew(env, *scs.subsession, rtspClient->url());
    // perhaps use your own custom "MediaSink" subclass instead
    if (scs.subsession->sink == NULL) {
      env << *rtspClient << "Failed to create a data sink for the \""
          << *scs.subsession << "\" subsession: " << env.getResultMsg() << "\n";
      break;
    }
    DummySink *dummy_sink_ptr =
        reinterpret_cast<DummySink *>(scs.subsession->sink);
    dummy_sink_ptr->SetFileName(our_rtsp_client->GetOutputFileName());
    dummy_sink_ptr->SetChannel(our_rtsp_client->GetChannel());
    dummy_sink_ptr->AddPipeLine(
        horizon::vision::MediaPipeManager::GetInstance()
            .GetPipeLine()[dummy_sink_ptr->GetChannel()]);

    env << *rtspClient << "Created a data sink for the \"" << *scs.subsession
        << "\" subsession\n";
    scs.subsession->miscPtr =
        rtspClient; // a hack to let subsession handler functions get the
                    // "RTSPClient" from the subsession
    scs.subsession->sink->startPlaying(*(scs.subsession->readSource()),
                                       subsessionAfterPlaying, scs.subsession);
    // Also set a handler to be called if a RTCP "BYE" arrives for this
    // subsession:
    if (scs.subsession->rtcpInstance() != NULL) {
      scs.subsession->rtcpInstance()->setByeHandler(subsessionByeHandler,
                                                    scs.subsession);
    }
  } while (0);
  delete[] resultString;

  // Set up the next subsession, if any:
  setupNextSubsession(rtspClient);
}

void continueAfterPLAY(RTSPClient *rtspClient, int resultCode,
                       char *resultString) {
  Boolean success = False;

  do {
    UsageEnvironment &env = rtspClient->envir();                 // alias
    StreamClientState &scs = ((ourRTSPClient *)rtspClient)->scs; // alias

    if (resultCode != 0) {
      env << *rtspClient << "Failed to start playing session: " << resultString
          << "\n";
      break;
    }

    // Set a timer to be handled at the end of the stream's expected duration
    // (if the stream does not already signal its end using a RTCP "BYE").  This
    // is optional.  If, instead, you want to keep the stream active - e.g., so
    // you can later 'seek' back within it and do another RTSP "PLAY" - then you
    // can omit this code. (Alternatively, if you don't want to receive the
    // entire stream, you could set this timer for some shorter value.)
    if (scs.duration > 0) {
      unsigned const delaySlop =
          2; // number of seconds extra to delay, after the stream's expected
             // duration.  (This is optional.)
      scs.duration += delaySlop;
      unsigned uSecsToDelay = (unsigned)(scs.duration * 1000000);
      scs.streamTimerTask = env.taskScheduler().scheduleDelayedTask(
          uSecsToDelay, (TaskFunc *)streamTimerHandler, rtspClient);
    }

    env << *rtspClient << "Started playing session";
    if (scs.duration > 0) {
      env << " (for up to " << scs.duration << " seconds)";
    }
    env << "...\n";

    success = True;
  } while (0);
  delete[] resultString;

  if (!success) {
    // An unrecoverable error occurred with this stream.
    shutdownStream(rtspClient);
  }
}

// Implementation of the other event handlers:

void subsessionAfterPlaying(void *clientData) {
  MediaSubsession *subsession = (MediaSubsession *)clientData;
  RTSPClient *rtspClient = (RTSPClient *)(subsession->miscPtr);

  // Begin by closing this subsession's stream:
  Medium::close(subsession->sink);
  subsession->sink = NULL;

  // Next, check whether *all* subsessions' streams have now been closed:
  MediaSession &session = subsession->parentSession();
  MediaSubsessionIterator iter(session);
  while ((subsession = iter.next()) != NULL) {
    if (subsession->sink != NULL)
      return; // this subsession is still active
  }

  // All subsessions' streams have now been closed, so shutdown the client:
  shutdownStream(rtspClient);
}

void subsessionByeHandler(void *clientData) {
  MediaSubsession *subsession = (MediaSubsession *)clientData;
  RTSPClient *rtspClient = (RTSPClient *)subsession->miscPtr;
  UsageEnvironment &env = rtspClient->envir(); // alias

  env << *rtspClient << "Received RTCP \"BYE\" on \"" << *subsession
      << "\" subsession\n";

  // Now act as if the subsession had closed:
  subsessionAfterPlaying(subsession);
}

void streamTimerHandler(void *clientData) {
  ourRTSPClient *rtspClient = (ourRTSPClient *)clientData;
  StreamClientState &scs = rtspClient->scs; // alias

  scs.streamTimerTask = NULL;

  // Shut down the stream:
  shutdownStream(rtspClient);
}

void shutdownStream(RTSPClient *rtspClient, int exitCode) {
  UsageEnvironment &env = rtspClient->envir();                 // alias
  StreamClientState &scs = ((ourRTSPClient *)rtspClient)->scs; // alias

  // First, check whether any subsessions have still to be closed:
  if (scs.session != NULL) {
    printf("scs.session\n");
    Boolean someSubsessionsWereActive = False;
    MediaSubsessionIterator iter(*scs.session);
    MediaSubsession *subsession;

    while ((subsession = iter.next()) != NULL) {
      printf("subsession name: %s \n", subsession->mediumName());
      if (subsession->sink != NULL) {
        printf("subsession->sink try close\n");
        Medium::close(subsession->sink);
        subsession->sink = NULL;
        printf("subsession->sink\n");
        if (subsession->rtcpInstance() != NULL) {
          subsession->rtcpInstance()->setByeHandler(
              NULL, NULL); // in case the server sends a RTCP "BYE" while
                           // handling "TEARDOWN"
        }
        someSubsessionsWereActive = True;
      }
    }
    printf("subsession->sink22\n");
    if (someSubsessionsWereActive) {
      // Send a RTSP "TEARDOWN" command, to tell the server to shutdown the
      // stream. Don't bother handling the response to the "TEARDOWN".
      rtspClient->sendTeardownCommand(*scs.session, NULL);
    }
  }

  env << *rtspClient << "Closing the stream.\n";
  Medium::close(rtspClient);
  // Note that this will also cause this stream's "StreamClientState" structure
  // to get reclaimed.

  if (--rtspClientCount == 0) {
    // The final stream has ended, so exit the application now.
    // (Of course, if you're embedding this code into your own application, you
    // might want to comment this out, and replace it with
    // "eventLoopWatchVariable = 1;", so that we leave the LIVE555 event loop,
    // and continue running "main()".) exit(exitCode);
  }
}
