#ifndef INCLUDE_RTSPCLIENT_SMARTPLUGIN_H_
#define INCLUDE_RTSPCLIENT_SMARTPLUGIN_H_

#include <fstream>
#include <iostream>
#include <memory>

#include "BasicUsageEnvironment.hh"
#include "H264VideoRTPSource.hh"
#include "liveMedia.hh"
#include "mediapipemanager/mediapipemanager.h"
#include "mediapipemanager/meidapipeline.h"

// Define a class to hold per-stream state that we maintain throughout each
// stream's lifetime:
class StreamClientState {
public:
  StreamClientState();
  virtual ~StreamClientState();

public:
  MediaSubsessionIterator *iter;
  MediaSession *session;
  MediaSubsession *subsession;
  TaskToken streamTimerTask;
  double duration;
};

// If you're streaming just a single stream (i.e., just from a single URL,
// once), then you can define and use just a single "StreamClientState"
// structure, as a global variable in your application.  However, because - in
// this demo application - we're showing how to play multiple streams,
// concurrently, we can't do that.  Instead, we have to have a separate
// "StreamClientState" structure for each "RTSPClient".  To do this, we subclass
// "RTSPClient", and add a "StreamClientState" field to the subclass:

class ourRTSPClient : public RTSPClient {
public:
  static ourRTSPClient *createNew(UsageEnvironment &env, char const *rtspURL,
                                  int verbosityLevel = 0,
                                  char const *applicationName = NULL,
                                  portNumBits tunnelOverHTTPPortNum = 0);
  void SetOutputFileName(const std::string &file_name);
  const std::string &GetOutputFileName(void);
  void SetChannel(int channel);
  int GetChannel(void) const;
  virtual ~ourRTSPClient();

protected:
  ourRTSPClient(UsageEnvironment &env, char const *rtspURL, int verbosityLevel,
                char const *applicationName, portNumBits tunnelOverHTTPPortNum);
  // called only by createNew();
  // virtual ~ourRTSPClient();

public:
  StreamClientState scs;

private:
  std::string file_name_;
  int channel_;
};

// Define a data sink (a subclass of "MediaSink") to receive the data for each
// subsession (i.e., each audio or video 'substream'). In practice, this might
// be a class (or a chain of classes) that decodes and then renders the incoming
// audio or video. Or it might be a "FileSink", for outputting the received data
// into a file (as is done by the "openRTSP" application). In this example code,
// however, we define a simple 'dummy' sink that receives incoming data, but
// does nothing with it.

class DummySink : public MediaSink {
public:
  static DummySink *
  createNew(UsageEnvironment &env,
            MediaSubsession &
                subsession, // identifies the kind of data that's being received
            char const *streamId = NULL, int buffer_size = 100000,
            int buffer_count = 8); // identifies the stream itself (optional)
  void SetFileName(const std::string &file_name);
  int SaveToFile(void *data, const int data_siz);
  void SetChannel(int channel);
  int GetChannel(void) const;
  void AddPipeLine(std::shared_ptr<horizon::vision::MediaPipeLine> pipe_line);

private:
  DummySink(UsageEnvironment &env, MediaSubsession &subsession,
            char const *streamId, int buffer_size, int buffer_count);
  // called only by "createNew()"
  virtual ~DummySink();

  static void afterGettingFrame(void *clientData, unsigned frameSize,
                                unsigned numTruncatedBytes,
                                struct timeval presentationTime,
                                unsigned durationInMicroseconds);
  void afterGettingFrame(unsigned frameSize, unsigned numTruncatedBytes,
                         struct timeval presentationTime,
                         unsigned durationInMicroseconds);

private:
  // redefined virtual functions:
  virtual Boolean continuePlaying();

private:
  MediaSubsession &subsession_;
  int buffer_size_;
  int buffer_count_;
  u_int8_t *buffers_vir_;
  uint64_t buffers_pyh_;

  char *stream_id_;
  std::string file_name_;
  int channel_;
  bool first_frame_;
  bool waiting_;
  uint64_t frame_count_;
  std::shared_ptr<horizon::vision::MediaPipeLine> pipe_line_;
};

// The main streaming routine (for each "rtsp://" URL):
ourRTSPClient *openURL(UsageEnvironment &env, char const *progName,
                       char const *rtspURL, const std::string &file_name);
void shutdownStream(RTSPClient *rtspClient, int exitCode = 1);

#endif  // INCLUDE_RTSPCLIENT_SMARTPLUGIN_H_